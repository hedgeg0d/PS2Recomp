#include "Common.h"
#include "SIF.h"
#include "../Syscalls/RPC.h"
#include "../../ps2_iop_transport.h"
#include "runtime/ee_scheduler.h"
#include "runtime/ps2_address.h"

#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <unordered_map>
#include <vector>

namespace ps2_stubs
{
    void sceSifCmdIntrHdlr(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        TODO_NAMED("sceSifCmdIntrHdlr", rdram, ctx, runtime);
    }

    void sceSifLoadModule(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifLoadModule(rdram, ctx, runtime);
    }

    void sceSifSendCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t srcAddr = getRegU32(ctx, 7); // $a3
        // The EE calling convention provides the fifth and sixth integer
        // arguments in $t0/$t1.  They are not o32 stack arguments: the
        // caller-reserved stack area starts only after the eight-register
        // argument window has been exhausted.
        const uint32_t dstAddr = getRegU32(ctx, 8); // $t0
        const uint32_t size = getRegU32(ctx, 9);    // $t1
        if (size != 0u && srcAddr != 0u && dstAddr != 0u)
        {
            for (uint32_t i = 0; i < size; ++i)
            {
                const uint8_t *src = getConstMemPtr(rdram, srcAddr + i);
                uint8_t *dst = getMemPtr(rdram, dstAddr + i);
                if (!src || !dst)
                {
                    break;
                }
                *dst = *src;
            }
        }

        setReturnS32(ctx, 1);
    }

    namespace
    {
        struct Ps2SifDmaTransfer
        {
            uint32_t src = 0;
            uint32_t dest = 0;
            int32_t size = 0;
            int32_t attr = 0;
        };
        static_assert(sizeof(Ps2SifDmaTransfer) == 16u, "Unexpected SIF DMA descriptor size");

        struct RawSifRpcBinding
        {
            uint32_t sid = 0u;
            uint32_t serverAddress = 0u;
            uint32_t serverBuffer = 0u;
        };

        std::mutex g_sifDmaTransferMutex;
        uint32_t g_nextSifDmaTransferId = 1u;
        std::mutex g_sifCmdStateMutex;
        std::mutex g_sifHeapMutex;
        std::unordered_map<uint32_t, uint32_t> g_sifRegs;
        std::unordered_map<uint32_t, uint32_t> g_sifSregs;
        std::unordered_map<uint32_t, uint32_t> g_sifCmdHandlers;
        std::unordered_map<PS2Runtime *, std::unordered_map<uint32_t, RawSifRpcBinding>> g_sifRawRpcBindings;
        std::map<uint32_t, uint32_t> g_sifHeapAllocations;
        std::array<uint8_t, kIopHeapLimit - kIopHeapBase> g_sifHeapStorage{};
        uint32_t g_sifCmdBuffer = 0u;
        uint32_t g_sifSysCmdBuffer = 0u;
        uint32_t g_sifEeReceiveBuffer = 0u;
        bool g_sifCmdInitialized = false;
        bool g_sifBootEndPending = false;
        uint32_t g_sifGetRegLogCount = 0u;
        uint32_t g_sifSetRegLogCount = 0u;
        uint32_t g_sifDmaCommandLogCount = 0u;

        constexpr uint32_t kSifRegBootStatus = 0x4u;
        constexpr uint32_t kSifRegMainAddr = 0x80000000u;
        constexpr uint32_t kSifRegSubAddr = 0x80000001u;
        constexpr uint32_t kSifRegMsCom = 0x80000002u;
        constexpr uint32_t kSifBootReadyMask = 0x00020000u;
        constexpr uint32_t kSifCmdSetSreg = 0x80000001u;
        constexpr uint32_t kSifCmdInit = 0x80000002u;
        constexpr uint32_t kSifCmdReset = 0x80000003u;
        constexpr uint32_t kSifCmdRpcEnd = 0x80000008u;
        constexpr uint32_t kSifCmdRpcBind = 0x80000009u;
        constexpr uint32_t kSifCmdRpcCall = 0x8000000Au;
        constexpr uint32_t kSifRawRpcServerStride = 0x80u;
        // SIF RPC payloads are bounded by the command transport. Reserving a
        // whole MiB for every bound needlessly exhausts the emulated IOP
        // heap as soon as several standard services are active.
        constexpr uint32_t kSifRawRpcBufferSize = 64u * 1024u;

        uint32_t allocateSifHeapBlock(uint32_t requestSize);
        bool freeSifHeapBlock(uint32_t addr);

        void releaseRawSifRpcBindings(PS2Runtime *runtimeFilter)
        {
            std::vector<uint32_t> buffers;
            {
                std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                for (auto it = g_sifRawRpcBindings.begin(); it != g_sifRawRpcBindings.end();)
                {
                    if (runtimeFilter && it->first != runtimeFilter)
                    {
                        ++it;
                        continue;
                    }

                    for (const auto &[clientAddress, binding] : it->second)
                    {
                        (void)clientAddress;
                        if (binding.serverBuffer != 0u)
                        {
                            buffers.push_back(binding.serverBuffer);
                        }
                    }
                    it = g_sifRawRpcBindings.erase(it);
                }
            }

            for (const uint32_t buffer : buffers)
            {
                (void)freeSifHeapBlock(buffer);
            }
        }

        void seedDefaultSifRegsLocked()
        {
            g_sifRegs.clear();
            g_sifSregs.clear();
            g_sifCmdHandlers.clear();
            g_sifCmdBuffer = 0u;
            g_sifSysCmdBuffer = 0u;
            g_sifEeReceiveBuffer = 0u;
            g_sifCmdInitialized = false;
            g_sifBootEndPending = false;
            g_sifGetRegLogCount = 0u;
            g_sifSetRegLogCount = 0u;
            g_sifDmaCommandLogCount = 0u;

            g_sifRegs[kSifRegBootStatus] = kSifBootReadyMask;
            g_sifRegs[kSifRegMainAddr] = 0u;
            g_sifRegs[kSifRegSubAddr] = 0u;
            g_sifRegs[kSifRegMsCom] = 0u;
        }

        bool shouldTraceSifReg(uint32_t reg)
        {
            switch (reg)
            {
            case 0x2u:
            case 0x4u:
            case 0x80000000u:
            case 0x80000001u:
            case 0x80000002u:
                return true;
            default:
                return false;
            }
        }

        struct SifStateInitializer
        {
            SifStateInitializer()
            {
                std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                seedDefaultSifRegsLocked();
            }
        } g_sifStateInitializer;

        uint32_t allocateSifDmaTransferId()
        {
            std::lock_guard<std::mutex> lock(g_sifDmaTransferMutex);
            uint32_t id = g_nextSifDmaTransferId++;
            if (id == 0u)
            {
                id = g_nextSifDmaTransferId++;
            }
            return id;
        }

        uint32_t alignIopHeapSize(uint32_t size)
        {
            return (size + (kIopHeapAlign - 1u)) & ~(kIopHeapAlign - 1u);
        }

        uint32_t allocateSifHeapBlock(uint32_t requestSize)
        {
            const uint32_t alignedSize = alignIopHeapSize(requestSize);
            if (alignedSize == 0u)
            {
                return 0u;
            }

            std::lock_guard<std::mutex> lock(g_sifHeapMutex);
            uint32_t candidate = kIopHeapBase;
            for (const auto &[addr, size] : g_sifHeapAllocations)
            {
                if (candidate + alignedSize <= addr)
                {
                    break;
                }

                const uint32_t blockEnd = alignIopHeapSize(addr + size);
                if (blockEnd > candidate)
                {
                    candidate = blockEnd;
                }
            }

            if (candidate < kIopHeapBase || candidate + alignedSize > kIopHeapLimit)
            {
                return 0u;
            }

            g_sifHeapAllocations[candidate] = alignedSize;
            std::fill_n(g_sifHeapStorage.data() + (candidate - kIopHeapBase),
                        alignedSize,
                        uint8_t{0});
            g_iopHeapNext = candidate + alignedSize;
            return candidate;
        }

        bool freeSifHeapBlock(uint32_t addr)
        {
            std::lock_guard<std::mutex> lock(g_sifHeapMutex);
            const auto it = g_sifHeapAllocations.find(addr);
            if (it == g_sifHeapAllocations.end())
            {
                return false;
            }

            g_sifHeapAllocations.erase(it);
            if (g_sifHeapAllocations.empty())
            {
                g_iopHeapNext = kIopHeapBase;
            }
            return true;
        }

        void resetSifHeapState()
        {
            std::lock_guard<std::mutex> lock(g_sifHeapMutex);
            g_sifHeapAllocations.clear();
            g_sifHeapStorage.fill(0u);
            g_iopHeapNext = kIopHeapBase;
        }

        bool isAllocatedSifHeapRangeLocked(uint32_t address, size_t size)
        {
            if (address < kIopHeapBase || address >= kIopHeapLimit || size > static_cast<size_t>(kIopHeapLimit - address))
            {
                return false;
            }

            auto it = g_sifHeapAllocations.upper_bound(address);
            if (it == g_sifHeapAllocations.begin())
            {
                return false;
            }
            --it;

            const uint64_t allocationEnd = static_cast<uint64_t>(it->first) + it->second;
            const uint64_t rangeEnd = static_cast<uint64_t>(address) + size;
            return address >= it->first && rangeEnd <= allocationEnd;
        }

        bool isCopyableGuestAddress(uint32_t addr)
        {
            if (Ps2AddressInRange(addr, PS2_SCRATCHPAD_BASE, PS2_SCRATCHPAD_SIZE))
            {
                return true;
            }

            if (addr < PS2_EE_UNCACHED_RAM_MIRROR_BASE)
            {
                return true;
            }

            if (Ps2IsUncachedRamMirrorAddress(addr))
            {
                return true;
            }

            if (Ps2IsKseg01Address(addr))
            {
                return true;
            }

            return false;
        }

        bool canCopyAddressRange(const uint8_t *rdram, uint32_t address, uint32_t sizeBytes)
        {
            if (isSifIopHeapRange(address, sizeBytes))
            {
                return true;
            }
            if (isSifIopHeapAddress(address) || !rdram)
            {
                return false;
            }
            if (sizeBytes == 0u)
            {
                return true;
            }
            if (sizeBytes - 1u > std::numeric_limits<uint32_t>::max() - address)
            {
                return false;
            }
            for (uint32_t i = 0u; i < sizeBytes; ++i)
            {
                const uint32_t byteAddress = address + i;
                if (!isCopyableGuestAddress(byteAddress) || getConstMemPtr(rdram, byteAddress) == nullptr)
                {
                    return false;
                }
            }
            return true;
        }

        bool canCopyGuestByteRange(const uint8_t *rdram, uint32_t dstAddr, uint32_t srcAddr, uint32_t sizeBytes)
        {
            return canCopyAddressRange(rdram, srcAddr, sizeBytes) && canCopyAddressRange(rdram, dstAddr, sizeBytes);
        }

        bool readSifBytes(const uint8_t *rdram, uint32_t address, void *destination, size_t size)
        {
            if ((!destination && size != 0u) || !rdram)
            {
                return false;
            }
            if (size == 0u)
            {
                return true;
            }
            if (size - 1u > std::numeric_limits<uint32_t>::max() - address)
            {
                return false;
            }
            if (isSifIopHeapRange(address, size))
            {
                return readSifIopHeap(address, destination, size);
            }

            uint8_t *bytes = static_cast<uint8_t *>(destination);
            for (size_t i = 0u; i < size; ++i)
            {
                const uint8_t *source = getConstMemPtr(rdram, address + static_cast<uint32_t>(i));
                if (!source)
                {
                    return false;
                }
                bytes[i] = *source;
            }
            return true;
        }

        bool readSifU32(const uint8_t *rdram, uint32_t address, uint32_t &value)
        {
            return readSifBytes(rdram, address, &value, sizeof(value));
        }

        bool writeGuestBytes(uint8_t *rdram, uint32_t address, const void *source, size_t size)
        {
            // TEMP-EXPERIMENT: catch DMA-side d0 writer. Revert.
            if (size != 0u && address < 0x5611E0u && address + size > 0x5611C0u)
            {
                static int sifw = 0;
                if (sifw < 20)
                {
                    ++sifw;
                    const uint8_t *b = static_cast<const uint8_t *>(source);
                    std::fprintf(stderr,
                                 "[sifw] addr=0x%08x size=0x%zx data=%02x%02x%02x%02x\n",
                                 address,
                                 size,
                                 size > 0 ? b[0] : 0,
                                 size > 1 ? b[1] : 0,
                                 size > 2 ? b[2] : 0,
                                 size > 3 ? b[3] : 0);
                }
            }
            if ((!source && size != 0u) || !rdram)
            {
                return false;
            }
            if (size != 0u && size - 1u > std::numeric_limits<uint32_t>::max() - address)
            {
                return false;
            }
            if (isSifIopHeapRange(address, size))
            {
                return writeSifIopHeap(address, source, size);
            }
            const uint8_t *bytes = static_cast<const uint8_t *>(source);
            for (size_t i = 0u; i < size; ++i)
            {
                uint8_t *destination = getMemPtr(rdram, address + static_cast<uint32_t>(i));
                if (!destination)
                {
                    return false;
                }
                *destination = bytes[i];
            }
            return true;
        }

        bool writeSifU32(uint8_t *rdram, uint32_t address, uint32_t value)
        {
            return writeGuestBytes(rdram, address, &value, sizeof(value));
        }

        bool copyGuestByteRange(uint8_t *rdram,
                                uint32_t dstAddr,
                                uint32_t srcAddr,
                                uint32_t sizeBytes);

        bool writeRawSifRpcEnd(uint8_t *rdram,
                               uint32_t packetAddress,
                               uint32_t clientAddress,
                               uint32_t command,
                               uint32_t serverAddress,
                               uint32_t serverBuffer,
                               bool &generatedReply)
        {
            generatedReply = false;
            uint32_t receiveBuffer = 0u;
            {
                std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                receiveBuffer = g_sifEeReceiveBuffer;
            }
            if (receiveBuffer == 0u || !canCopyAddressRange(rdram, receiveBuffer, 64u))
            {
                return false;
            }

            // This is the SifRpcRendPkt layout used by the PS2SDK IOP-side
            // sifrpc implementation. The guest's normal SIF command handler
            // owns completion, semaphore signaling, and packet recycling.
            const uint32_t reply[16] = {
                64u,
                0u,
                kSifCmdRpcEnd,
                0u,
                0u,
                packetAddress,
                0u,
                clientAddress,
                command,
                serverAddress,
                serverBuffer,
                0u,
                0u,
                0u,
                0u,
                0u,
            };
            generatedReply = writeGuestBytes(rdram, receiveBuffer, reply, sizeof(reply));
            return generatedReply;
        }

        bool writeRawSifRpcDirectCompletion(uint8_t *rdram,
                                            uint32_t packetAddress,
                                            uint32_t clientAddress,
                                            bool &generatedReply)
        {
            generatedReply = false;
            if (packetAddress == 0u || !canCopyAddressRange(rdram, packetAddress, 64u))
            {
                return false;
            }

            // The IOP-side sceSifExecRequest DMA-copies this render packet
            // directly to the EE packet allocated by sceSifCallRpc when the
            // call has no receive buffer. This is used both by NOWAIT calls
            // without an end callback and as the second completion step for
            // the callback-enabled no-receive path.
            const uint32_t completion[16] = {
                64u,
                0u,
                0u,
                0u,
                0u,
                packetAddress,
                0u,
                clientAddress,
                kSifCmdRpcCall,
                0u,
                0u,
                0u,
                0u,
                0u,
                0u,
                0u,
            };
            generatedReply = writeGuestBytes(rdram,
                                              packetAddress,
                                              completion,
                                              sizeof(completion));
            return generatedReply;
        }

        bool handleRawSifRpcBind(uint8_t *rdram,
                                 PS2Runtime *runtime,
                                 uint32_t sourceAddress,
                                 uint32_t sizeBytes,
                                 bool &generatedReply)
        {
            generatedReply = false;
            if (!runtime || sizeBytes < 0x24u)
            {
                return false;
            }

            uint32_t packetAddress = 0u;
            uint32_t clientAddress = 0u;
            uint32_t sid = 0u;
            if (!readSifU32(rdram, sourceAddress + 0x14u, packetAddress) ||
                !readSifU32(rdram, sourceAddress + 0x1Cu, clientAddress) ||
                !readSifU32(rdram, sourceAddress + 0x20u, sid))
            {
                return false;
            }
            RawSifRpcBinding binding{};
            if (PS2IopTransport::hasRpcService(runtime, sid) && clientAddress != 0u)
            {
                uint32_t previousBuffer = 0u;
                {
                    std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                    const auto runtimeIt = g_sifRawRpcBindings.find(runtime);
                    if (runtimeIt != g_sifRawRpcBindings.end())
                    {
                        const auto it = runtimeIt->second.find(clientAddress);
                        if (it != runtimeIt->second.end() && it->second.sid == sid)
                        {
                            binding = it->second;
                        }
                        else if (it != runtimeIt->second.end())
                        {
                            previousBuffer = it->second.serverBuffer;
                            runtimeIt->second.erase(it);
                        }
                    }
                }
                if (previousBuffer != 0u)
                {
                    (void)freeSifHeapBlock(previousBuffer);
                }

                if (binding.serverAddress == 0u)
                {
                    binding.sid = sid;
                    binding.serverAddress = PS2IopTransport::allocateIopHandle(
                        runtime,
                        ps2x::iop::IopHandleKind::RpcServer);
                }

                bool allocatedBuffer = false;
                if (binding.serverAddress != 0u && binding.serverBuffer == 0u)
                {
                    // RPC server data lives in IOP memory. Keeping it in
                    // the SIF IOP heap also works when the EE program has
                    // no spare general-purpose heap after startup setup.
                    binding.serverBuffer = allocateSifHeapBlock(kSifRawRpcBufferSize);
                    allocatedBuffer = binding.serverBuffer != 0u;
                }

            if (binding.serverAddress != 0u && binding.serverBuffer != 0u &&
                    canCopyAddressRange(rdram, binding.serverAddress, kSifRawRpcServerStride) &&
                    isSifIopHeapRange(binding.serverBuffer, kSifRawRpcBufferSize))
                {
                    std::array<uint8_t, kSifRawRpcServerStride> server{};
                    std::memcpy(server.data() + 0x00u, &sid, sizeof(sid));
                    std::memcpy(server.data() + 0x08u, &binding.serverBuffer, sizeof(binding.serverBuffer));
                    if (writeGuestBytes(rdram,
                                        binding.serverAddress,
                                        server.data(),
                                        server.size()))
                    {
                        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                        g_sifRawRpcBindings[runtime][clientAddress] = binding;
                    }
                    else
                    {
                        if (allocatedBuffer)
                        {
                            (void)freeSifHeapBlock(binding.serverBuffer);
                        }
                        binding = {};
                    }
                }
                else
                {
                    if (allocatedBuffer)
                    {
                        (void)freeSifHeapBlock(binding.serverBuffer);
                    }
                    binding = {};
                }
            }
            else if (clientAddress != 0u)
            {
                uint32_t previousBuffer = 0u;
                {
                    std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                    const auto runtimeIt = g_sifRawRpcBindings.find(runtime);
                    if (runtimeIt != g_sifRawRpcBindings.end())
                    {
                        const auto bindingIt = runtimeIt->second.find(clientAddress);
                        if (bindingIt != runtimeIt->second.end())
                        {
                            previousBuffer = bindingIt->second.serverBuffer;
                            runtimeIt->second.erase(bindingIt);
                        }
                        if (runtimeIt->second.empty())
                        {
                            g_sifRawRpcBindings.erase(runtimeIt);
                        }
                    }
                }
                if (previousBuffer != 0u)
                {
                    (void)freeSifHeapBlock(previousBuffer);
                }
            }

            if (clientAddress != 0u)
            {
                // sceSifRpc's bind completion installs the server handle in
                // the EE-side client object before the caller issues calls.
                // Keep that shared structure coherent for raw SIF users.
                (void)writeSifU32(rdram, clientAddress + 0x24u, binding.serverAddress);
            }

            // Unknown SIDs intentionally receive a normal null bind result;
            // fabricating a server would hide missing IOP modules and make
            // raw RPC behavior depend on the game being run.
            return writeRawSifRpcEnd(rdram,
                                     packetAddress,
                                     clientAddress,
                                     kSifCmdRpcBind,
                                     binding.serverAddress,
                                     binding.serverBuffer,
                                     generatedReply);
        }

        bool handleRawSifRpcCall(uint8_t *rdram,
                                 PS2Runtime *runtime,
                                 uint32_t sourceAddress,
                                 uint32_t sizeBytes,
                                 bool &generatedReply)
        {
            generatedReply = false;
            if (!runtime || sizeBytes < 0x38u)
            {
                return false;
            }

            uint32_t packetAddress = 0u;
            uint32_t clientAddress = 0u;
            uint32_t rpcNumber = 0u;
            uint32_t sendSize = 0u;
            uint32_t receiveBuffer = 0u;
            uint32_t receiveSize = 0u;
            uint32_t rmode = 0u;
            uint32_t serverAddress = 0u;
            uint32_t rpcId = 0u;
            if (!readSifU32(rdram, sourceAddress + 0x14u, packetAddress) ||
                !readSifU32(rdram, sourceAddress + 0x18u, rpcId) ||
                !readSifU32(rdram, sourceAddress + 0x1Cu, clientAddress) ||
                !readSifU32(rdram, sourceAddress + 0x20u, rpcNumber) ||
                !readSifU32(rdram, sourceAddress + 0x24u, sendSize) ||
                !readSifU32(rdram, sourceAddress + 0x28u, receiveBuffer) ||
                !readSifU32(rdram, sourceAddress + 0x2Cu, receiveSize) ||
                !readSifU32(rdram, sourceAddress + 0x30u, rmode) ||
                !readSifU32(rdram, sourceAddress + 0x34u, serverAddress))
            {
                return false;
            }
            RawSifRpcBinding binding{};
            {
                std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                const auto runtimeIt = g_sifRawRpcBindings.find(runtime);
                if (runtimeIt != g_sifRawRpcBindings.end())
                {
                    const auto it = runtimeIt->second.find(clientAddress);
                    if (it != runtimeIt->second.end() && it->second.serverAddress == serverAddress)
                    {
                        binding = it->second;
                    }
                }
            }
            if (binding.serverAddress == 0u)
            {
                // TEMP-EXPERIMENT: log dropped raw RPC calls (e.g. libmc GetInfo
                // on an unbound client). Revert.
                {
                    static int dropCount = 0;
                    if (dropCount < 40)
                    {
                        ++dropCount;
                        uint32_t cSid = 0u, cServer = 0u;
                        (void)readSifU32(rdram, clientAddress + 0x00u, cSid);
                        (void)readSifU32(rdram, clientAddress + 0x24u, cServer);
                        std::cerr << "[sifdrop] n=" << std::dec << dropCount
                                  << " rpcId=0x" << std::hex << rpcId
                                  << " fn=0x" << rpcNumber
                                  << " client=0x" << clientAddress
                                  << " server(pkt)=0x" << serverAddress
                                  << " clientSid=0x" << cSid
                                  << " clientServer=0x" << cServer
                                  << " send=" << std::dec << sendSize
                                  << " recv=0x" << std::hex << receiveBuffer << "/" << std::dec << receiveSize
                                  << std::endl;
                    }
                }
                return false;
            }

            // sceSifExecRequest fills these fields in the IOP-side server
            // record before invoking the registered service. Services and
            // their callbacks may inspect the record while handling a call.
            (void)writeSifU32(rdram, binding.serverAddress + 0x1Cu, clientAddress);
            (void)writeSifU32(rdram, binding.serverAddress + 0x20u, packetAddress);
            (void)writeSifU32(rdram, binding.serverAddress + 0x24u, rpcNumber);
            (void)writeSifU32(rdram, binding.serverAddress + 0x28u, receiveBuffer);
            (void)writeSifU32(rdram, binding.serverAddress + 0x2Cu, receiveSize);
            (void)writeSifU32(rdram, binding.serverAddress + 0x30u, rmode);
            (void)writeSifU32(rdram, binding.serverAddress + 0x34u, rpcId);

            uint32_t mode = 0u;
            uint32_t endFunction = 0u;
            uint32_t endParameter = 0u;
            (void)readSifU32(rdram, clientAddress + 0x0Cu, mode);
            (void)readSifU32(rdram, clientAddress + 0x1Cu, endFunction);
            (void)readSifU32(rdram, clientAddress + 0x20u, endParameter);

            ps2x::iop::RpcRequest request{};
            request.clientAddress = clientAddress;
            request.serverAddress = binding.serverAddress;
            request.serverBuffer = binding.serverBuffer;
            request.sid = binding.sid;
            request.function = rpcNumber;
            request.mode = mode;
            request.send = {binding.serverBuffer, sendSize};
            request.receive = {receiveBuffer, receiveSize};
            request.endFunction = endFunction;
            request.endParameter = endParameter;

            // Temporary narrow trace for the generic file-control stream
            // while validating asynchronous read progress.  Keep it bounded
            // so it cannot become a hot-path logging source.
            // TEMP-EXPERIMENT: also trace mcserv calls (rmode/endfunc). Revert.
            if ((binding.sid == 0x00010000u && rpcNumber >= 2u && rpcNumber <= 6u) ||
                binding.sid == 0x80000400u)
            {
                static uint32_t filectrlRpcProbeCount = 0u;
                if (filectrlRpcProbeCount++ < 160u)
                {
                    uint32_t w0 = 0u;
                    uint32_t w1 = 0u;
                    uint32_t w2 = 0u;
                    uint32_t w3 = 0u;
                    (void)readSifU32(rdram, request.send.address + 0x00u, w0);
                    (void)readSifU32(rdram, request.send.address + 0x04u, w1);
                    (void)readSifU32(rdram, request.send.address + 0x08u, w2);
                    (void)readSifU32(rdram, request.send.address + 0x0Cu, w3);
                    std::cerr << "[probe:filectrl-rpc] fn=0x" << std::hex << rpcNumber
                              << " sid=0x" << binding.sid
                              << " rmode=0x" << rmode
                              << " end=0x" << endFunction << "/0x" << endParameter
                              << " recv=0x" << receiveBuffer << "/" << std::dec << receiveSize
                              << " w0=0x" << std::hex << w0 << " w1=0x" << w1
                              << " w2=0x" << w2 << " w3=0x" << w3
                              << std::dec << '\n';
                }
            }

            PS2_IF_AGRESSIVE_LOGS({
                static uint32_t rpcTraceCount = 0u;
                if (rpcTraceCount++ < 160u)
                {
                    uint32_t sendWord0 = 0u;
                    uint32_t sendWord1 = 0u;
                    uint32_t sendWord2 = 0u;
                    uint32_t sendWord3 = 0u;
                    (void)readSifU32(rdram, request.send.address + 0x00u, sendWord0);
                    (void)readSifU32(rdram, request.send.address + 0x04u, sendWord1);
                    (void)readSifU32(rdram, request.send.address + 0x08u, sendWord2);
                    (void)readSifU32(rdram, request.send.address + 0x0Cu, sendWord3);
                    std::cerr << "[sif:rpc-call] sid=0x" << std::hex << binding.sid
                              << " fn=0x" << rpcNumber
                              << " send=0x" << sendSize
                              << " recv=0x" << receiveBuffer
                              << " recvSize=0x" << receiveSize
                              << " mode=0x" << rmode
                              << " client=0x" << clientAddress
                              << " server=0x" << serverAddress
                              << " w0=0x" << sendWord0
                              << " w1=0x" << sendWord1
                              << " w2=0x" << sendWord2
                              << " w3=0x" << sendWord3
                              << " end=0x" << endFunction
                              << " endArg=0x" << endParameter
                              << std::dec << std::endl;
                }
            });

            const ps2x::iop::RpcResult result =
                PS2IopTransport::handleRpc(runtime, rdram, nullptr, request);
            if (binding.sid == 0x00010000u && rpcNumber >= 2u && rpcNumber <= 6u)
            {
                static uint32_t filectrlCompletionProbeCount = 0u;
                if (filectrlCompletionProbeCount++ < 160u)
                {
                    uint32_t p0 = 0u;
                    uint32_t p1 = 0u;
                    uint32_t p2 = 0u;
                    uint32_t p5 = 0u;
                    uint32_t p7 = 0u;
                    uint32_t sifReceiveBuffer = 0u;
                    {
                        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                        sifReceiveBuffer = g_sifEeReceiveBuffer;
                    }
                    (void)readSifU32(rdram, packetAddress + 0x00u, p0);
                    (void)readSifU32(rdram, packetAddress + 0x04u, p1);
                    (void)readSifU32(rdram, packetAddress + 0x08u, p2);
                    (void)readSifU32(rdram, packetAddress + 0x14u, p5);
                    (void)readSifU32(rdram, packetAddress + 0x1Cu, p7);
                    uint32_t sifP2 = 0u;
                    if (sifReceiveBuffer != 0u)
                    {
                        (void)readSifU32(rdram, sifReceiveBuffer + 0x08u, sifP2);
                    }
                    std::cerr << "[probe:filectrl-completion] fn=0x" << std::hex << rpcNumber
                              << " rmode=0x" << rmode
                              << " end=0x" << endFunction
                              << " arg=0x" << endParameter
                              << " packet=0x" << packetAddress
                              << " recv=0x" << receiveBuffer
                              << " recvSize=0x" << receiveSize
                              << " sifRecv=0x" << sifReceiveBuffer
                              << " reply=[0x" << p0 << ",0x" << p1 << ",0x" << p2
                              << ",p5=0x" << p5 << ",p7=0x" << p7 << "]"
                              << " sifP2=0x" << sifP2
                              << " result=" << std::dec << result.handled
                              << " addr=0x" << std::hex << result.resultAddress
                              << std::dec << '\n';
                }
            }
            PS2_IF_AGRESSIVE_LOGS({
                static uint32_t rpcResultTraceCount = 0u;
                if (rpcResultTraceCount++ < 160u)
                {
                    uint32_t resultWord0 = 0u;
                    uint32_t resultWord1 = 0u;
                    uint32_t resultWord2 = 0u;
                    uint32_t resultWord3 = 0u;
                    if (receiveBuffer != 0u)
                    {
                        (void)readSifU32(rdram, receiveBuffer + 0x00u, resultWord0);
                        (void)readSifU32(rdram, receiveBuffer + 0x04u, resultWord1);
                        (void)readSifU32(rdram, receiveBuffer + 0x08u, resultWord2);
                        (void)readSifU32(rdram, receiveBuffer + 0x0Cu, resultWord3);
                    }
                    std::cerr << "[sif:rpc-result] sid=0x" << std::hex << binding.sid
                              << " fn=0x" << rpcNumber
                              << " handled=" << result.handled
                              << " result=0x" << result.resultAddress
                              << " recv0=0x" << resultWord0
                              << " recv1=0x" << resultWord1
                              << " recv2=0x" << resultWord2
                              << " recv3=0x" << resultWord3
                              << std::dec << std::endl;
                }
            });
            constexpr uint32_t kMaxRpcTransferBytes = 1u * 1024u * 1024u;
            const uint32_t transferSize = std::min(receiveSize, kMaxRpcTransferBytes);
            if (transferSize != 0u && receiveBuffer != 0u)
            {
                if (result.handled && result.resultAddress != 0u &&
                    result.resultAddress != receiveBuffer)
                {
                    (void)copyGuestByteRange(rdram,
                                             receiveBuffer,
                                             result.resultAddress,
                                             transferSize);
                }
                else if (!result.handled)
                {
                    std::vector<uint8_t> zeroes(transferSize, 0u);
                    (void)writeGuestBytes(rdram,
                                          receiveBuffer,
                                          zeroes.data(),
                                          zeroes.size());
                }
            }

            if (rmode == 0u)
            {
                if (receiveSize != 0u)
                {
                    // A no-callback call with a receive buffer completes by
                    // the data DMA itself. The data was copied above; there
                    // is no RPC_END command to enqueue.
                    generatedReply = true;
                    return true;
                }

                return writeRawSifRpcDirectCompletion(rdram,
                                                       packetAddress,
                                                       clientAddress,
                                                       generatedReply);
            }

            bool commandReply = false;
            const bool hasCommandReply = writeRawSifRpcEnd(rdram,
                                                           packetAddress,
                                                           clientAddress,
                                                           kSifCmdRpcCall,
                                                           0u,
                                                           0u,
                                                           commandReply);
            if (receiveSize == 0u)
            {
                // sceSifExecRequest emits RPC_END and also DMA-copies the
                // render packet when rmode is enabled but no receive buffer
                // was supplied. Both completions are observable by the EE
                // SIF state machine and must be produced by the transport.
                bool directReply = false;
                const bool hasDirectReply = writeRawSifRpcDirectCompletion(rdram,
                                                                             packetAddress,
                                                                             clientAddress,
                                                                             directReply);
                generatedReply = commandReply || directReply;
                return hasCommandReply || hasDirectReply;
            }

            generatedReply = commandReply;
            return hasCommandReply;
        }

        bool copyGuestByteRange(uint8_t *rdram, uint32_t dstAddr, uint32_t srcAddr, uint32_t sizeBytes)
        {
            if (!canCopyGuestByteRange(rdram, dstAddr, srcAddr, sizeBytes))
            {
                return false;
            }

            if (sizeBytes == 0u)
            {
                return true;
            }

            const bool sourceIsIop = isSifIopHeapRange(srcAddr, sizeBytes);
            const bool destinationIsIop = isSifIopHeapRange(dstAddr, sizeBytes);
            if (sourceIsIop || destinationIsIop)
            {
                std::vector<uint8_t> payload(sizeBytes);
                if (sourceIsIop)
                {
                    if (!readSifIopHeap(srcAddr, payload.data(), payload.size()))
                    {
                        return false;
                    }
                }
                else
                {
                    for (uint32_t i = 0u; i < sizeBytes; ++i)
                    {
                        const uint8_t *src = getConstMemPtr(rdram, srcAddr + i);
                        if (!src)
                        {
                            return false;
                        }
                        payload[i] = *src;
                    }
                }

                if (destinationIsIop)
                {
                    return writeSifIopHeap(dstAddr, payload.data(), payload.size());
                }

                ps2TraceGuestRangeWrite(rdram, dstAddr, sizeBytes, "sifCopyGuestByteRange", nullptr);
                for (uint32_t i = 0u; i < sizeBytes; ++i)
                {
                    uint8_t *dst = getMemPtr(rdram, dstAddr + i);
                    if (!dst)
                    {
                        return false;
                    }
                    *dst = payload[i];
                }
                return true;
            }

            ps2TraceGuestRangeWrite(rdram, dstAddr, sizeBytes, "sifCopyGuestByteRange", nullptr);

            const uint64_t srcBegin = srcAddr;
            const uint64_t srcEnd = srcBegin + static_cast<uint64_t>(sizeBytes);
            const uint64_t dstBegin = dstAddr;
            const bool copyBackward = (dstBegin > srcBegin) && (dstBegin < srcEnd);

            if (copyBackward)
            {
                for (uint32_t i = sizeBytes; i > 0u; --i)
                {
                    const uint32_t index = i - 1u;
                    const uint8_t *src = getConstMemPtr(rdram, srcAddr + index);
                    uint8_t *dst = getMemPtr(rdram, dstAddr + index);
                    if (!src || !dst)
                    {
                        return false;
                    }
                    *dst = *src;
                }
                return true;
            }

            for (uint32_t i = 0; i < sizeBytes; ++i)
            {
                const uint8_t *src = getConstMemPtr(rdram, srcAddr + i);
                uint8_t *dst = getMemPtr(rdram, dstAddr + i);
                if (!src || !dst)
                {
                    return false;
                }
                *dst = *src;
            }
            return true;
        }
    }

    bool isSifIopHeapAddress(uint32_t address)
    {
        return address >= kIopHeapBase && address < kIopHeapLimit;
    }

    bool isSifIopHeapRange(uint32_t address, size_t size)
    {
        std::lock_guard<std::mutex> lock(g_sifHeapMutex);
        return isAllocatedSifHeapRangeLocked(address, size);
    }

    uint32_t allocateSifIopHeap(uint32_t size)
    {
        return allocateSifHeapBlock(size);
    }

    bool freeSifIopHeap(uint32_t address)
    {
        return freeSifHeapBlock(address);
    }

    bool readSifIopHeap(uint32_t address, void *destination, size_t size)
    {
        if (!destination && size != 0u)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(g_sifHeapMutex);
        if (!isAllocatedSifHeapRangeLocked(address, size))
        {
            return false;
        }
        if (size != 0u)
        {
            std::memcpy(destination,
                        g_sifHeapStorage.data() + (address - kIopHeapBase),
                        size);
        }
        return true;
    }

    bool writeSifIopHeap(uint32_t address, const void *source, size_t size)
    {
        if (!source && size != 0u)
        {
            return false;
        }
        std::lock_guard<std::mutex> lock(g_sifHeapMutex);
        if (!isAllocatedSifHeapRangeLocked(address, size))
        {
            return false;
        }
        if (size != 0u)
        {
            std::memcpy(g_sifHeapStorage.data() + (address - kIopHeapBase),
                        source,
                        size);
        }
        return true;
    }

    bool zeroSifIopHeap(uint32_t address, size_t size)
    {
        std::lock_guard<std::mutex> lock(g_sifHeapMutex);
        if (!isAllocatedSifHeapRangeLocked(address, size))
        {
            return false;
        }
        if (size != 0u)
        {
            std::memset(g_sifHeapStorage.data() + (address - kIopHeapBase), 0, size);
        }
        return true;
    }

    void resetSifState()
    {
        releaseRawSifRpcBindings(nullptr);
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        seedDefaultSifRegsLocked();
        resetSifHeapState();
    }

    void resetSifRuntimeState(PS2Runtime *runtime)
    {
        releaseRawSifRpcBindings(runtime);
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        g_sifEeReceiveBuffer = 0u;
        g_sifBootEndPending = false;
    }

    void sceSifAddCmdHandler(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t cid = getRegU32(ctx, 4);
        const uint32_t handler = getRegU32(ctx, 5);
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        g_sifCmdHandlers[cid] = handler;
        setReturnS32(ctx, 0);
    }

    void sceSifAllocIopHeap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;

        const uint32_t reqSize = getRegU32(ctx, 4);
        setReturnU32(ctx, allocateSifHeapBlock(reqSize));
    }

    void sceSifAllocSysMemory(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;

        const uint32_t size = getRegU32(ctx, 5);
        setReturnU32(ctx, allocateSifHeapBlock(size));
    }

    void sceSifBindRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifBindRpc(rdram, ctx, runtime);
    }

    void sceSifCheckStatRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifCheckStatRpc(rdram, ctx, runtime);
    }

    void sceSifDmaStat(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        (void)getRegU32(ctx, 4); // trid

        // Transfers are applied immediately by sceSifSetDma in this runtime.
        setReturnS32(ctx, -1);
    }

    void sceSifExecRequest(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifExitCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        // SIF command state is process-global in this compatibility layer, so
        // exiting the command service invalidates every raw RPC binding.
        releaseRawSifRpcBindings(nullptr);
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        seedDefaultSifRegsLocked();
        setReturnS32(ctx, 0);
    }

    void sceSifExitRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifFreeIopHeap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;

        const uint32_t addr = getRegU32(ctx, 4);
        setReturnS32(ctx, freeSifHeapBlock(addr) ? 0 : -1);
    }

    void sceSifFreeSysMemory(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;

        const uint32_t addr = getRegU32(ctx, 4);
        setReturnS32(ctx, freeSifHeapBlock(addr) ? 0 : -1);
    }

    void sceSifGetDataTable(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        setReturnU32(ctx, g_sifCmdBuffer);
    }

    void sceSifGetIopAddr(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnU32(ctx, getRegU32(ctx, 4));
    }

    void sceSifGetNextRequest(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifGetOtherData(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t rdAddr = getRegU32(ctx, 4);
        const uint32_t srcAddr = getRegU32(ctx, 5);
        const uint32_t dstAddr = getRegU32(ctx, 6);
        const int32_t sizeSigned = static_cast<int32_t>(getRegU32(ctx, 7));

        if (sizeSigned <= 0)
        {
            setReturnS32(ctx, 0);
            return;
        }

        const uint32_t size = static_cast<uint32_t>(sizeSigned);
        if (size > PS2_RAM_SIZE)
        {
            static uint32_t warnCount = 0;
            if (warnCount < 32u)
            {
                std::cerr << "sceSifGetOtherData rejected oversized transfer size=0x"
                          << std::hex << size << std::dec << std::endl;
                ++warnCount;
            }
            setReturnS32(ctx, -1);
            return;
        }

        if (runtime)
        {
            PS2IopTransport::notifyTransfer(runtime, rdram, {
                ps2x::iop::SifTransferKind::GetOtherData,
                ps2x::iop::SifTransferPhase::BeforeCopy,
                srcAddr,
                dstAddr,
                size,
            });
        }

        if (!copyGuestByteRange(rdram, dstAddr, srcAddr, size))
        {
            static uint32_t warnCount = 0;
            if (warnCount < 32u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    std::cerr << "sceSifGetOtherData copy failed src=0x" << std::hex << srcAddr
                              << " dst=0x" << dstAddr
                              << " size=0x" << size
                              << std::dec << std::endl;
                });
                ++warnCount;
            }
            setReturnS32(ctx, -1);
            return;
        }

        // SifRpcReceiveData_t keeps src/dest/size at offsets 0x10/0x14/0x18.
        if (uint8_t *rd = getMemPtr(rdram, rdAddr))
        {
            std::memcpy(rd + 0x10u, &srcAddr, sizeof(srcAddr));
            std::memcpy(rd + 0x14u, &dstAddr, sizeof(dstAddr));
            std::memcpy(rd + 0x18u, &size, sizeof(size));
        }

        if (runtime)
        {
            PS2IopTransport::notifyTransfer(runtime, rdram, {
                ps2x::iop::SifTransferKind::GetOtherData,
                ps2x::iop::SifTransferPhase::AfterCopy,
                srcAddr,
                dstAddr,
                size,
            });
        }

        setReturnS32(ctx, 0);
    }

    void sceSifGetReg(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t reg = getRegU32(ctx, 4);
        uint32_t value = 0u;
        bool shouldLog = false;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            auto it = g_sifRegs.find(reg);
            if (it != g_sifRegs.end())
            {
                value = it->second;
            }
            if (reg == kSifRegBootStatus && g_sifBootEndPending)
            {
                value |= 0x00040000u;
                g_sifRegs[reg] = value;
                g_sifBootEndPending = false;
            }
            shouldLog = shouldTraceSifReg(reg) && g_sifGetRegLogCount < 128u;
            if (shouldLog)
            {
                ++g_sifGetRegLogCount;
            }
        }
        if (shouldLog)
        {
            PS2_IF_AGRESSIVE_LOGS({
                auto flags = std::cerr.flags();
                std::cerr << "[sceSifGetReg] reg=0x" << std::hex << reg
                          << " value=0x" << value
                          << " pc=0x" << (ctx ? ctx->pc : 0u)
                          << " ra=0x" << (ctx ? getRegU32(ctx, 31) : 0u)
                          << std::dec << std::endl;
                std::cerr.flags(flags);
            });
        }
        setReturnU32(ctx, value);
    }

    void sceSifGetSreg(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t reg = getRegU32(ctx, 4);
        uint32_t value = 0u;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            auto it = g_sifSregs.find(reg);
            if (it != g_sifSregs.end())
            {
                value = it->second;
            }
        }
        setReturnU32(ctx, value);
    }

    void sceSifInitCmd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        g_sifCmdInitialized = true;
        setReturnS32(ctx, 0);
    }

    void sceSifInitIopHeap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        resetSifHeapState();
        setReturnS32(ctx, 0);
    }

    void sceSifInitRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifInitRpc(rdram, ctx, runtime);
    }

    void sceSifIsAliveIop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceSifLoadElf(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::sceSifLoadElf(rdram, ctx, runtime);
    }

    void sceSifLoadElfPart(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::sceSifLoadElfPart(rdram, ctx, runtime);
    }

    void sceSifLoadFileReset(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifLoadIopHeap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifLoadModuleBuffer(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::sceSifLoadModuleBuffer(rdram, ctx, runtime);
    }

    void sceSifRebootIop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceSifRegisterRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifRegisterRpc(rdram, ctx, runtime);
    }

    void sceSifRemoveCmdHandler(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t cid = getRegU32(ctx, 4);
        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
        g_sifCmdHandlers.erase(cid);
        setReturnS32(ctx, 0);
    }

    void sceSifRemoveRpc(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifRemoveRpc(rdram, ctx, runtime);
    }

    void sceSifRemoveRpcQueue(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifRemoveRpcQueue(rdram, ctx, runtime);
    }

    void sceSifResetIop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        PS2IopTransport::reset(runtime);
        setReturnS32(ctx, 1);
    }

    void sceSifRpcLoop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifSetCmdBuffer(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t newBuffer = getRegU32(ctx, 4);
        uint32_t prev = 0u;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            prev = g_sifCmdBuffer;
            g_sifCmdBuffer = newBuffer;
        }
        setReturnU32(ctx, prev);
    }

    void isceSifSetDChain(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        sceSifSetDChain(rdram, ctx, runtime);
    }

    void isceSifSetDma(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        sceSifSetDma(rdram, ctx, runtime);
    }

    void sceSifSetDChain(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        setReturnS32(ctx, 0);
    }

    void sceSifSetDma(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t dmatAddr = getRegU32(ctx, 4);
        const uint32_t count = getRegU32(ctx, 5);

        const uint32_t listAddr = getRegU32(ctx, 4);
        // TEMP-EXPERIMENT: log SIF DMA senders to find libmc's packet builder. Revert.
        {
            static int dmaCount = 0;
            if (dmaCount < 300 && dmatAddr != 0u && count > 0u && count <= 32u)
            {
                ++dmaCount;
                const uint8_t *e0 = getConstMemPtr(rdram, dmatAddr);
                uint32_t src = 0, dst = 0, size = 0, attr = 0;
                if (e0)
                {
                    std::memcpy(&src, e0, 4); std::memcpy(&dst, e0 + 4, 4);
                    std::memcpy(&size, e0 + 8, 4); std::memcpy(&attr, e0 + 12, 4);
                }
                std::cerr << "[sifdma] n=" << std::dec << dmaCount
                          << " pc=0x" << std::hex << ctx->pc
                          << " ra=0x" << getRegU32(ctx, 31)
                          << " src=0x" << (src & 0x1FFFFFFF) << " dst=0x" << (dst & 0x1FFFFFFF)
                          << " size=0x" << size << " attr=0x" << attr
                          << std::dec << std::endl;
            }
        }
        PS2_IF_AGRESSIVE_LOGS({
            std::cerr << "[sceSifSetDma:CALL] pc=0x" << std::hex << ctx->pc
                      << " ra=0x" << getRegU32(ctx, 31)
                      << " list=0x" << listAddr
                      << " count=" << std::dec << count
                      << std::endl;

            for (uint32_t i = 0; i < count; ++i)
            {
                const uint32_t desc = listAddr + i * 16;
                const uint32_t src = READ32(desc + 0);
                const uint32_t dst = READ32(desc + 4);
                const uint32_t size = READ32(desc + 8);
                const uint32_t attr = READ32(desc + 12);

                std::cerr << "[sceSifSetDma:DESC] i=" << i
                          << " src=0x" << std::hex << src
                          << " dst=0x" << dst
                          << " size=0x" << size
                          << " attr=0x" << attr
                          << " pc=0x" << ctx->pc
                          << " ra=0x" << getRegU32(ctx, 31)
                          << std::dec << std::endl;
            }
        });

        if (!dmatAddr || count == 0u || count > 32u)
        {
            setReturnS32(ctx, 0);
            return;
        }

        std::array<Ps2SifDmaTransfer, 32u> pending{};
        uint32_t pendingCount = 0u;
        bool ok = true;
        for (uint32_t i = 0; i < count; ++i)
        {
            const uint32_t entryAddr = dmatAddr + (i * static_cast<uint32_t>(sizeof(Ps2SifDmaTransfer)));
            const uint8_t *entry = getConstMemPtr(rdram, entryAddr);
            if (!entry)
            {
                ok = false;
                break;
            }

            Ps2SifDmaTransfer xfer{};
            std::memcpy(&xfer, entry, sizeof(xfer));
            if (xfer.size <= 0)
            {
                continue;
            }

            const uint32_t sizeBytes = static_cast<uint32_t>(xfer.size);
            if (sizeBytes > PS2_RAM_SIZE)
            {
                ok = false;
                break;
            }
            const bool validTransfer = xfer.dest == 0u
                                           ? canCopyAddressRange(rdram, xfer.src, sizeBytes)
                                           : canCopyGuestByteRange(rdram, xfer.dest, xfer.src, sizeBytes);
            if (!validTransfer)
            {
                ok = false;
                break;
            }

            pending[pendingCount++] = xfer;
        }

        if (ok)
        {
            for (uint32_t i = 0; i < pendingCount; ++i)
            {
                const Ps2SifDmaTransfer &xfer = pending[i];
                if (runtime)
                {
                    PS2IopTransport::notifyTransfer(runtime, rdram, {
                        ps2x::iop::SifTransferKind::SetDma,
                        ps2x::iop::SifTransferPhase::BeforeCopy,
                        xfer.src,
                        xfer.dest,
                        static_cast<uint32_t>(xfer.size),
                    });
                }
                if (xfer.dest != 0u &&
                    !copyGuestByteRange(rdram, xfer.dest, xfer.src, static_cast<uint32_t>(xfer.size)))
                {
                    ok = false;
                    break;
                }
                if (runtime)
                {
                    PS2IopTransport::notifyTransfer(runtime, rdram, {
                        ps2x::iop::SifTransferKind::SetDma,
                        ps2x::iop::SifTransferPhase::AfterCopy,
                        xfer.src,
                        xfer.dest,
                        static_cast<uint32_t>(xfer.size),
                    });
                }
            }
        }

        bool generatedSifReply = false;
        bool generatedRpcReply = false;
        if (ok)
        {
            // SIF commands sent to destination zero are delivered to the IOP
            // sifcmd endpoint. INIT_CMD is the standard boot handshake used
            // by sifcmd/sifrpc; it is independent of any game module.
            for (uint32_t i = 0u; i < pendingCount; ++i)
            {
                const Ps2SifDmaTransfer &xfer = pending[i];
                const uint32_t sizeBytes = static_cast<uint32_t>(xfer.size);
                if (xfer.dest != 0u || sizeBytes < 0x10u)
                {
                    continue;
                }

                uint32_t command = 0u;
                uint32_t option = 0u;
                if (!readSifU32(rdram, xfer.src + 0x08u, command) ||
                    !readSifU32(rdram, xfer.src + 0x0Cu, option))
                {
                    continue;
                }

                PS2_IF_AGRESSIVE_LOGS({
                    if (g_sifDmaCommandLogCount++ < 96u)
                    {
                        std::cerr << "[sif:dma-command] src=0x" << std::hex << xfer.src
                                  << " size=0x" << sizeBytes
                                  << " command=0x" << command
                                  << " option=0x" << option
                                  << " pc=0x" << (ctx ? ctx->pc : 0u)
                                  << std::dec << std::endl;
                    }
                });

                if (command == kSifCmdReset)
                {
                    // SifIopReset completes asynchronously on real hardware.
                    // Keep the standard BOOTEND flag visible so the caller can
                    // leave its reset wait loop, and invalidate old RPC state
                    // just as an IOP reboot would.
                    PS2IopTransport::reset(runtime);
                    {
                        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                        g_sifRegs[0x4u] |= 0x00040000u;
                        g_sifBootEndPending = true;
                        g_sifRegs[0x80000002u] = 0u;
                        g_sifRegs[0x80000000u] = 0u;
                    }
                    continue;
                }

                if (command != kSifCmdInit)
                {
                    if (command == kSifCmdRpcBind)
                    {
                        bool replyGenerated = false;
                        (void)handleRawSifRpcBind(rdram,
                                                  runtime,
                                                  xfer.src,
                                                  sizeBytes,
                                                  replyGenerated);
                        generatedRpcReply = generatedRpcReply || replyGenerated;
                    }
                    else if (command == kSifCmdRpcCall)
                    {
                        bool replyGenerated = false;
                        (void)handleRawSifRpcCall(rdram,
                                                  runtime,
                                                  xfer.src,
                                                  sizeBytes,
                                                  replyGenerated);
                        generatedRpcReply = generatedRpcReply || replyGenerated;
                    }
                    continue;
                }

                if (option == 0u)
                {
                    if (sizeBytes < 0x14u)
                    {
                        continue;
                    }
                    uint32_t receiveBuffer = 0u;
                    if (readSifU32(rdram, xfer.src + 0x10u, receiveBuffer))
                    {
                        std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                        g_sifEeReceiveBuffer = receiveBuffer;
                    }
                    continue;
                }

                if (option != 1u)
                {
                    continue;
                }

                uint32_t receiveBuffer = 0u;
                {
                    std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
                    receiveBuffer = g_sifEeReceiveBuffer;
                }
                if (receiveBuffer == 0u || isSifIopHeapRange(receiveBuffer, 24u) ||
                    !canCopyAddressRange(rdram, receiveBuffer, 24u))
                {
                    continue;
                }

                const uint32_t reply[6] = {
                    24u,
                    0u,
                    kSifCmdSetSreg,
                    0u,
                    0u,
                    1u,
                };
                if (writeGuestBytes(rdram, receiveBuffer, reply, sizeof(reply)))
                {
                    generatedSifReply = true;
                }
            }
        }

        if (!ok)
        {
            static uint32_t warnCount = 0;
            if (warnCount < 32u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    std::cerr << "sceSifSetDma failed dmat=0x" << std::hex << dmatAddr
                              << " count=0x" << count
                              << std::dec << std::endl;
                });
                ++warnCount;
            }
            setReturnS32(ctx, 0);
            return;
        }

        setReturnS32(ctx, static_cast<int32_t>(allocateSifDmaTransferId()));
        if (generatedRpcReply)
        {
            static uint32_t sifReplyProbeCount = 0u;
            if (sifReplyProbeCount++ < 96u)
            {
                std::cerr << "[probe:sif0-reply] generated=1 pc=0x" << std::hex
                          << (ctx ? ctx->pc : 0u) << std::dec << '\n';
            }
        }
        if (runtime && generatedSifReply)
        {
            // The boot SET_SREG packet is already in the EE receive buffer.
            // A same-turn dispatch lets the guest leave its initialization
            // wait loop before the next scheduler checkpoint.
            runtime->eeScheduler().dispatchIrqNow(true, 5u);
        }
        else if (generatedRpcReply)
        {
            // RPC completion is a SIF0 DMA response. Keep it on the normal
            // deferred interrupt path so the guest command handler consumes
            // the packet at a scheduler boundary.
            ps2_syscalls::dispatchDmacHandlersForCause(rdram, runtime, 5u);
        }
        else
        {
            // Ordinary DMA completions retain the existing deferred ordering.
            ps2_syscalls::dispatchDmacHandlersForCause(rdram, runtime, 5u);
        }
    }

    void sceSifSetIopAddr(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnU32(ctx, getRegU32(ctx, 5));
    }

    void sceSifSetReg(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t reg = getRegU32(ctx, 4);
        const uint32_t value = getRegU32(ctx, 5);
        uint32_t prev = 0u;
        bool shouldLog = false;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            auto it = g_sifRegs.find(reg);
            if (it != g_sifRegs.end())
            {
                prev = it->second;
            }
            g_sifRegs[reg] = value;
            shouldLog = shouldTraceSifReg(reg) && g_sifSetRegLogCount < 128u;
            if (shouldLog)
            {
                ++g_sifSetRegLogCount;
            }
        }
        if (shouldLog)
        {
            PS2_IF_AGRESSIVE_LOGS({
                auto flags = std::cerr.flags();
                std::cerr << "[sceSifSetReg] reg=0x" << std::hex << reg
                          << " prev=0x" << prev
                          << " value=0x" << value
                          << " pc=0x" << (ctx ? ctx->pc : 0u)
                          << " ra=0x" << (ctx ? getRegU32(ctx, 31) : 0u)
                          << std::dec << std::endl;
                std::cerr.flags(flags);
            });
        }
        setReturnU32(ctx, prev);
    }

    void sceSifSetRpcQueue(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        ps2_syscalls::SifSetRpcQueue(rdram, ctx, runtime);
    }

    void sceSifSetSreg(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t reg = getRegU32(ctx, 4);
        const uint32_t value = getRegU32(ctx, 5);
        uint32_t prev = 0u;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            auto it = g_sifSregs.find(reg);
            if (it != g_sifSregs.end())
            {
                prev = it->second;
            }
            g_sifSregs[reg] = value;
        }
        setReturnU32(ctx, prev);
    }

    void sceSifSetSysCmdBuffer(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t newBuffer = getRegU32(ctx, 4);
        uint32_t prev = 0u;
        {
            std::lock_guard<std::mutex> lock(g_sifCmdStateMutex);
            prev = g_sifSysCmdBuffer;
            g_sifSysCmdBuffer = newBuffer;
        }
        setReturnU32(ctx, prev);
    }

    void sceSifStopDma(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }

    void sceSifSyncIop(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 1);
    }

    void sceSifWriteBackDCache(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        setReturnS32(ctx, 0);
    }
}
