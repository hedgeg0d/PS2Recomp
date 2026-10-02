#include "MiniTest.h"
#include "ps2_runtime.h"
#include "ps2_iop_host.h"
#include "ps2_iop_transport.h"
#include "ps2_syscalls.h"
#include "ps2_stubs.h"
#include "Kernel/Stubs/SIF.h"
#include "runtime/ee_scheduler.h"

#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ps2_stubs
{
    void resetSifState();
}

namespace
{
    constexpr int KE_OK = 0;

    struct TestEnv
    {
        std::vector<uint8_t> rdram;
        R5900Context ctx{};
        PS2Runtime runtime;

        TestEnv() : rdram(PS2_RAM_SIZE, 0u)
        {
            ps2_stubs::resetSifState();
            std::memset(&ctx, 0, sizeof(ctx));
        }
    };

    #pragma pack(push, 1)
    struct Ps2SifDmaTransfer
    {
        uint32_t src;
        uint32_t dest;
        int32_t size;
        int32_t attr;
    };

    struct SifRpcHeader
    {
        uint32_t pkt_addr;
        uint32_t rpc_id;
        int32_t sema_id;
        uint32_t mode;
    };

    struct SifRpcReceiveData
    {
        SifRpcHeader hdr;
        uint32_t src;
        uint32_t dest;
        int32_t size;
    };
    #pragma pack(pop)

    static_assert(sizeof(Ps2SifDmaTransfer) == 16u, "Unexpected Ps2SifDmaTransfer size.");
    static_assert(sizeof(SifRpcReceiveData) == 28u, "Unexpected SifRpcReceiveData size.");

    void setRegU32(R5900Context &ctx, int reg, uint32_t value)
    {
        ctx.r[reg] = _mm_set_epi64x(0, static_cast<int64_t>(value));
    }

    int32_t getRegS32(const R5900Context &ctx, int reg)
    {
        return static_cast<int32_t>(::getRegU32(&ctx, reg));
    }

    void writeGuestU32(uint8_t *rdram, uint32_t addr, uint32_t value)
    {
        std::memcpy(rdram + addr, &value, sizeof(value));
    }

    uint32_t readGuestU32(const uint8_t *rdram, uint32_t addr)
    {
        uint32_t value = 0;
        std::memcpy(&value, rdram + addr, sizeof(value));
        return value;
    }

    uint32_t g_dmacHandlerWriteAddr = 0u;
    uint32_t g_dmacHandlerValue = 0u;
    uint32_t g_dmacHandlerLastCause = 0u;
    uint32_t g_dmacHandlerLastArg = 0u;
    uint32_t g_dmacDispatchSequence = 0u;
    uint32_t g_dmacHandlerObservedSequence = 0u;
    uint32_t g_dmacHandlerReplyAddr = 0u;
    int32_t g_sifDmaResult = 0;

    constexpr uint32_t kSchedulerSifDmaEntryPc = 0x00101000u;
    constexpr uint32_t kSchedulerSifDmaResumePc = 0x00101010u;
    constexpr uint32_t kSchedulerSifDmaHandlerPc = 0x00101020u;
    constexpr uint32_t kSchedulerSifDmaDescAddr = 0x00020300u;
    constexpr uint32_t kSchedulerSifDmaHandlerArg = 0x12345678u;

    void testDmacHandler(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)runtime;
        g_dmacHandlerLastCause = ::getRegU32(ctx, 4);
        g_dmacHandlerLastArg = ::getRegU32(ctx, 5);
        g_dmacHandlerObservedSequence = g_dmacDispatchSequence;
        if (g_dmacHandlerWriteAddr != 0u)
        {
            writeGuestU32(rdram, g_dmacHandlerWriteAddr, g_dmacHandlerValue);
        }
        ctx->pc = 0u;
    }

    void schedulerSifDmaEntry(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        runtime->eeScheduler().addIrqHandler(true,
                                             5u,
                                             kSchedulerSifDmaHandlerPc,
                                             true,
                                             kSchedulerSifDmaHandlerArg,
                                             0u,
                                             0u);
        setRegU32(*ctx, 4, kSchedulerSifDmaDescAddr);
        setRegU32(*ctx, 5, 1u);
        ctx->pc = kSchedulerSifDmaResumePc;
        g_dmacDispatchSequence = 1u;
        ps2_stubs::sceSifSetDma(rdram, ctx, runtime);
        g_dmacDispatchSequence = 2u;
    }

    void schedulerSifDmaResume(uint8_t *, R5900Context *ctx, PS2Runtime *runtime)
    {
        g_sifDmaResult = getRegS32(*ctx, 2);
        ctx->pc = 0u;
        runtime->requestStop();
    }
}

void register_ps2_sif_dma_tests()
{
    MiniTest::Case("PS2SifDma", [](TestCase &tc)
    {
        tc.Run("sceSifSetDma copies payload and sceSifDmaStat reports complete", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kDescAddr = 0x00020000u;
            constexpr uint32_t kSrcAddr = 0x00020100u;
            constexpr uint32_t kDstAddr = 0x00020200u;

            std::array<uint8_t, 16> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
            {
                payload[i] = static_cast<uint8_t>(0x30u + i);
            }
            std::memcpy(env.rdram.data() + kSrcAddr, payload.data(), payload.size());
            std::memset(env.rdram.data() + kDstAddr, 0x5A, payload.size());

            const Ps2SifDmaTransfer desc{
                kSrcAddr,
                kDstAddr,
                static_cast<int32_t>(payload.size()),
                0};
            std::memcpy(env.rdram.data() + kDescAddr, &desc, sizeof(desc));

            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);
            const int32_t dmaId = getRegS32(env.ctx, 2);
            t.IsTrue(dmaId > 0, "sceSifSetDma should return a positive transfer id on success");

            std::array<uint8_t, 16> iopReadback{};
            t.IsTrue(env.runtime.readIopMemory(kDstAddr, iopReadback.data(), iopReadback.size()) &&
                         iopReadback == payload,
                     "sceSifSetDma should copy EE payload into IOP RAM");
            const std::array<uint8_t, 16> eeSentinel = {
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A,
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
            t.IsTrue(std::memcmp(env.rdram.data() + kDstAddr, eeSentinel.data(), eeSentinel.size()) == 0,
                     "sceSifSetDma must not alias an equal-numbered EE address");

            setRegU32(env.ctx, 4, static_cast<uint32_t>(dmaId));
            ps2_stubs::sceSifDmaStat(env.rdram.data(), &env.ctx, &env.runtime);
            t.IsTrue(getRegS32(env.ctx, 2) < 0, "sceSifDmaStat should be negative when transfer is complete");
        });

        tc.Run("raw SIF INIT_CMD replies through the EE receive buffer", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kDescAddr = 0x00020700u;
            constexpr uint32_t kPacketAddr = 0x00020800u;
            constexpr uint32_t kReceiveAddr = 0x00020900u;
            constexpr uint32_t kInitCmd = 0x80000002u;

            const Ps2SifDmaTransfer initDesc{kPacketAddr, 0u, 20, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &initDesc, sizeof(initDesc));
            const uint32_t initPacket[5] = {20u, 0u, kInitCmd, 0u, kReceiveAddr};
            std::memcpy(env.rdram.data() + kPacketAddr, initPacket, sizeof(initPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            const Ps2SifDmaTransfer bootEndDesc{kPacketAddr, 0u, 16, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &bootEndDesc, sizeof(bootEndDesc));
            const uint32_t bootEndPacket[4] = {16u, 0u, kInitCmd, 1u};
            std::memcpy(env.rdram.data() + kPacketAddr, bootEndPacket, sizeof(bootEndPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(readGuestU32(env.rdram.data(), 0u), 0u,
                     "raw SIF commands should not overwrite EE address zero");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0u), 24u,
                     "SET_SREG reply should carry a 24-byte packet size");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 8u), 0x80000001u,
                     "reply should use SIF_CMD_SET_SREG");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 16u), 0u,
                     "reply should select SREG 0");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 20u), 1u,
                     "reply should mark RPC initialization complete");
        });

        tc.Run("raw SIF RPC uses registered services and null-binds unknown SIDs", [](TestCase &t)
        {
            TestEnv env;
            t.IsTrue(env.runtime.memory().initialize(), "runtime memory initialize should succeed");

            constexpr uint32_t kDescAddr = 0x00020A00u;
            constexpr uint32_t kInitPacketAddr = 0x00020B00u;
            constexpr uint32_t kBindPacketAddr = 0x00020C00u;
            constexpr uint32_t kCallPacketAddr = 0x00020D00u;
            constexpr uint32_t kReceiveAddr = 0x00020E00u;
            constexpr uint32_t kClientAddr = 0x00020F00u;
            constexpr uint32_t kPayloadAddr = 0x00021000u;
            constexpr uint32_t kBindRequestPacket = 0x0002F000u;
            constexpr uint32_t kCallRequestPacket = 0x0002F040u;
            constexpr uint32_t kUnknownRequestPacket = 0x0002F080u;
            constexpr uint32_t kKnownSid = 0x80000701u; // core libsd service
            constexpr uint32_t kUnknownSid = 0xDEADBEEFu;
            constexpr uint32_t kRpcEnd = 0x80000008u;
            constexpr uint32_t kRpcBind = 0x80000009u;
            constexpr uint32_t kRpcCall = 0x8000000Au;

            const Ps2SifDmaTransfer initDesc{kInitPacketAddr, 0u, 20, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &initDesc, sizeof(initDesc));
            const uint32_t initPacket[5] = {20u, 0u, 0x80000002u, 0u, kReceiveAddr};
            std::memcpy(env.rdram.data() + kInitPacketAddr, initPacket, sizeof(initPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            const uint32_t bindPacket[16] = {
                64u, 0u, kRpcBind, 0u,
                5u, kBindRequestPacket, 2u, kClientAddr,
                kKnownSid, 0u, 0u, 0u,
                0u, 0u, 0u, 0u,
            };
            const Ps2SifDmaTransfer bindDesc{kBindPacketAddr, 0u, 64, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &bindDesc, sizeof(bindDesc));
            std::memcpy(env.rdram.data() + kBindPacketAddr, bindPacket, sizeof(bindPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x00u), 64u,
                     "raw BIND should return a full RPC_END packet");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x08u), kRpcEnd,
                     "raw BIND should use RPC_END");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x14u), kBindRequestPacket,
                     "RPC_END should retain the request packet address");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x1Cu), kClientAddr,
                     "RPC_END should identify the client data");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x20u), kRpcBind,
                     "RPC_END should identify BIND completion");

            const uint32_t serverAddr = readGuestU32(env.rdram.data(), kReceiveAddr + 0x24u);
            const uint32_t serverBuffer = readGuestU32(env.rdram.data(), kReceiveAddr + 0x28u);
            t.IsTrue(serverAddr != 0u && serverBuffer != 0u,
                     "known raw RPC services should receive allocated server state");
            t.Equals(readGuestU32(env.rdram.data(), serverAddr + 0x00u), kKnownSid,
                     "allocated raw server state should carry the bound SID");
            t.Equals(readGuestU32(env.rdram.data(), serverAddr + 0x08u), serverBuffer,
                     "allocated raw server state should carry its request buffer");

            std::memset(env.rdram.data() + kReceiveAddr, 0xA5, 64u);
            std::memset(env.rdram.data() + kPayloadAddr, 0x3C, 4u);
            const uint32_t callPacket[16] = {
                64u, 0u, kRpcCall, 0u,
                7u, kCallRequestPacket, 3u, kClientAddr,
                1u, 4u, kReceiveAddr, 4u, 1u, serverAddr,
                0u, 0u,
            };
            const Ps2SifDmaTransfer callDescs[2] = {
                {kCallPacketAddr, 0u, 64, 4},
                {kPayloadAddr, serverBuffer, 4, 0},
            };
            std::memcpy(env.rdram.data() + kDescAddr, callDescs, sizeof(callDescs));
            std::memcpy(env.rdram.data() + kCallPacketAddr, callPacket, sizeof(callPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 2u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x08u), kRpcEnd,
                     "raw CALL should complete through RPC_END");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x20u), kRpcCall,
                     "RPC_END should identify CALL completion");
            t.Equals(env.rdram[kReceiveAddr + 0x00u], 0x40u,
                     "a handled raw CALL should leave its receive buffer available to the service");

            const uint32_t noReceivePacket[16] = {
                64u, 0u, kRpcCall, 0u,
                8u, kCallRequestPacket, 0u, kClientAddr,
                1u, 4u, 0u, 0u, 1u, serverAddr,
                0u, 0u,
            };
            std::memset(env.rdram.data() + kCallRequestPacket, 0xA5, 64u);
            std::memcpy(env.rdram.data() + kCallPacketAddr, noReceivePacket, sizeof(noReceivePacket));
            std::memcpy(env.rdram.data() + kDescAddr, callDescs, sizeof(callDescs));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 2u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x08u), kRpcEnd,
                     "a no-receive raw CALL should still emit RPC_END");
            t.Equals(readGuestU32(env.rdram.data(), kCallRequestPacket + 0x10u), 0u,
                     "a no-receive raw CALL should clear the direct completion RPC id");
            t.Equals(readGuestU32(env.rdram.data(), kCallRequestPacket + 0x14u), kCallRequestPacket,
                     "a no-receive raw CALL should DMA its completion to the request packet");
            t.Equals(readGuestU32(env.rdram.data(), kCallRequestPacket + 0x1Cu), kClientAddr,
                     "direct CALL completion should retain the client address");
            t.Equals(readGuestU32(env.rdram.data(), kCallRequestPacket + 0x20u), kRpcCall,
                     "direct CALL completion should identify CALL completion");

            const uint32_t unknownClientAddr = kClientAddr + 0x40u;
            const uint32_t unknownPacketAddr = kBindPacketAddr + 0x40u;
            uint32_t unknownPacket[16] = {};
            unknownPacket[0] = 64u;
            unknownPacket[2] = kRpcBind;
            unknownPacket[4] = 9u;
            unknownPacket[5] = kUnknownRequestPacket;
            unknownPacket[6] = 4u;
            unknownPacket[7] = unknownClientAddr;
            unknownPacket[8] = kUnknownSid;
            const Ps2SifDmaTransfer unknownDesc{unknownPacketAddr, 0u, 64, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &unknownDesc, sizeof(unknownDesc));
            std::memcpy(env.rdram.data() + unknownPacketAddr, unknownPacket, sizeof(unknownPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x08u), kRpcEnd,
                     "unknown raw BIND should still complete through RPC_END");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x24u), 0u,
                     "unknown raw BIND should return a null server");
            t.Equals(readGuestU32(env.rdram.data(), kReceiveAddr + 0x28u), 0u,
                     "unknown raw BIND should return no request buffer");

            // Resetting the IOP invalidates the EE-side raw RPC state. A
            // request submitted with a server handle from before the reset
            // must not be dispatched or produce a stale RPC_END reply.
            std::memset(env.rdram.data() + kReceiveAddr, 0xA5, 64u);
            ps2_stubs::sceSifResetIop(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 1,
                     "sceSifResetIop should report that the reset was accepted");

            const Ps2SifDmaTransfer staleCallDesc{kCallPacketAddr, 0u, 64, 4};
            std::memcpy(env.rdram.data() + kDescAddr, &staleCallDesc, sizeof(staleCallDesc));
            std::memcpy(env.rdram.data() + kCallPacketAddr, callPacket, sizeof(callPacket));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);

            t.Equals(env.rdram[kReceiveAddr], 0xA5u,
                     "a pre-reset raw RPC binding must not generate a completion reply");
        });

        tc.Run("IOP heap DMA uses private backing instead of aliasing EE RDRAM", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kDescAddr = 0x00020040u;
            constexpr uint32_t kSrcAddr = 0x00020140u;
            constexpr uint32_t kRoundTripAddr = 0x00020240u;
            constexpr uint32_t kIopBlockSize = 0x880u;

            std::array<uint8_t, 32> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
            {
                payload[i] = static_cast<uint8_t>(0x80u + i);
            }
            std::memcpy(env.rdram.data() + kSrcAddr, payload.data(), payload.size());
            std::memset(env.rdram.data() + kRoundTripAddr, 0, payload.size());

            setRegU32(env.ctx, 4, kIopBlockSize);
            ps2_stubs::sceSifAllocIopHeap(env.rdram.data(), &env.ctx, &env.runtime);
            const uint32_t iopAddress = ::getRegU32(&env.ctx, 2);
            t.IsTrue(iopAddress >= 0x00120000u && iopAddress < 0x00200000u,
                     "sceSifAllocIopHeap should return an address in physical IOP RAM");
            std::memset(env.rdram.data() + iopAddress, 0x5Au, payload.size());

            Ps2SifDmaTransfer desc{
                kSrcAddr,
                iopAddress,
                static_cast<int32_t>(payload.size()),
                0};
            std::memcpy(env.rdram.data() + kDescAddr, &desc, sizeof(desc));
            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);
            t.IsTrue(getRegS32(env.ctx, 2) > 0,
                     "EE-to-IOP DMA should accept a private IOP heap destination");

            const std::array<uint8_t, 32> aliasSentinel{
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A,
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A,
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A,
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
            t.IsTrue(std::memcmp(env.rdram.data() + iopAddress,
                                 aliasSentinel.data(), aliasSentinel.size()) == 0,
                     "IOP DMA must not overwrite the equal-numbered EE range");

            PS2IopHostAdapter host(env.runtime);
            auto scope = host.enterCall(&env.ctx, env.rdram.data());
            std::array<uint8_t, 32> hostReadback{};
            t.IsTrue(host.readIopMemory(iopAddress, hostReadback.data(), hostReadback.size()) &&
                         hostReadback == payload,
                     "IOP modules should read the shared physical IOP RAM");

            constexpr uint32_t kRdAddr = 0x00020340u;
            setRegU32(env.ctx, 4, kRdAddr);
            setRegU32(env.ctx, 5, iopAddress);
            setRegU32(env.ctx, 6, kRoundTripAddr);
            setRegU32(env.ctx, 7, static_cast<uint32_t>(payload.size()));
            ps2_stubs::sceSifGetOtherData(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0,
                     "IOP-to-EE transfer should accept a physical IOP source");
            t.IsTrue(std::memcmp(env.rdram.data() + kRoundTripAddr,
                                 payload.data(), payload.size()) == 0,
                     "IOP-to-EE DMA should round-trip the payload");
        });

        tc.Run("isceSifSetDma and isceSifSetDChain alias the SIF DMA helpers", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kDescAddr = 0x00020240u;
            constexpr uint32_t kSrcAddr = 0x00020340u;
            constexpr uint32_t kDstAddr = 0x00020440u;

            std::array<uint8_t, 12> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
            {
                payload[i] = static_cast<uint8_t>(0x50u + i);
            }
            std::memcpy(env.rdram.data() + kSrcAddr, payload.data(), payload.size());
            std::memset(env.rdram.data() + kDstAddr, 0x5A, payload.size());

            const Ps2SifDmaTransfer desc{
                kSrcAddr,
                kDstAddr,
                static_cast<int32_t>(payload.size()),
                0};
            std::memcpy(env.rdram.data() + kDescAddr, &desc, sizeof(desc));

            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 1u);
            ps2_stubs::isceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);
            t.IsTrue(getRegS32(env.ctx, 2) > 0, "isceSifSetDma should report a successful transfer id");
            std::array<uint8_t, 12> iopReadback{};
            t.IsTrue(env.runtime.readIopMemory(kDstAddr, iopReadback.data(), iopReadback.size()) &&
                         iopReadback == payload,
                     "isceSifSetDma should copy EE payload into IOP RAM");

            ps2_stubs::isceSifSetDChain(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0, "isceSifSetDChain should mirror sceSifSetDChain");
        });

        tc.Run("ordinary sceSifSetDma queues enabled DMAC handlers for SIF0 cause 5", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kSrcAddr = 0x00020400u;
            constexpr uint32_t kDstAddr = 0x00020500u;
            constexpr uint32_t kHandlerWriteAddr = 0x00020600u;

            g_dmacHandlerWriteAddr = kHandlerWriteAddr;
            g_dmacHandlerValue = 0xCAFEBABEu;
            g_dmacHandlerLastCause = 0u;
            g_dmacHandlerLastArg = 0u;
            g_dmacDispatchSequence = 0u;
            g_dmacHandlerObservedSequence = 0u;
            g_dmacHandlerReplyAddr = 0u;
            g_sifDmaResult = 0;
            env.runtime.registerFunction(kSchedulerSifDmaEntryPc, schedulerSifDmaEntry);
            env.runtime.registerFunction(kSchedulerSifDmaResumePc, schedulerSifDmaResume);
            env.runtime.registerFunction(kSchedulerSifDmaHandlerPc, testDmacHandler);

            std::array<uint8_t, 16> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
            {
                payload[i] = static_cast<uint8_t>(0x40u + i);
            }
            std::memcpy(env.rdram.data() + kSrcAddr, payload.data(), payload.size());

            const Ps2SifDmaTransfer desc{
                kSrcAddr,
                kDstAddr,
                static_cast<int32_t>(payload.size()),
                0};
            std::memcpy(env.rdram.data() + kSchedulerSifDmaDescAddr, &desc, sizeof(desc));

            R5900Context mainContext{};
            mainContext.pc = kSchedulerSifDmaEntryPc;
            env.runtime.eeScheduler().reset(env.rdram.data(), mainContext);
            env.runtime.eeScheduler().run();

            t.IsTrue(g_sifDmaResult > 0, "sceSifSetDma should still report success");
            t.Equals(readGuestU32(env.rdram.data(), kHandlerWriteAddr), g_dmacHandlerValue,
                     "the scheduler should execute the queued DMAC invocation");
            t.Equals(g_dmacHandlerLastCause, 5u, "DMAC handler should observe SIF0 cause 5");
            t.Equals(g_dmacHandlerLastArg, kSchedulerSifDmaHandlerArg,
                     "DMAC handler should receive registered argument");
            t.Equals(g_dmacHandlerObservedSequence, 2u,
                     "ordinary DMA handler should remain queued until sceSifSetDma returns");
        });

        tc.Run("resetSifState seeds boot-ready SIF registers", [](TestCase &t)
        {
            TestEnv env;

            auto getReg = [&](uint32_t reg) -> uint32_t
            {
                setRegU32(env.ctx, 4, reg);
                ps2_stubs::sceSifGetReg(env.rdram.data(), &env.ctx, &env.runtime);
                return ::getRegU32(&env.ctx, 2);
            };

            t.Equals(getReg(0x4u), 0x00020000u, "SIF boot status register should expose ready bit by default");
            t.Equals(getReg(0x80000000u), 0u, "SIF main-address register should default to zero");
            t.Equals(getReg(0x80000001u), 0u, "SIF sub-address register should default to zero");
            t.Equals(getReg(0x80000002u), 0u, "SIF mscom register should default to zero");
        });

        tc.Run("sceSifExitCmd restores default boot-ready SIF registers", [](TestCase &t)
        {
            TestEnv env;

            setRegU32(env.ctx, 4, 0x4u);
            setRegU32(env.ctx, 5, 0x12340000u);
            ps2_stubs::sceSifSetReg(env.rdram.data(), &env.ctx, &env.runtime);

            setRegU32(env.ctx, 4, 0x80000002u);
            setRegU32(env.ctx, 5, 0x89ABCDEFu);
            ps2_stubs::sceSifSetReg(env.rdram.data(), &env.ctx, &env.runtime);

            ps2_stubs::sceSifExitCmd(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0, "sceSifExitCmd should succeed");

            auto getReg = [&](uint32_t reg) -> uint32_t
            {
                setRegU32(env.ctx, 4, reg);
                ps2_stubs::sceSifGetReg(env.rdram.data(), &env.ctx, &env.runtime);
                return ::getRegU32(&env.ctx, 2);
            };

            t.Equals(getReg(0x4u), 0x00020000u, "sceSifExitCmd should restore the boot-ready status bit");
            t.Equals(getReg(0x80000002u), 0u, "sceSifExitCmd should clear transient mscom state");
        });

        tc.Run("sceSifSetDma rejects invalid descriptors without partial writes", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kDescAddr = 0x00021000u;
            constexpr uint32_t kSrcA = 0x00021100u;
            constexpr uint32_t kDstA = 0x00021200u;
            constexpr uint32_t kSrcB = 0x00021300u;
            constexpr uint32_t kInvalidDstB = 0xE0000100u; // unsupported guest segment

            std::array<uint8_t, 8> payloadA{};
            for (size_t i = 0; i < payloadA.size(); ++i)
            {
                payloadA[i] = static_cast<uint8_t>(0x70u + i);
            }
            std::array<uint8_t, 8> payloadB{};
            for (size_t i = 0; i < payloadB.size(); ++i)
            {
                payloadB[i] = static_cast<uint8_t>(0x90u + i);
            }

            std::memcpy(env.rdram.data() + kSrcA, payloadA.data(), payloadA.size());
            std::memcpy(env.rdram.data() + kSrcB, payloadB.data(), payloadB.size());
            std::memset(env.rdram.data() + kDstA, 0x5Au, payloadA.size());

            const Ps2SifDmaTransfer descs[2] = {
                {kSrcA, kDstA, static_cast<int32_t>(payloadA.size()), 0},
                {kSrcB, kInvalidDstB, static_cast<int32_t>(payloadB.size()), 0}};
            std::memcpy(env.rdram.data() + kDescAddr, descs, sizeof(descs));

            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 2u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0, "sceSifSetDma should fail when any descriptor is invalid");

            const std::array<uint8_t, 8> expectedUnchanged{
                0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};
            t.IsTrue(std::memcmp(env.rdram.data() + kDstA, expectedUnchanged.data(), expectedUnchanged.size()) == 0,
                     "failed multi-descriptor sceSifSetDma should not partially write earlier descriptors");
        });

        tc.Run("sceSifSetDma enforces descriptor count limit", [](TestCase &t)
        {
            TestEnv env;
            constexpr uint32_t kDescAddr = 0x00022000u;

            setRegU32(env.ctx, 4, kDescAddr);
            setRegU32(env.ctx, 5, 33u);
            ps2_stubs::sceSifSetDma(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0, "sceSifSetDma should reject count > 32");
        });

        tc.Run("sceSifGetOtherData copies payload and writes receive metadata", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kRdAddr = 0x00023000u;
            constexpr uint32_t kSrcAddr = 0x00023100u;
            constexpr uint32_t kDstAddr = 0x00023200u;
            constexpr uint32_t kSize = 20u;

            std::array<uint8_t, kSize> payload{};
            for (size_t i = 0; i < payload.size(); ++i)
            {
                payload[i] = static_cast<uint8_t>((i * 7u) & 0xFFu);
            }
            t.IsTrue(env.runtime.writeIopMemory(kSrcAddr, payload.data(), payload.size()),
                     "test setup should populate physical IOP RAM");
            std::memset(env.rdram.data() + kDstAddr, 0, payload.size());
            std::memset(env.rdram.data() + kRdAddr, 0, sizeof(SifRpcReceiveData));

            setRegU32(env.ctx, 4, kRdAddr);
            setRegU32(env.ctx, 5, kSrcAddr);
            setRegU32(env.ctx, 6, kDstAddr);
            setRegU32(env.ctx, 7, kSize);
            ps2_stubs::sceSifGetOtherData(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), 0, "sceSifGetOtherData should succeed for valid transfer");

            t.IsTrue(std::memcmp(env.rdram.data() + kDstAddr, payload.data(), payload.size()) == 0,
                     "sceSifGetOtherData should copy payload");

            const SifRpcReceiveData rd = *reinterpret_cast<const SifRpcReceiveData *>(env.rdram.data() + kRdAddr);
            t.Equals(rd.src, kSrcAddr, "receive metadata src should be populated");
            t.Equals(rd.dest, kDstAddr, "receive metadata dest should be populated");
            t.Equals(static_cast<uint32_t>(rd.size), kSize, "receive metadata size should be populated");
        });

        tc.Run("sceSifGetOtherData rejects unsupported guest segments", [](TestCase &t)
        {
            TestEnv env;

            constexpr uint32_t kRdAddr = 0x00024000u;
            constexpr uint32_t kDstAddr = 0x00024100u;
            constexpr uint32_t kInvalidSrcAddr = 0xE0000200u;
            constexpr uint32_t kSize = 16u;

            std::memset(env.rdram.data() + kDstAddr, 0xA5, kSize);
            writeGuestU32(env.rdram.data(), kRdAddr + 0x10u, 0x11111111u);
            writeGuestU32(env.rdram.data(), kRdAddr + 0x14u, 0x22222222u);
            writeGuestU32(env.rdram.data(), kRdAddr + 0x18u, 0x33333333u);

            setRegU32(env.ctx, 4, kRdAddr);
            setRegU32(env.ctx, 5, kInvalidSrcAddr);
            setRegU32(env.ctx, 6, kDstAddr);
            setRegU32(env.ctx, 7, kSize);
            ps2_stubs::sceSifGetOtherData(env.rdram.data(), &env.ctx, &env.runtime);
            t.Equals(getRegS32(env.ctx, 2), -1, "sceSifGetOtherData should fail for unsupported source segment");

            std::array<uint8_t, kSize> expected{};
            expected.fill(0xA5u);
            t.IsTrue(std::memcmp(env.rdram.data() + kDstAddr, expected.data(), expected.size()) == 0,
                     "failed sceSifGetOtherData should not modify destination");
            t.Equals(readGuestU32(env.rdram.data(), kRdAddr + 0x10u), 0x11111111u,
                     "failed sceSifGetOtherData should not overwrite rd metadata");
        });
    });
}
