#include "ps2_iop_host.h"

#include "ps2_runtime.h"
#include "ps2_stubs.h"
#include "Kernel/Stubs/SIF.h"
#include "Kernel/Stubs/CD.h"
#include "runtime/ps2_memory.h"
#include "Kernel/Stubs/MemoryCard.h"
#include "Kernel/Syscalls/Common.h"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <sys/types.h>
#endif

PS2IopHostAdapter::CallScope::CallScope(PS2IopHostAdapter &owner, R5900Context *context, uint8_t *rdram)
    : m_lock(owner.m_callMutex),
      m_owner(&owner),
      m_previousContext(owner.m_activeContext),
      m_previousRdram(owner.m_activeRdram),
      m_previousToken(owner.m_activeToken)
{
    m_token = owner.m_nextToken++;
    if (m_token == 0)
    {
        m_token = owner.m_nextToken++;
    }
    owner.m_activeContext = context;
    owner.m_activeRdram = rdram;
    owner.m_activeToken = m_token;
}

PS2IopHostAdapter::CallScope::~CallScope()
{
    release();
}

PS2IopHostAdapter::CallScope::CallScope(CallScope &&other) noexcept
    : m_lock(std::move(other.m_lock)),
      m_owner(std::exchange(other.m_owner, nullptr)),
      m_previousContext(other.m_previousContext),
      m_previousRdram(other.m_previousRdram),
      m_previousToken(other.m_previousToken),
      m_token(other.m_token)
{
}

PS2IopHostAdapter::CallScope &PS2IopHostAdapter::CallScope::operator=(CallScope &&other) noexcept
{
    if (this != &other)
    {
        release();
        m_lock = std::move(other.m_lock);
        m_owner = std::exchange(other.m_owner, nullptr);
        m_previousContext = other.m_previousContext;
        m_previousRdram = other.m_previousRdram;
        m_previousToken = other.m_previousToken;
        m_token = other.m_token;
    }
    return *this;
}

void PS2IopHostAdapter::CallScope::release()
{
    if (m_owner)
    {
        m_owner->m_activeContext = m_previousContext;
        m_owner->m_activeRdram = m_previousRdram;
        m_owner->m_activeToken = m_previousToken;
        m_owner = nullptr;
    }
}

PS2IopHostAdapter::PS2IopHostAdapter(PS2Runtime &runtime)
    : m_runtime(runtime)
{
}

PS2IopHostAdapter::~PS2IopHostAdapter()
{
    std::lock_guard<std::mutex> lock(m_hostFileMutex);
    for (auto &[handle, file] : m_hostFiles)
    {
        (void)handle;
        if (file.stream)
        {
            std::fclose(file.stream);
        }
    }
    m_hostFiles.clear();
}

PS2IopHostAdapter::CallScope PS2IopHostAdapter::enterCall(R5900Context *context, uint8_t *rdram)
{
    return CallScope(*this, context, rdram);
}

bool PS2IopHostAdapter::guestRange(uint32_t address, size_t size, uint8_t *&begin) const
{
    begin = nullptr;
    uint8_t *const rdram = m_activeRdram
                               ? m_activeRdram
                               : m_runtime.memory().getRDRAM();
    if (!rdram)
    {
        return false;
    }
    if (size == 0)
    {
        begin = getMemPtr(rdram, address);
        return begin != nullptr;
    }
    if (size - 1 > std::numeric_limits<uint32_t>::max() - address)
    {
        return false;
    }
    uint8_t *const first = getMemPtr(rdram, address);
    uint8_t *const last = getMemPtr(rdram, address + static_cast<uint32_t>(size - 1));
    if (!first || !last || last < first || static_cast<size_t>(last - first) != size - 1)
    {
        return false;
    }
    begin = first;
    return true;
}

bool PS2IopHostAdapter::readGuest(uint32_t address, void *destination, size_t size) const
{
    if (!destination && size != 0)
    {
        return false;
    }
    if (ps2_stubs::isSifIopHeapAddress(address))
    {
        return ps2_stubs::readSifIopHeap(address, destination, size);
    }

    uint8_t *source = nullptr;
    if (!guestRange(address, size, source))
    {
        return false;
    }
    if (size != 0)
    {
        std::memcpy(destination, source, size);
    }
    return true;
}

bool PS2IopHostAdapter::writeGuest(uint32_t address, const void *source, size_t size)
{
    // TEMP-EXPERIMENT: catch the d0/cc/cursor writer. Revert.
    if (address < 0x5611E0u && address + size > 0x5611C0u)
    {
        static int d0w = 0;
        if (d0w < 40)
        {
            ++d0w;
            const uint8_t *b = static_cast<const uint8_t *>(source);
            std::printf("[d0-watch] addr=0x%08x size=0x%zx data=%02x%02x%02x%02x...\n",
                        address,
                        size,
                        size > 0 ? b[0] : 0,
                        size > 1 ? b[1] : 0,
                        size > 2 ? b[2] : 0,
                        size > 3 ? b[3] : 0);
            std::fflush(stdout);
        }
    }
    if (!source && size != 0)
    {
        return false;
    }
    if (ps2_stubs::isSifIopHeapAddress(address))
    {
        return ps2_stubs::writeSifIopHeap(address, source, size);
    }

    uint8_t *destination = nullptr;
    if (!guestRange(address, size, destination))
    {
        return false;
    }
    if (size != 0)
    {
        uint8_t *const rdram = m_activeRdram ? m_activeRdram : m_runtime.memory().getRDRAM();
        ps2TraceGuestRangeWrite(rdram, address, static_cast<uint32_t>(size), "IopHost::writeGuest", nullptr);
        // Temporary decoder refill probe: identify asynchronous writes to the
        // bitstream state (qword count lives at +0x10).  This stays in the host
        // adapter so it does not force generated game objects to rebuild.
        const uint32_t norm = address & 0x1FFFFFFFu;
        if (norm <= 0x5611DCu && norm + static_cast<uint32_t>(size) > 0x5611DCu)
        {
            const auto *bytes = static_cast<const uint8_t *>(source);
            std::cerr << "[probe:iop-boot-global-write] addr=0x" << std::hex << address
                      << " size=0x" << size << " bytes=";
            const size_t shown = std::min<size_t>(size, 16u);
            for (size_t i = 0; i < shown; ++i)
                std::cerr << static_cast<unsigned>(bytes[i]);
            std::cerr << std::dec << '\n';
        }
        if (norm < 0x0101D470u && norm + static_cast<uint32_t>(size) > 0x0101D440u)
        {
            const auto *bytes = static_cast<const uint8_t *>(source);
            std::cerr << "[probe:decoder-range] addr=0x" << std::hex << address
                      << " size=0x" << size << " bytes=";
            const size_t shown = std::min<size_t>(size, 0x30u);
            for (size_t i = 0; i < shown; ++i)
                std::cerr << static_cast<unsigned>(bytes[i] >> 4)
                          << static_cast<unsigned>(bytes[i] & 0xFu);
            std::cerr << std::dec << std::endl;
        }
        std::memcpy(destination, source, size);
        // Temporary IOP-to-EE integrity probe.  The Katamari stream is copied
        // in small staging chunks; compare the source and destination bytes
        // immediately after each copy to distinguish a transport fault from
        // a later guest overwrite.  Keep this in the adapter (rather than a
        // generated header) so diagnostics do not invalidate the translation
        // cache.
        {
            const uint32_t normEnd = norm + static_cast<uint32_t>(size);
            if (norm < 0x01E4D080u + 0x1A1770u && normEnd > 0x01E4D080u)
            {
                static uint32_t streamWriteProbeCount = 0u;
                if (streamWriteProbeCount < 24u)
                {
                    const auto *bytes = static_cast<const uint8_t *>(source);
                    uint64_t sourceHash = 1469598103934665603ull;
                    uint64_t destHash = 1469598103934665603ull;
                    for (size_t i = 0; i < size; ++i)
                    {
                        sourceHash ^= bytes[i];
                        sourceHash *= 1099511628211ull;
                        destHash ^= destination[i];
                        destHash *= 1099511628211ull;
                    }
                    std::cerr << "[probe:iop-host-write] addr=0x" << std::hex << address
                              << " size=0x" << size << " srcfnv=0x" << sourceHash
                              << " dstfnv=0x" << destHash << " first=";
                    const size_t shown = std::min<size_t>(size, 16u);
                    for (size_t i = 0; i < shown; ++i)
                        std::cerr << (i ? " " : "") << static_cast<unsigned>(destination[i]);
                    std::cerr << std::dec << '\n';
                    ++streamWriteProbeCount;
                }
                if ((norm < 0x01F2D080u && normEnd > 0x01F0D080u) ||
                    (norm < 0x01FEE7F0u && normEnd > 0x01FCD080u))
                {
                    std::cerr << "[probe:iop-host-overlap] addr=0x" << std::hex << address
                              << " size=0x" << size << " norm=0x" << norm << std::dec << '\n';
                }
            }
        }
    }
    return true;
}

bool PS2IopHostAdapter::zeroGuest(uint32_t address, size_t size)
{
    if (ps2_stubs::isSifIopHeapAddress(address))
    {
        return ps2_stubs::zeroSifIopHeap(address, size);
    }

    uint8_t *destination = nullptr;
    if (!guestRange(address, size, destination))
    {
        return false;
    }
    if (size != 0)
    {
        uint8_t *const rdram = m_activeRdram ? m_activeRdram : m_runtime.memory().getRDRAM();
        ps2TraceGuestRangeWrite(rdram, address, static_cast<uint32_t>(size), "IopHost::zeroGuest", nullptr);
        std::memset(destination, 0, size);
    }
    return true;
}

bool PS2IopHostAdapter::normalizeGuestAddress(uint32_t address, uint32_t &normalized) const
{
    if (ps2_stubs::isSifIopHeapAddress(address))
    {
        normalized = address;
        return ps2_stubs::isSifIopHeapRange(address, 0u);
    }

    bool scratchpad = false;
    if (!ps2ResolveGuestPointer(address, normalized, scratchpad) || scratchpad)
    {
        normalized = 0;
        return false;
    }
    return true;
}

uint32_t PS2IopHostAdapter::allocateIopHandle(ps2x::iop::IopHandleKind kind)
{
    uint8_t *const rdram = m_activeRdram
                               ? m_activeRdram
                               : m_runtime.memory().getRDRAM();
    if (!rdram)
    {
        return 0;
    }
    return kind == ps2x::iop::IopHandleKind::RpcPacket
               ? rpcAllocPacketAddr(rdram)
               : rpcAllocServerAddr(rdram);
}

uint32_t PS2IopHostAdapter::allocateIopHeap(uint32_t size)
{
    return ps2_stubs::allocateSifIopHeap(size);
}

bool PS2IopHostAdapter::freeIopHeap(uint32_t address)
{
    return ps2_stubs::freeSifIopHeap(address);
}

uint32_t PS2IopHostAdapter::allocateGuest(uint32_t size, uint32_t alignment)
{
    return m_runtime.guestMalloc(size, alignment);
}

void PS2IopHostAdapter::freeGuest(uint32_t address)
{
    m_runtime.guestFree(address);
}

void PS2IopHostAdapter::audioCommand(uint32_t sid,
                                     uint32_t function,
                                     ps2x::iop::GuestBuffer send,
                                     ps2x::iop::GuestBuffer receive)
{
    uint8_t *sendPointer = nullptr;
    uint8_t *receivePointer = nullptr;
    std::vector<uint8_t> sendStorage;
    std::vector<uint8_t> receiveStorage;
    const bool sendIsIopHeap = send.address != 0u &&
                               ps2_stubs::isSifIopHeapAddress(send.address);
    const bool receiveIsIopHeap = receive.address != 0u &&
                                  ps2_stubs::isSifIopHeapAddress(receive.address);
    if (sendIsIopHeap)
    {
        sendStorage.resize(send.size);
        if (!ps2_stubs::readSifIopHeap(send.address, sendStorage.data(), sendStorage.size()))
        {
            sendStorage.clear();
        }
        else
        {
            sendPointer = sendStorage.data();
        }
    }
    if (receiveIsIopHeap)
    {
        receiveStorage.resize(receive.size);
        if (!ps2_stubs::readSifIopHeap(receive.address,
                                       receiveStorage.data(),
                                       receiveStorage.size()))
        {
            receiveStorage.clear();
        }
        else
        {
            receivePointer = receiveStorage.data();
        }
    }
    if (send.address && !guestRange(send.address, send.size, sendPointer))
    {
        if (!sendIsIopHeap)
        {
            sendPointer = nullptr;
        }
    }
    if (receive.address && !guestRange(receive.address, receive.size, receivePointer))
    {
        if (!receiveIsIopHeap)
        {
            receivePointer = nullptr;
        }
    }
    m_runtime.audioBackend().onSoundCommand(sid,
                                            function,
                                            sendPointer,
                                            send.size,
                                            receivePointer,
                                            receive.size);
    if (receivePointer && receiveIsIopHeap)
    {
        (void)ps2_stubs::writeSifIopHeap(receive.address,
                                         receiveStorage.data(),
                                         receiveStorage.size());
    }
}

std::string PS2IopHostAdapter::hostPath(ps2x::iop::HostPathKind kind) const
{
    const PS2Runtime::IoPaths &paths = PS2Runtime::getIoPaths();
    switch (kind)
    {
    case ps2x::iop::HostPathKind::CdRoot:
        return paths.cdRoot.string();
    case ps2x::iop::HostPathKind::CdImage:
        return paths.cdImage.string();
    case ps2x::iop::HostPathKind::HostRoot:
        return paths.hostRoot.string();
    case ps2x::iop::HostPathKind::MemoryCardRoot:
        return paths.mcRoot.string();
    case ps2x::iop::HostPathKind::ElfDirectory:
    default:
        return paths.elfDirectory.string();
    }
}

std::string PS2IopHostAdapter::translateGuestPath(std::string_view path) const
{
    return translatePs2Path(std::string(path).c_str());
}

uint64_t PS2IopHostAdapter::openHostFile(std::string_view path)
{
    if (path.empty())
    {
        return 0u;
    }

    const std::filesystem::path hostPath{std::string(path)};
#if defined(_WIN32)
    std::FILE *stream = ::_wfopen(hostPath.c_str(), L"rb");
#else
    std::FILE *stream = std::fopen(hostPath.string().c_str(), "rb");
#endif
    if (!stream)
    {
        return 0u;
    }

    std::error_code error;
    const uint64_t size = std::filesystem::file_size(hostPath, error);
    if (error)
    {
        std::fclose(stream);
        return 0u;
    }

    std::lock_guard<std::mutex> lock(m_hostFileMutex);
    uint64_t handle = m_nextHostFileHandle++;
    if (handle == 0u)
    {
        handle = m_nextHostFileHandle++;
    }
    m_hostFiles.emplace(handle, HostFile{stream, size});
    return handle;
}

bool PS2IopHostAdapter::hostFileSize(uint64_t handle, uint64_t &size) const
{
    size = 0u;
    std::lock_guard<std::mutex> lock(m_hostFileMutex);
    const auto it = m_hostFiles.find(handle);
    if (it == m_hostFiles.end() || !it->second.stream)
    {
        return false;
    }
    size = it->second.size;
    return true;
}

bool PS2IopHostAdapter::readHostFile(uint64_t handle,
                                     uint64_t offset,
                                     void *destination,
                                     size_t size,
                                     size_t &bytesRead)
{
    bytesRead = 0u;
    if (!destination && size != 0u)
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_hostFileMutex);
    const auto it = m_hostFiles.find(handle);
    if (it == m_hostFiles.end() || !it->second.stream)
    {
        return false;
    }

#if defined(_WIN32)
    if (offset > static_cast<uint64_t>(std::numeric_limits<int64_t>::max()) ||
        _fseeki64(it->second.stream, static_cast<int64_t>(offset), SEEK_SET) != 0)
#else
    if (offset > static_cast<uint64_t>(std::numeric_limits<off_t>::max()) ||
        fseeko(it->second.stream, static_cast<off_t>(offset), SEEK_SET) != 0)
#endif
    {
        return false;
    }

    bytesRead = size == 0u
                    ? 0u
                    : std::fread(destination, 1u, size, it->second.stream);
    if (bytesRead < size && std::ferror(it->second.stream))
    {
        std::clearerr(it->second.stream);
        return false;
    }
    return true;
}

void PS2IopHostAdapter::closeHostFile(uint64_t handle)
{
    std::FILE *stream = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_hostFileMutex);
        const auto it = m_hostFiles.find(handle);
        if (it == m_hostFiles.end())
        {
            return;
        }
        stream = it->second.stream;
        m_hostFiles.erase(it);
    }
    if (stream)
    {
        std::fclose(stream);
    }
}

bool PS2IopHostAdapter::readCdSectors(uint32_t lbn,
                                      uint32_t sectors,
                                      void *destination,
                                      size_t size)
{
    return ps2_stubs::readCdSectorsForIop(lbn, sectors, destination, size);
}

bool PS2IopHostAdapter::readPadState(int port, int slot, uint8_t *data, size_t size)
{
    return m_runtime.padBackend().readState(port, slot, data, size);
}

int32_t PS2IopHostAdapter::memoryCard(const ps2x::iop::MemoryCardRequest &request){
    using Handler = void (*)(uint8_t *, R5900Context *, PS2Runtime *);
    Handler handler = nullptr;
    switch (request.operation)
    {
    case ps2x::iop::MemoryCardOperation::Init:
        handler = ps2_stubs::sceMcInit;
        break;
    case ps2x::iop::MemoryCardOperation::GetInfo:
        handler = ps2_stubs::sceMcGetInfo;
        break;
    case ps2x::iop::MemoryCardOperation::Open:
        handler = ps2_stubs::sceMcOpen;
        break;
    case ps2x::iop::MemoryCardOperation::Close:
        handler = ps2_stubs::sceMcClose;
        break;
    case ps2x::iop::MemoryCardOperation::Seek:
        handler = ps2_stubs::sceMcSeek;
        break;
    case ps2x::iop::MemoryCardOperation::Read:
        handler = ps2_stubs::sceMcRead;
        break;
    case ps2x::iop::MemoryCardOperation::Write:
        handler = ps2_stubs::sceMcWrite;
        break;
    case ps2x::iop::MemoryCardOperation::Flush:
        handler = ps2_stubs::sceMcFlush;
        break;
    case ps2x::iop::MemoryCardOperation::Chdir:
        handler = ps2_stubs::sceMcChdir;
        break;
    case ps2x::iop::MemoryCardOperation::GetDir:
        handler = ps2_stubs::sceMcGetDir;
        break;
    case ps2x::iop::MemoryCardOperation::SetFileInfo:
        handler = ps2_stubs::sceMcSetFileInfo;
        break;
    case ps2x::iop::MemoryCardOperation::Delete:
        handler = ps2_stubs::sceMcDelete;
        break;
    case ps2x::iop::MemoryCardOperation::Format:
        handler = ps2_stubs::sceMcFormat;
        break;
    case ps2x::iop::MemoryCardOperation::Unformat:
        handler = ps2_stubs::sceMcUnformat;
        break;
    case ps2x::iop::MemoryCardOperation::Mkdir:
        handler = ps2_stubs::sceMcMkdir;
        break;
    }
    if (!handler)
    {
        return -1;
    }

    R5900Context context{};
    for (size_t i = 0; i < 4u; ++i)
    {
        setRegU32(&context, static_cast<int>(4 + i), request.arguments[i]);
    }

    // EE n32 ABI: the fifth argument travels in $t0, matching the sceMc* stubs.
    setRegU32(&context, 8, request.arguments[4]);

    handler(m_activeRdram ? m_activeRdram : m_runtime.memory().getRDRAM(),
            &context,
            &m_runtime);
    return ps2_stubs::getMemoryCardDebugSnapshot().lastResult;
}

bool PS2IopHostAdapter::hasGuestFunction(uint32_t address) const
{
    return m_runtime.hasFunction(address);
}

bool PS2IopHostAdapter::invokeGuestFunction(uint64_t callToken,
                                            uint32_t address,
                                            uint32_t a0,
                                            uint32_t a1,
                                            uint32_t a2,
                                            uint32_t a3,
                                            uint32_t *resultAddress)
{
    (void)callToken;
    (void)address;
    (void)a0;
    (void)a1;
    (void)a2;
    (void)a3;
    if (resultAddress)
    {
        *resultAddress = 0u;
    }
    return false;
}

void PS2IopHostAdapter::log(ps2x::iop::LogLevel level, std::string_view message)
{
    const char *prefix = "[ps2xIOP]";
    if (level == ps2x::iop::LogLevel::Warning)
    {
        prefix = "[ps2xIOP:warning]";
    }
    else if (level == ps2x::iop::LogLevel::Error)
    {
        prefix = "[ps2xIOP:error]";
    }
    std::cerr << prefix << ' ' << message << std::endl;
}
