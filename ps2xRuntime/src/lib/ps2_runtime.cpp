#include "ps2_runtime.h"
#include "ps2_log.h"
#include "ps2_stubs.h"
#include "ps2_syscalls.h"
#include "game_overrides.h"
#include "ps2_runtime_macros.h"
#include "runtime/gs/gs_frontend.h"
#include "runtime/ee_scheduler.h"
#include "runtime/code_overlays.h"
#include "ThreadNaming.h"
#include "Kernel/Stubs/Audio.h"
#include "Kernel/Stubs/GS.h"
#include "Kernel/Stubs/MPEG.h"
#include "Kernel/Stubs/SIF.h"
#include "ps2_host_backend.h"
#include "ps2_iop_host.h"
#include "ps2x/iop/iop_subsystem.h"

#include <iostream>
#include <fstream>
#include <algorithm>
#include <map>
#include <array>
#include <cctype>
#include <cstring>
#include <limits>
#include <chrono>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <sstream>
#include <mutex>

namespace
{
    struct MissingTargetReports
    {
        std::mutex mutex;
        std::map<const PS2Runtime *, std::vector<uint64_t>> targets;
    };

    MissingTargetReports &missingTargetReports()
    {
        // Global runtime instances may outlive function-local statics during
        // shutdown. Entries are erased with each VM; keep the registry alive.
        static auto *reports = new MissingTargetReports;
        return *reports;
    }
}

#ifndef PS2_TRACE_CALL_WATCH
#define PS2_TRACE_CALL_WATCH 0
#endif

#define ELF_MAGIC 0x464C457F // "\x7FELF" in little endian
#define ET_EXEC 2            // Executable file
#define EM_MIPS 8            // MIPS architecture
#define PT_LOAD 1            // Loadable segment

static constexpr int FB_WIDTH = 640;
static constexpr int FB_HEIGHT = 512;
static constexpr int DEFAULT_DISPLAY_HEIGHT = 448;
static constexpr uint32_t DEFAULT_FB_SIZE = FB_WIDTH * FB_HEIGHT * 4;
static constexpr uint32_t DEFAULT_FB_ADDR = (PS2_RAM_SIZE - DEFAULT_FB_SIZE - 0x10000u);
#if defined(PLATFORM_VITA)
static constexpr int HOST_WINDOW_WIDTH = 960;
static constexpr int HOST_WINDOW_HEIGHT = 544;
#else
static constexpr int HOST_WINDOW_WIDTH = FB_WIDTH;
static constexpr int HOST_WINDOW_HEIGHT = DEFAULT_DISPLAY_HEIGHT;
#endif
struct ElfHeader
{
    uint32_t magic;
    uint8_t elf_class;
    uint8_t endianness;
    uint8_t version;
    uint8_t os_abi;
    uint8_t abi_version;
    uint8_t padding[7];
    uint16_t type;
    uint16_t machine;
    uint32_t version2;
    uint32_t entry;
    uint32_t phoff;
    uint32_t shoff;
    uint32_t flags;
    uint16_t ehsize;
    uint16_t phentsize;
    uint16_t phnum;
    uint16_t shentsize;
    uint16_t shnum;
    uint16_t shstrndx;
};

struct ProgramHeader
{
    uint32_t type;
    uint32_t offset;
    uint32_t vaddr;
    uint32_t paddr;
    uint32_t filesz;
    uint32_t memsz;
    uint32_t flags;
    uint32_t align;
};

namespace
{
    constexpr uint32_t kGuestHeapDefaultBase = 0x00100000u;
    constexpr uint32_t kGuestHeapDefaultAlignment = 16u;
    constexpr uint32_t kGuestHeapSafetyPad = 0x1000u;
    constexpr uint32_t kGuestHeapHardLimit = 0x01F00000u;

    constexpr uint32_t COP0_CAUSE_EXCCODE_MASK = 0x0000007Cu;
    constexpr uint32_t COP0_CAUSE_BD = 0x80000000u;
    constexpr uint32_t COP0_STATUS_EXL = 0x00000002u;
    constexpr uint32_t COP0_STATUS_BEV = 0x00400000u;
    constexpr uint32_t EXCEPTION_VECTOR_GENERAL = 0x80000080u;
    constexpr uint32_t EXCEPTION_VECTOR_TLB_REFILL = 0x80000000u;
    constexpr uint32_t EXCEPTION_VECTOR_BOOT = 0xBFC00200u;

    struct DispatchHistory
    {
        std::array<uint32_t, 64> pcs{};
        uint32_t next = 0u;
        bool wrapped = false;
    };

    thread_local DispatchHistory g_dispatchHistory;

    bool computeFileCrc32(const std::string &path, uint32_t &crcOut)
    {
        std::ifstream file(path, std::ios::binary);
        if (!file.is_open())
        {
            return false;
        }

        static const std::array<uint32_t, 256> table = []
        {
            std::array<uint32_t, 256> values{};
            for (uint32_t i = 0; i < values.size(); ++i)
            {
                uint32_t value = i;
                for (uint32_t bit = 0; bit < 8; ++bit)
                {
                    value = (value & 1u) ? (0xEDB88320u ^ (value >> 1u)) : (value >> 1u);
                }
                values[i] = value;
            }
            return values;
        }();

        uint32_t crc = 0xFFFFFFFFu;
        std::array<uint8_t, 16 * 1024> buffer{};
        while (file.good())
        {
            file.read(reinterpret_cast<char *>(buffer.data()), static_cast<std::streamsize>(buffer.size()));
            const std::streamsize count = file.gcount();
            for (std::streamsize i = 0; i < count; ++i)
            {
                crc = table[(crc ^ buffer[static_cast<size_t>(i)]) & 0xFFu] ^ (crc >> 8u);
            }
        }
        if (file.bad())
        {
            return false;
        }
        crcOut = ~crc;
        return true;
    }

    void pushDispatchPc(uint32_t pc)
    {
        DispatchHistory &h = g_dispatchHistory;
        h.pcs[h.next] = pc;
        h.next = (h.next + 1u) % static_cast<uint32_t>(h.pcs.size());
        if (h.next == 0u)
        {
            h.wrapped = true;
        }
    }

    std::string formatDispatchHistory()
    {
        const DispatchHistory &h = g_dispatchHistory;
        const uint32_t count = h.wrapped ? static_cast<uint32_t>(h.pcs.size()) : h.next;
        if (count == 0u)
        {
            return "(empty)";
        }

        std::ostringstream oss;
        bool first = true;
        for (uint32_t i = 0u; i < count; ++i)
        {
            const uint32_t idx = (h.next + h.pcs.size() - count + i) % static_cast<uint32_t>(h.pcs.size());
            if (!first)
            {
                oss << " -> ";
            }
            first = false;
            oss << "0x" << std::hex << h.pcs[idx];
        }
        return oss.str();
    }

    uint32_t selectExceptionVector(const R5900Context *ctx, bool tlbRefill)
    {
        if (ctx->cop0_status & COP0_STATUS_BEV)
        {
            return EXCEPTION_VECTOR_BOOT;
        }
        return tlbRefill ? EXCEPTION_VECTOR_TLB_REFILL : EXCEPTION_VECTOR_GENERAL;
    }

    void seedVu0IdleSuccess(R5900Context *ctx)
    {
        if (!ctx)
        {
            return;
        }

        ctx->vu0_clip_flags = 0;
        ctx->vu0_clip_flags2 = 0;
        ctx->vu0_mac_flags = 0;
        ctx->vu0_status = 0;
        ctx->vu0_q = 1.0f;
        ctx->vu0_r = _mm_castsi128_ps(_mm_set1_epi32(0x3F800000));
        ctx->vu0_vpu_stat = 0;
        ctx->vu0_vpu_stat2 = 0;
    }

    void copyVu0ContextToState(const R5900Context *ctx, VU1State &state)
    {
        std::memset(&state, 0, sizeof(state));

        for (uint32_t i = 0; i < 32u; ++i)
        {
            _mm_storeu_ps(state.vf[i], ctx->vu0_vf[i]);
        }
        for (uint32_t i = 0; i < 16u; ++i)
        {
            state.vi[i] = static_cast<int16_t>(ctx->vi[i]);
        }

        _mm_storeu_ps(state.acc, ctx->vu0_acc);
        state.q = ctx->vu0_q;
        state.p = ctx->vu0_p;
        state.i = ctx->vu0_i;
        alignas(16) uint32_t rWords[4]{};
        _mm_storeu_si128(reinterpret_cast<__m128i *>(rWords), _mm_castps_si128(ctx->vu0_r));
        state.r = 0x3F800000u | (rWords[0] & 0x007FFFFFu);
        state.pc = ctx->vu0_pc;
        state.mac = ctx->vu0_mac_flags;
        state.clip = ctx->vu0_clip_flags;
        state.status = ctx->vu0_status;
        state.itop = ctx->vu0_itop;
        state.dBitEnabled = (ctx->vu0_fbrst & (1u << 2)) != 0u;
        state.tBitEnabled = (ctx->vu0_fbrst & (1u << 3)) != 0u;

        state.vf[0][0] = 0.0f;
        state.vf[0][1] = 0.0f;
        state.vf[0][2] = 0.0f;
        state.vf[0][3] = 1.0f;
        state.vi[0] = 0;
    }

    void copyVu0StateToContext(const VU1State &state, R5900Context *ctx)
    {
        for (uint32_t i = 0; i < 32u; ++i)
        {
            ctx->vu0_vf[i] = _mm_loadu_ps(state.vf[i]);
        }
        for (uint32_t i = 0; i < 16u; ++i)
        {
            ctx->vi[i] = static_cast<uint16_t>(state.vi[i]);
        }

        ctx->vu0_acc = _mm_loadu_ps(state.acc);
        ctx->vu0_q = state.q;
        ctx->vu0_p = state.p;
        ctx->vu0_i = state.i;
        ctx->vu0_r = _mm_castsi128_ps(_mm_set1_epi32(static_cast<int32_t>(state.r)));
        ctx->vu0_mac_flags = state.mac;
        ctx->vu0_clip_flags = state.clip;
        ctx->vu0_clip_flags2 = state.clip;
        ctx->vu0_status = static_cast<uint16_t>(state.status);
        ctx->vu0_itop = state.itop;
        ctx->vu0_pc = state.pc;
        ctx->vu0_tpc = state.pc;
        ctx->vu0_vpu_stat = (ctx->vu0_vpu_stat & 0xFF00u) | (state.stoppedByD ? (1u << 1) : 0u) | (state.stoppedByT ? (1u << 2) : 0u);
        ctx->vu0_vpu_stat2 = 0;

        ctx->vu0_vf[0] = _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f);
        ctx->vi[0] = 0;
    }

    void raiseCop0Exception(R5900Context *ctx, uint32_t exceptionCode, bool tlbRefill = false)
    {
        if (ctx->in_delay_slot)
        {
            ctx->cop0_epc = ctx->branch_pc;
            ctx->cop0_cause = (ctx->cop0_cause & ~COP0_CAUSE_EXCCODE_MASK) |
                              ((exceptionCode << 2) & COP0_CAUSE_EXCCODE_MASK) |
                              COP0_CAUSE_BD;
        }
        else
        {
            ctx->cop0_epc = ctx->pc;
            ctx->cop0_cause = (ctx->cop0_cause & ~(COP0_CAUSE_EXCCODE_MASK | COP0_CAUSE_BD)) |
                              ((exceptionCode << 2) & COP0_CAUSE_EXCCODE_MASK);
        }

        ctx->cop0_status |= COP0_STATUS_EXL;
        ctx->pc = selectExceptionVector(ctx, tlbRefill);
        ctx->in_delay_slot = false;
    }

    std::filesystem::path normalizeAbsolutePath(const std::filesystem::path &path)
    {
        if (path.empty())
        {
            return {};
        }

#if defined(PLATFORM_VITA)
        const std::string generic = path.generic_string();
        const std::size_t colon = generic.find(':');
        if (colon != std::string::npos && colon != 0u)
        {
            const std::size_t slash = generic.find_first_of("/\\");
            if (slash == std::string::npos || colon < slash)
            {
                return path.lexically_normal();
            }
        }
#endif

        std::error_code ec;
        const std::filesystem::path absolute = std::filesystem::absolute(path, ec);
        if (ec)
        {
            return path.lexically_normal();
        }
        return absolute.lexically_normal();
    }

    PS2Runtime::IoPaths &runtimeIoPaths()
    {
        static PS2Runtime::IoPaths paths = []()
        {
            PS2Runtime::IoPaths defaults;
            std::error_code ec;
            const std::filesystem::path cwd = std::filesystem::current_path(ec);
            defaults.elfDirectory = ec ? std::filesystem::path(".") : cwd.lexically_normal();
            defaults.hostRoot = defaults.elfDirectory;
            defaults.cdRoot = defaults.elfDirectory;
            defaults.mcRoot = defaults.elfDirectory / "mc0";
            return defaults;
        }();

        return paths;
    }

    std::string readGuestPrintableString(const uint8_t *rdram, uint32_t addr, size_t maxLen)
    {
        std::string out;
        if (!rdram || maxLen == 0)
        {
            return out;
        }

        out.reserve(std::min<size_t>(maxLen, 64));
        for (size_t i = 0; i < maxLen; ++i)
        {
            const char ch = static_cast<char>(rdram[(addr + static_cast<uint32_t>(i)) & PS2_RAM_MASK]);
            if (ch == '\0')
            {
                break;
            }
            if (ch >= 0x20 && ch < 0x7F)
            {
                out.push_back(ch);
            }
            else
            {
                out.push_back('.');
            }
        }
        return out;
    }
}

static void UploadFrame(Texture2D &tex, PS2Runtime *rt, uint32_t &outWidth, uint32_t &outHeight)
{
    static std::vector<uint8_t> s_scratch;
    static std::vector<uint8_t> s_uploadBuffer(DEFAULT_FB_SIZE, 0u);

    // Presentation state belongs to the current runtime instance.  Do not
    // cache it in function statics: a second run can reuse the same tick
    // values and would otherwise display a stale framebuffer from the prior
    // run.  Latching every host frame is inexpensive compared with the
    // existing framebuffer copy and keeps presentation deterministic.
    rt->gs().latchHostPresentationFrame();

    s_scratch.clear();
    uint32_t width = 0u;
    uint32_t height = 0u;
    uint32_t displayFbp = 0u;
    uint32_t sourceFbp = 0u;
    bool usedPreferredDisplaySource = false;
    if (!rt->gs().copyLatchedHostPresentationFrame(s_scratch,
                                                   width,
                                                   height,
                                                   &displayFbp,
                                                   &sourceFbp,
                                                   &usedPreferredDisplaySource))
    {
        Image blank = GenImageColor(FB_WIDTH, FB_HEIGHT, MAGENTA);
        UpdateTexture(tex, blank.data);
        UnloadImage(blank);
        outWidth = FB_WIDTH;
        outHeight = DEFAULT_DISPLAY_HEIGHT;
        return;
    }

    (void)displayFbp;
    (void)sourceFbp;
    (void)usedPreferredDisplaySource;

    std::fill(s_uploadBuffer.begin(), s_uploadBuffer.end(), 0u);
    if (!s_scratch.empty() && width != 0u && height != 0u)
    {
        const uint32_t copyWidth = std::min<uint32_t>(width, FB_WIDTH);
        const uint32_t copyHeight = std::min<uint32_t>(height, FB_HEIGHT);
        const size_t srcRowBytes = static_cast<size_t>(width) * 4u;
        const size_t dstRowBytes = static_cast<size_t>(FB_WIDTH) * 4u;
        const size_t copyRowBytes = static_cast<size_t>(copyWidth) * 4u;
        for (uint32_t y = 0; y < copyHeight; ++y)
        {
            const size_t srcOffset = static_cast<size_t>(y) * srcRowBytes;
            const size_t dstOffset = static_cast<size_t>(y) * dstRowBytes;
            if (srcOffset + copyRowBytes > s_scratch.size() ||
                dstOffset + copyRowBytes > s_uploadBuffer.size())
            {
                break;
            }
            std::memcpy(s_uploadBuffer.data() + dstOffset, s_scratch.data() + srcOffset, copyRowBytes);
        }
    }

    UpdateTexture(tex, s_uploadBuffer.data());
    outWidth = width;
    outHeight = height;
}

PS2Runtime::PS2Runtime()
{
    // Diagnostics are intentionally retained during bring-up, but they must
    // not force a host-side flush for every guest event.  Keep the same log
    // stream and ordering while allowing libc to buffer writes in hot loops.
    std::cerr << std::nounitbuf;
    m_iopHost = std::make_unique<PS2IopHostAdapter>(*this);
    m_iopSubsystem = std::make_unique<ps2x::iop::IopSubsystem>(*m_iopHost);
    m_eeScheduler = std::make_unique<EeScheduler>(*this);
#if defined(PS2X_IOP_ENABLE_PLUGINS) && PS2X_IOP_ENABLE_PLUGINS && \
    !defined(PLATFORM_VITA) && (defined(_WIN32) || defined(__linux__))
    if (const char *applicationDirectory = GetApplicationDirectory();
        applicationDirectory && applicationDirectory[0] != '\0')
    {
        m_iopSubsystem->setPluginSearchPaths({std::filesystem::path(applicationDirectory) / "iop_plugins"});
    }
#endif

    // Assign rather than memset: R5900Context's constructor zeroes itself and
    // then applies the COP0 reset values, which a memset here would discard.
    m_cpuContext = R5900Context{};

    // R0 is always zero in MIPS
    m_cpuContext.r[0] = _mm_set1_epi32(0);
    m_cpuContext.vu0_vf[0] = _mm_set_ps(1.0f, 0.0f, 0.0f, 0.0f);
    m_cpuContext.vu0_q = 1.0f;
    m_cpuContext.vu0_r = _mm_castsi128_ps(_mm_set1_epi32(0x3F800000));

    // Stack pointer (SP) and global pointer (GP) will be set by the loaded ELF

    m_loadedModules.clear();
    m_guestHeapBlocks.clear();
    m_guestHeapBase = kGuestHeapDefaultBase;
    m_guestHeapEnd = kGuestHeapDefaultBase;
    m_guestHeapLimit = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
    m_guestHeapSuggestedBase = kGuestHeapDefaultBase;
    m_guestHeapConfigured = false;
    m_asyncCallbackStackFloor = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
    m_asyncCallbackStackTop = PS2_RAM_SIZE;
}

void PS2Runtime::setDebugUiCallbacks(DebugUiCallback initCallback,
                                     DebugUiCallback drawCallback,
                                     DebugUiCallback shutdownCallback,
                                     void *userData)
{
    if (m_debugUiInitialized && m_debugUiShutdownCallback)
    {
        m_debugUiShutdownCallback(*this, m_debugUiUserData);
        m_debugUiInitialized = false;
    }

    m_debugUiInitCallback = initCallback;
    m_debugUiDrawCallback = drawCallback;
    m_debugUiShutdownCallback = shutdownCallback;
    m_debugUiUserData = userData;
}

PS2Runtime::~PS2Runtime()
{
    resetMissingFunctionReportOnce();
    ps2x::clearCodeOverlays(this);
    try
    {
        requestStop();
        ps2_stubs::resetSifRuntimeState(this);
        m_iopSubsystem.reset();
        m_iopHost.reset();
#if defined(PLATFORM_VITA)
        m_audioBackend.stopAll();
        m_audioBackend.setAudioReady(false);
#else
        if (IsAudioDeviceReady())
        {
            CloseAudioDevice();
            m_audioBackend.setAudioReady(false);
        }
#endif
        if (m_debugUiInitialized && m_debugUiShutdownCallback)
        {
            m_debugUiShutdownCallback(*this, m_debugUiUserData);
            m_debugUiInitialized = false;
        }

        if (IsWindowReady())
        {
            CloseWindow();
        }

        m_loadedModules.clear();
    }
    catch (const std::exception &e)
    {
        std::cerr << "[~PS2Runtime] cleanup exception: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "[~PS2Runtime] cleanup exception: unknown" << std::endl;
    }
}

void PS2Runtime::setIopPluginSearchPaths(std::vector<std::filesystem::path> paths)
{
    m_iopSubsystem->setPluginSearchPaths(std::move(paths));
}

ps2x::iop::RpcAbi PS2Runtime::selectIopRpcAbi(const ps2x::iop::RpcAbiRequest &request) const
{
    return m_iopSubsystem->selectRpcAbi(request);
}

bool PS2Runtime::hasIopRpcService(uint32_t sid) const
{
    return m_iopSubsystem && m_iopSubsystem->hasRpcService(sid);
}

uint32_t PS2Runtime::allocateIopHandle(ps2x::iop::IopHandleKind kind)
{
    return m_iopHost ? m_iopHost->allocateIopHandle(kind) : 0u;
}

ps2x::iop::RpcResult PS2Runtime::handleIopRpc(uint8_t *rdram, R5900Context *ctx, ps2x::iop::RpcRequest request)
{
    auto scope = m_iopHost->enterCall(ctx, rdram);
    request.callToken = scope.token();
    return m_iopSubsystem->handleRpc(request);
}

void PS2Runtime::notifyIopSifTransfer(uint8_t *rdram, const ps2x::iop::SifTransfer &transfer)
{
    auto scope = m_iopHost->enterCall(nullptr, rdram);
    m_iopSubsystem->onSifTransfer(transfer);
}

void PS2Runtime::resetIop()
{
    ps2_stubs::resetSifRuntimeState(this);
    m_iopSubsystem->reset();
}

ps2x::iop::DebugSnapshot PS2Runtime::iopDebugSnapshot() const
{
    return m_iopSubsystem->debugSnapshot();
}

bool PS2Runtime::syncCoreSubsystems()
{
    uint8_t *const rdram = m_memory.getRDRAM();
    uint8_t *const gsVram = m_memory.getGSVRAM();
    if (!rdram || !gsVram)
    {
        return false;
    }

    if (m_boundRdram == rdram && m_boundGSVram == gsVram)
    {
        return true;
    }

    m_gs.init(gsVram, static_cast<uint32_t>(PS2_GS_VRAM_SIZE), &m_memory.gs());
    m_gifArbiter.setProcessPacketFn([this](const uint8_t *data, uint32_t size)
                                    { m_gs.processGIFPacket(data, size); });
    m_memory.setGifArbiter(&m_gifArbiter);
    m_memory.setVu1MscalCallback([this](uint32_t startPC, uint32_t top, uint32_t itop)
                                 {
                                     static std::atomic<uint32_t> vu1MscalProbes{0u};
                                     const uint32_t vu1Probe = vu1MscalProbes.fetch_add(1u, std::memory_order_relaxed);
                                     if (vu1Probe < 64u)
                                         std::cerr << "[probe:vu1-mscal] start=0x" << std::hex << startPC
                                                   << " top=0x" << top << " itop=0x" << itop << std::dec << '\n';
                                     R5900Context *cpuContext = m_eeScheduler ? m_eeScheduler->currentContext() : nullptr;
                                     if (!cpuContext)
                                     {
                                         cpuContext = &m_cpuContext;
                                     }
                                     m_vu1.state().dBitEnabled =
                                         (cpuContext->vu0_fbrst & (1u << 10)) != 0u;
                                     m_vu1.state().tBitEnabled =
                                         (cpuContext->vu0_fbrst & (1u << 11)) != 0u;
                                     m_vu1.execute(m_memory.getVU1Code(), PS2_VU1_CODE_SIZE,
                                                   m_memory.getVU1Data(), PS2_VU1_DATA_SIZE,
                                                   m_gs, &m_memory, startPC, top, itop, 65536);
                                     cpuContext->vu0_vpu_stat =
                                         (cpuContext->vu0_vpu_stat & ~0x0600u) |
                                         (m_vu1.state().stoppedByD ? 0x0200u : 0u) |
                                         (m_vu1.state().stoppedByT ? 0x0400u : 0u); });
    m_memory.setVu1MscntCallback([this](uint32_t top, uint32_t itop)
                                 {
                                     static std::atomic<uint32_t> vu1MscntProbes{0u};
                                     const uint32_t vu1Probe = vu1MscntProbes.fetch_add(1u, std::memory_order_relaxed);
                                     if (vu1Probe < 64u)
                                         std::cerr << "[probe:vu1-mscnt] top=0x" << std::hex << top
                                                   << " itop=0x" << itop << std::dec << '\n';
                                     R5900Context *cpuContext = m_eeScheduler ? m_eeScheduler->currentContext() : nullptr;
                                     if (!cpuContext)
                                     {
                                         cpuContext = &m_cpuContext;
                                     }
                                     m_vu1.state().dBitEnabled =
                                         (cpuContext->vu0_fbrst & (1u << 10)) != 0u;
                                     m_vu1.state().tBitEnabled =
                                         (cpuContext->vu0_fbrst & (1u << 11)) != 0u;
                                     m_vu1.resume(m_memory.getVU1Code(), PS2_VU1_CODE_SIZE,
                                                  m_memory.getVU1Data(), PS2_VU1_DATA_SIZE,
                                                  m_gs, &m_memory, top, itop, 65536);
                                     cpuContext->vu0_vpu_stat =
                                         (cpuContext->vu0_vpu_stat & ~0x0600u) |
                                         (m_vu1.state().stoppedByD ? 0x0200u : 0u) |
                                         (m_vu1.state().stoppedByT ? 0x0400u : 0u); });
    resetIop();
    m_vu0.reset();
    m_vu1.reset();

    m_boundRdram = rdram;
    m_boundGSVram = gsVram;
    return true;
}

bool PS2Runtime::initialize(const char *title)
{
    try
    {
        if (!m_memory.initialize())
        {
            std::cerr << "Failed to initialize PS2 memory" << std::endl;
            return false;
        }

        if (!syncCoreSubsystems())
        {
            std::cerr << "Failed to bind runtime core subsystems" << std::endl;
            return false;
        }
#if defined(PS2X_IOP_ENABLE_PLUGINS) && PS2X_IOP_ENABLE_PLUGINS && \
    !defined(PLATFORM_VITA) && (defined(_WIN32) || defined(__linux__))
        std::string pluginError;
        if (!m_iopSubsystem->loadPlugins(&pluginError))
        {
            std::cerr << "Failed to load IOP plugins: " << pluginError << std::endl;
            return false;
        }
#endif
#if defined(PLATFORM_VITA)
        InitWindow(HOST_WINDOW_WIDTH, HOST_WINDOW_HEIGHT, title); // raylib vita does not support audio
#else
        SetConfigFlags(FLAG_WINDOW_RESIZABLE);
        InitWindow(HOST_WINDOW_WIDTH, HOST_WINDOW_HEIGHT, title);
        InitAudioDevice();
        m_audioBackend.setAudioReady(IsAudioDeviceReady());
#endif
        SetTargetFPS(60);
        if (m_debugUiInitCallback)
        {
            m_debugUiInitCallback(*this, m_debugUiUserData);
            m_debugUiInitialized = true;
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to initialize PS2 runtime: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Failed to initialize PS2 runtime: unknown exception" << std::endl;
    }

    return false;
}

bool PS2Runtime::loadELF(const std::string &elfPath)
{
    configureIoPathsFromElf(elfPath);

    std::ifstream file(elfPath, std::ios::binary);
    if (!file)
    {
        std::cerr << "Failed to open ELF file: " << elfPath << std::endl;
        return false;
    }

    file.seekg(0, std::ios::end);
    const std::streamoff fileSize = file.tellg();
    if (fileSize < static_cast<std::streamoff>(sizeof(ElfHeader)))
    {
        std::cerr << "ELF file is too small: " << elfPath << std::endl;
        return false;
    }
    file.seekg(0, std::ios::beg);

    ElfHeader header{};
    if (!file.read(reinterpret_cast<char *>(&header), sizeof(header)))
    {
        std::cerr << "Failed to read ELF header from: " << elfPath << std::endl;
        return false;
    }

    if (header.magic != ELF_MAGIC)
    {
        std::cerr << "Invalid ELF magic number" << std::endl;
        return false;
    }

    if (header.elf_class != 1u || header.endianness != 1u)
    {
        std::cerr << "Unsupported ELF format (expected 32-bit little-endian)." << std::endl;
        return false;
    }

    if (header.machine != EM_MIPS || header.type != ET_EXEC)
    {
        std::cerr << "Not a MIPS executable ELF file" << std::endl;
        return false;
    }

    if (header.phnum != 0u && header.phentsize < sizeof(ProgramHeader))
    {
        std::cerr << "Unsupported ELF program-header entry size: " << header.phentsize << std::endl;
        return false;
    }

    const uint64_t programHeaderTableEnd =
        static_cast<uint64_t>(header.phoff) +
        static_cast<uint64_t>(header.phnum) * static_cast<uint64_t>(header.phentsize);
    if (programHeaderTableEnd > static_cast<uint64_t>(fileSize))
    {
        std::cerr << "ELF program-header table is out of range." << std::endl;
        return false;
    }

    m_cpuContext.pc = header.entry;
    m_debugPc.store(m_cpuContext.pc, std::memory_order_relaxed);

    uint32_t maxLoadedRdramEnd = kGuestHeapDefaultBase;
    uint32_t moduleBase = std::numeric_limits<uint32_t>::max();
    uint32_t moduleEnd = 0u;
    bool loadedAnySegment = false;

    for (uint16_t i = 0; i < header.phnum; i++)
    {
        const uint64_t phOffset =
            static_cast<uint64_t>(header.phoff) +
            static_cast<uint64_t>(i) * static_cast<uint64_t>(header.phentsize);
        if (phOffset + sizeof(ProgramHeader) > static_cast<uint64_t>(fileSize))
        {
            std::cerr << "ELF program header " << i << " is out of range." << std::endl;
            return false;
        }

        ProgramHeader ph{};
        file.seekg(static_cast<std::streamoff>(phOffset), std::ios::beg);
        if (!file.read(reinterpret_cast<char *>(&ph), sizeof(ph)))
        {
            std::cerr << "Failed to read ELF program header " << i << std::endl;
            return false;
        }

        if (ph.type != PT_LOAD || ph.memsz == 0u)
        {
            continue;
        }

        if (ph.filesz > ph.memsz)
        {
            std::cerr << "ELF segment " << i << " has filesz > memsz." << std::endl;
            return false;
        }

        const uint64_t segmentFileEnd = static_cast<uint64_t>(ph.offset) + static_cast<uint64_t>(ph.filesz);
        if (segmentFileEnd > static_cast<uint64_t>(fileSize))
        {
            std::cerr << "ELF segment " << i << " exceeds file bounds." << std::endl;
            return false;
        }

        const bool scratch =
            ph.vaddr >= PS2_SCRATCHPAD_BASE &&
            ph.vaddr < (PS2_SCRATCHPAD_BASE + PS2_SCRATCHPAD_SIZE);

        uint32_t physAddr = 0u;
        try
        {
            physAddr = m_memory.translateAddress(ph.vaddr);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Failed to translate ELF segment " << i
                      << " virtual address 0x" << std::hex << ph.vaddr
                      << std::dec << ": " << e.what() << std::endl;
            return false;
        }
        const uint64_t regionSize = scratch ? static_cast<uint64_t>(PS2_SCRATCHPAD_SIZE)
                                            : static_cast<uint64_t>(PS2_RAM_SIZE);
        const uint64_t segmentMemEnd = static_cast<uint64_t>(physAddr) + static_cast<uint64_t>(ph.memsz);
        if (segmentMemEnd > regionSize)
        {
            std::cerr << "ELF segment " << i << " exceeds "
                      << (scratch ? "scratchpad" : "RDRAM")
                      << " bounds (vaddr=0x" << std::hex << ph.vaddr
                      << " memsz=0x" << ph.memsz << std::dec << ")." << std::endl;
            return false;
        }

        uint8_t *destBase = scratch ? m_memory.getScratchpad() : m_memory.getRDRAM();
        if (!destBase)
        {
            std::cerr << "ELF segment " << i << " has no destination memory backing." << std::endl;
            return false;
        }

        uint8_t *dest = destBase + physAddr;
        if (ph.filesz > 0u)
        {
            file.seekg(static_cast<std::streamoff>(ph.offset), std::ios::beg);
            if (!file.read(reinterpret_cast<char *>(dest), ph.filesz))
            {
                std::cerr << "Failed to read ELF segment " << i << " payload." << std::endl;
                return false;
            }
        }

        if (ph.memsz > ph.filesz)
        {
            std::memset(dest + ph.filesz, 0, ph.memsz - ph.filesz);
        }

        RUNTIME_LOG("Loading segment: 0x" << std::hex << ph.vaddr
                                          << " - 0x" << (static_cast<uint64_t>(ph.vaddr) + static_cast<uint64_t>(ph.memsz))
                                          << " (filesz: 0x" << ph.filesz
                                          << ", memsz: 0x" << ph.memsz << ")"
                                          << std::dec << std::endl);

        if (!scratch)
        {
            maxLoadedRdramEnd = std::max(maxLoadedRdramEnd, static_cast<uint32_t>(segmentMemEnd));
        }

        if (ph.flags & 0x1u) // PF_X
        {
            const uint64_t execEnd = static_cast<uint64_t>(ph.vaddr) + static_cast<uint64_t>(ph.filesz);
            if (execEnd <= std::numeric_limits<uint32_t>::max())
            {
                m_memory.registerCodeRegion(ph.vaddr, static_cast<uint32_t>(execEnd));
            }
        }

        loadedAnySegment = true;
        moduleBase = std::min(moduleBase, ph.vaddr);
        const uint64_t segmentVirtualEnd = static_cast<uint64_t>(ph.vaddr) + static_cast<uint64_t>(ph.memsz);
        const uint32_t clampedVirtualEnd =
            (segmentVirtualEnd > std::numeric_limits<uint32_t>::max())
                ? std::numeric_limits<uint32_t>::max()
                : static_cast<uint32_t>(segmentVirtualEnd);
        moduleEnd = std::max(moduleEnd, clampedVirtualEnd);
    }

    if (!loadedAnySegment)
    {
        std::cerr << "ELF contains no loadable PT_LOAD segments." << std::endl;
        return false;
    }

    if (maxLoadedRdramEnd > PS2_RAM_SIZE)
    {
        maxLoadedRdramEnd = PS2_RAM_SIZE;
    }

    const uint32_t paddedEnd = (maxLoadedRdramEnd > (PS2_RAM_SIZE - kGuestHeapSafetyPad))
                                   ? PS2_RAM_SIZE
                                   : (maxLoadedRdramEnd + kGuestHeapSafetyPad);
    const uint32_t suggestedHeapBase = alignGuestHeapValue(paddedEnd, kGuestHeapDefaultAlignment);
    {
        std::lock_guard<std::mutex> lock(m_guestHeapMutex);
        if (!m_guestHeapConfigured)
        {
            const uint32_t hardLimit = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
            m_guestHeapSuggestedBase = std::min(suggestedHeapBase, hardLimit);
            m_guestHeapBase = m_guestHeapSuggestedBase;
            m_guestHeapEnd = m_guestHeapSuggestedBase;
            m_guestHeapLimit = hardLimit;
        }
    }
    {
        std::lock_guard<std::mutex> lock(m_asyncCallbackStackMutex);
        const uint32_t hardLimit = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
        // Keep the unconfigured gap after the loaded image available for the
        // compatibility callback pool. A title may configure an empty heap at
        // the hard limit while still using that gap for static buffers;
        // reserving callbacks from the physical RAM ceiling would then
        // collide with those buffers.
        m_asyncCallbackStackFloor = std::min(suggestedHeapBase, hardLimit);
        m_asyncCallbackStackTop = hardLimit;
    }

    LoadedModule module;
    module.name = elfPath.substr(elfPath.find_last_of("/\\") + 1);
    module.baseAddress = (moduleBase == std::numeric_limits<uint32_t>::max()) ? 0x00100000u : moduleBase;
    module.size = (moduleEnd > module.baseAddress) ? static_cast<size_t>(moduleEnd - module.baseAddress) : 0u;
    module.active = true;

    m_loadedModules.push_back(module);

    uint32_t elfCrc32 = 0u;
    const bool elfCrc32Valid = computeFileCrc32(elfPath, elfCrc32);
    if (!elfCrc32Valid)
    {
        std::cerr << "[ps2xIOP] failed to compute ELF CRC32 for '" << elfPath << "'" << std::endl;
    }
    ps2x::iop::GameIdentity identity;
    identity.elfName = module.name;
    identity.entryPoint = m_cpuContext.pc;
    identity.crc32 = elfCrc32;
    std::string iopError;
    if (!m_iopSubsystem->configure(identity, &iopError))
    {
        std::cerr << "[ps2xIOP] failed to configure profile: " << iopError << std::endl;
        return false;
    }

    ps2_game_overrides::applyMatching(*this,
                                      elfPath,
                                      m_cpuContext.pc,
                                      elfCrc32,
                                      elfCrc32Valid);

    RUNTIME_LOG("ELF file loaded successfully. Entry point: 0x" << std::hex << m_cpuContext.pc << std::dec);
    return true;
}

const PS2Runtime::IoPaths &PS2Runtime::getIoPaths()
{
    return runtimeIoPaths();
}

void PS2Runtime::setIoPaths(const IoPaths &paths)
{
    IoPaths normalized = paths;
    normalized.elfPath = normalizeAbsolutePath(normalized.elfPath);
    normalized.elfDirectory = normalizeAbsolutePath(normalized.elfDirectory);
    normalized.hostRoot = normalizeAbsolutePath(normalized.hostRoot);
    normalized.cdRoot = normalizeAbsolutePath(normalized.cdRoot);
    normalized.mcRoot = normalizeAbsolutePath(normalized.mcRoot);
    normalized.cdImage = normalizeAbsolutePath(normalized.cdImage);

    if (normalized.elfDirectory.empty() && !normalized.elfPath.empty())
    {
        normalized.elfDirectory = normalized.elfPath.parent_path();
    }

    if (normalized.hostRoot.empty())
    {
        normalized.hostRoot = normalized.elfDirectory;
    }
    if (normalized.cdRoot.empty())
    {
        normalized.cdRoot = normalized.elfDirectory;
    }
    if (normalized.mcRoot.empty())
    {
        normalized.mcRoot = normalized.elfDirectory / "mc0";
    }

    runtimeIoPaths() = normalized;
}

void PS2Runtime::configureIoPathsFromElf(const std::string &elfPath)
{
    IoPaths paths = runtimeIoPaths();
    paths.elfPath = normalizeAbsolutePath(std::filesystem::path(elfPath));
    if (!paths.elfPath.empty())
    {
        paths.elfDirectory = paths.elfPath.parent_path();
    }

    if (!paths.elfDirectory.empty())
    {
        paths.hostRoot = paths.elfDirectory;
        paths.cdRoot = paths.elfDirectory;
        paths.mcRoot = paths.elfDirectory / "mc0";
    }

    setIoPaths(paths);
}

namespace
{
    bool generatedFunctionTableSlot(uint32_t address, uint32_t &slot)
    {
        if ((address & 3u) != 0u || g_ps2RecompiledFunctionTableSlotCount == 0u)
        {
            return false;
        }

        if (address < g_ps2RecompiledFunctionTableBase || address >= g_ps2RecompiledFunctionTableEnd)
        {
            return false;
        }

        const uint32_t offset = address - g_ps2RecompiledFunctionTableBase;
        slot = offset >> 2;
        return slot < g_ps2RecompiledFunctionTableSlotCount;
    }
}

bool PS2Runtime::replaceFunction(uint32_t address, RecompiledFunction func)
{
    uint32_t slot = 0u;
    if (!generatedFunctionTableSlot(address, slot))
    {
        std::cerr << "[function-table] cannot replace guest PC 0x" << std::hex << address
                  << ": outside generated dense table [0x" << g_ps2RecompiledFunctionTableBase
                  << ", 0x" << g_ps2RecompiledFunctionTableEnd << ")"
                  << std::dec << std::endl;
        return false;
    }

    g_ps2RecompiledFunctionTable[slot] = func;
    return true;
}

bool PS2Runtime::registerFunction(uint32_t address, RecompiledFunction func)
{
    return replaceFunction(address, func);
}

bool PS2Runtime::hasFunction(uint32_t address) const
{
    RecompiledFunction overlayFunction = nullptr;
    if (ps2x::resolveCodeOverlay(this, address, overlayFunction))
        return overlayFunction != nullptr;
    uint32_t slot = 0u;
    return generatedFunctionTableSlot(address, slot) && g_ps2RecompiledFunctionTable[slot] != nullptr;
}

void PS2Runtime::notifyIopVBlank()
{
    if (m_iopSubsystem)
    {
        m_iopSubsystem->onVBlank();
    }
}

// TEMP-EXPERIMENT: one-shot full RDRAM dump for HW diff. Revert.
void PS2Runtime::dumpGuestRam(const char *path)
{
    const uint8_t *ram = m_memory.getRDRAM();
    if (!ram || !path || !*path)
    {
        return;
    }
    FILE *f = std::fopen(path, "wb");
    if (!f)
    {
        return;
    }
    std::fwrite(ram, 1, PS2_RAM_SIZE, f);
    std::fclose(f);
}

// TEMP-EXPERIMENT: 1Hz guest-state trace for HW diff. Revert.
void PS2Runtime::dumpGuestStateTrace()
{
    const uint8_t *ram = m_memory.getRDRAM();
    if (!ram)
    {
        return;
    }
    auto rb = [ram](uint32_t a) -> uint32_t
    { return a < PS2_RAM_SIZE ? ram[a] : 0xFFu; };
    auto rw = [ram](uint32_t a) -> uint32_t
    {
        return a + 3u < PS2_RAM_SIZE
                   ? (uint32_t)ram[a] | ((uint32_t)ram[a + 1u] << 8) |
                         ((uint32_t)ram[a + 2u] << 16) | ((uint32_t)ram[a + 3u] << 24)
                   : 0u;
    };
    std::printf("[state] vsync=%llu flow2708=0x%x flow271d=0x%x flow270a=0x%x "
                "phase=0x%x cc=0x%x d0=0x%x cursor=0x%x q=0x%x decac=0x%x decst=0x%x\n",
                (unsigned long long)m_memory.gs().vsyncTick.load(std::memory_order_relaxed),
                rb(0x571188u),
                rb(0x57119Du),
                rb(0x57118Au),
                rw(0x5611C0u),
                rb(0x5611CCu),
                rw(0x5611D0u),
                rw(0x5611D4u),
                rw(0x987900u),
                rb(0x101D42Cu),
                rb(0x101D45Eu));
    std::fflush(stdout);
}

const char *describeGuestBranchKind(PS2Runtime::GuestBranchKind kind)
{
    switch (kind)
    {
    case PS2Runtime::GuestBranchKind::DirectJump:
        return "DirectJump";
    case PS2Runtime::GuestBranchKind::DirectCall:
        return "DirectCall";
    case PS2Runtime::GuestBranchKind::IndirectJump:
        return "IndirectJump";
    case PS2Runtime::GuestBranchKind::IndirectCall:
        return "IndirectCall";
    case PS2Runtime::GuestBranchKind::Return:
        return "Return";
    default:
        return "Unknown";
    }
}

PS2Runtime::RecompiledFunction PS2Runtime::lookupFunction(uint32_t address)
{
    pushDispatchPc(address);

    RecompiledFunction overlayFunction = nullptr;
    const bool overlayOwnsAddress = ps2x::resolveCodeOverlay(this, address, overlayFunction);
    if (overlayFunction)
        return overlayFunction;
    uint32_t slot = 0u;
    if (!overlayOwnsAddress && generatedFunctionTableSlot(address, slot))
    {
        RecompiledFunction fn = g_ps2RecompiledFunctionTable[slot];
        if (fn != nullptr)
        {
            return fn;
        }
    }

    std::cerr << "Error: No exact recompiled function for guest PC 0x" << std::hex << address
              << " tableBase=0x" << g_ps2RecompiledFunctionTableBase
              << " tableEnd=0x" << g_ps2RecompiledFunctionTableEnd
              << " codeRegion=" << (m_memory.isCodeAddress(address) ? "yes" : "no")
              << " trace=" << formatDispatchHistory()
              << std::dec << std::endl;

    static RecompiledFunction missingFunction = [](uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t badPc = ctx->pc;
        runtime->reportMissingFunction(rdram,
                                       ctx,
                                       badPc,
                                       0u,
                                       PS2Runtime::GuestBranchKind::IndirectJump,
                                       "dispatch");
    };

    return missingFunction;
}

void PS2Runtime::setMissingFunctionPolicy(MissingFunctionPolicy policy)
{
    m_missingFunctionPolicy.store(static_cast<uint32_t>(policy), std::memory_order_release);
}

PS2Runtime::MissingFunctionPolicy PS2Runtime::missingFunctionPolicy() const
{
    return static_cast<MissingFunctionPolicy>(m_missingFunctionPolicy.load(std::memory_order_acquire));
}

void PS2Runtime::resetMissingFunctionReportOnce()
{
    m_missingFunctionReported.store(false, std::memory_order_release);
    auto &reports = missingTargetReports();
    std::lock_guard<std::mutex> lock(reports.mutex);
    reports.targets.erase(this);
}

void PS2Runtime::reportMissingFunction(uint8_t *rdram,
                                       R5900Context *ctx,
                                       uint32_t targetPc,
                                       uint32_t sourcePc,
                                       GuestBranchKind kind,
                                       const char *debugName)
{
    const MissingFunctionPolicy policy = missingFunctionPolicy();
    const bool firstReport = !m_missingFunctionReported.exchange(true, std::memory_order_acq_rel);
    bool reportTarget = false;
    {
        // A null optional callback must not hide subsequent missing code.
        // Deduplicate call sites and bound diagnostic memory/output per VM.
        auto &reports = missingTargetReports();
        std::lock_guard<std::mutex> lock(reports.mutex);
        auto &targets = reports.targets[this];
        const uint64_t key = (static_cast<uint64_t>(sourcePc) << 32) | targetPc;
        if (targets.size() < 64u && std::find(targets.begin(), targets.end(), key) == targets.end())
        {
            targets.push_back(key);
            reportTarget = true;
        }
    }

    const uint32_t pc = ctx->pc;
    const uint32_t ra = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[31], 0));
    const uint32_t sp = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[29], 0));
    const uint32_t gp = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[28], 0));
    const uint32_t a0 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[4], 0));
    const uint32_t a1 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[5], 0));
    const uint32_t a2 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[6], 0));
    const uint32_t a3 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[7], 0));
    const uint32_t s0 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[16], 0));
    const uint32_t s1 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[17], 0));
    const uint32_t v0 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[2], 0));
    const uint32_t v1 = static_cast<uint32_t>(_mm_extract_epi32(ctx->r[3], 0));

    auto readGuestU32At = [rdram](uint32_t addr, uint32_t &out) -> bool
    {
        // TODO this !rdram exist only because of test fix those test later
        if (!rdram || addr > PS2_RAM_SIZE - sizeof(uint32_t))
        {
            out = 0u;
            return false;
        }

        std::memcpy(&out, rdram + addr, sizeof(uint32_t));
        return true;
    };

    auto readGuestU32Offset = [&readGuestU32At](uint32_t base, uint32_t offset, uint32_t &out) -> bool
    {
        if (base > PS2_RAM_SIZE - sizeof(uint32_t) || offset > PS2_RAM_SIZE - sizeof(uint32_t) - base)
        {
            out = 0u;
            return false;
        }

        return readGuestU32At(base + offset, out);
    };

    uint32_t a0Word0 = 0u;
    uint32_t a0Word4 = 0u;
    uint32_t a0Word8 = 0u;
    uint32_t a0WordC = 0u;
    const bool a0Readable =
        readGuestU32Offset(a0, 0x00u, a0Word0) &&
        readGuestU32Offset(a0, 0x04u, a0Word4) &&
        readGuestU32Offset(a0, 0x08u, a0Word8) &&
        readGuestU32Offset(a0, 0x0cu, a0WordC);

    uint32_t s0Word0 = 0u;
    uint32_t s0Word4 = 0u;
    uint32_t s0Word8 = 0u;
    uint32_t s0WordC = 0u;
    const bool s0Readable =
        readGuestU32Offset(s0, 0x00u, s0Word0) &&
        readGuestU32Offset(s0, 0x04u, s0Word4) &&
        readGuestU32Offset(s0, 0x08u, s0Word8) &&
        readGuestU32Offset(s0, 0x0cu, s0WordC);

    uint32_t recordWord0 = 0u;
    uint32_t recordWord4 = 0u;
    uint32_t recordWord8 = 0u;
    uint32_t recordWordC = 0u;
    const bool recordReadable =
        s0Readable && s0Word4 != 0u &&
        readGuestU32Offset(s0Word4, 0x00u, recordWord0) &&
        readGuestU32Offset(s0Word4, 0x04u, recordWord4) &&
        readGuestU32Offset(s0Word4, 0x08u, recordWord8) &&
        readGuestU32Offset(s0Word4, 0x0cu, recordWordC);

    uint32_t vtableSlot0 = 0u;
    uint32_t vtableSlot4 = 0u;
    uint32_t vtableSlot8 = 0u;
    uint32_t vtableSlotC = 0u;
    const bool vtableReadable =
        a0Readable && a0Word0 != 0u &&
        readGuestU32Offset(a0Word0, 0x00u, vtableSlot0) &&
        readGuestU32Offset(a0Word0, 0x04u, vtableSlot4) &&
        readGuestU32Offset(a0Word0, 0x08u, vtableSlot8) &&
        readGuestU32Offset(a0Word0, 0x0cu, vtableSlotC);

    if (reportTarget)
    {
        std::ostringstream oss;
        oss << "[guest-branch:missing-target] kind=" << describeGuestBranchKind(kind)
            << " op=" << (debugName ? debugName : "<unknown>")
            << " source=0x" << std::hex << sourcePc
            << " target=0x" << targetPc
            << " pc=0x" << pc
            << " ra=0x" << ra
            << " sp=0x" << sp
            << " gp=0x" << gp
            << " a0=0x" << a0
            << " a1=0x" << a1
            << " a2=0x" << a2
            << " a3=0x" << a3
            << " s0=0x" << s0
            << " s1=0x" << s1
            << " v0=0x" << v0
            << " v1=0x" << v1
            << " a0Readable=" << (a0Readable ? "yes" : "no")
            << " a0[0]=0x" << a0Word0
            << " a0[4]=0x" << a0Word4
            << " a0[8]=0x" << a0Word8
            << " a0[c]=0x" << a0WordC
            << " s0Readable=" << (s0Readable ? "yes" : "no")
            << " s0[0]=0x" << s0Word0
            << " s0[4]=0x" << s0Word4
            << " s0[8]=0x" << s0Word8
            << " s0[c]=0x" << s0WordC
            << " recordReadable=" << (recordReadable ? "yes" : "no")
            << " record[0]=0x" << recordWord0
            << " record[4]=0x" << recordWord4
            << " record[8]=0x" << recordWord8
            << " record[c]=0x" << recordWordC
            << " vtableReadable=" << (vtableReadable ? "yes" : "no")
            << " vtbl[0]=0x" << vtableSlot0
            << " vtbl[4]=0x" << vtableSlot4
            << " vtbl[8]=0x" << vtableSlot8
            << " vtbl[c]=0x" << vtableSlotC
            << " codeRegion=" << (m_memory.isCodeAddress(targetPc) ? "yes" : "no")
            << " policy=" << static_cast<uint32_t>(policy)
            << " trace=" << formatDispatchHistory()
            << std::dec;

        static std::mutex s_missingFunctionLogMutex;
        {
            std::lock_guard<std::mutex> lock(s_missingFunctionLogMutex);
            std::cerr << oss.str() << std::endl;
        }
    }

    if (firstReport && policy == MissingFunctionPolicy::BreakOnce)
    {
#if defined(_MSC_VER)
        __debugbreak();
#endif // TODO others breakpoints
    }

    if (ctx)
    {
        ctx->pc = targetPc;
    }

    if (policy == MissingFunctionPolicy::Stop)
    {
        requestStop();
    }
}

bool PS2Runtime::dispatchGuestBranch(uint8_t *rdram,
                                     R5900Context *ctx,
                                     uint32_t targetPc,
                                     uint32_t sourcePc,
                                     uint32_t fallthroughPc,
                                     GuestBranchKind kind,
                                     const char *debugName)
{
    // Unit callers may exercise branch dispatch before allocating guest RAM.
    // Keep diagnostics and the dispatch contract safe in that case; generated
    // game wrappers always pass the live RDRAM pointer.
    static std::array<uint8_t, PS2_RAM_SIZE> nullRdram{};
    if (!rdram)
        rdram = nullRdram.data();

    ctx->pc = targetPc;
    const bool isCall = (kind == GuestBranchKind::DirectCall || kind == GuestBranchKind::IndirectCall);

    // Temporary guard for accidental guest stack placement over active DMA
    // buffers.  This catches nested calls that do not pass through the EE
    // scheduler entry loop.
    {
        const uint32_t sp = getRegU32(ctx, 29);
        if (sp >= 0x1E4D080u && sp < 0x1FEE7F0u)
        {
            static uint32_t overlapStackCallProbe = 0u;
            if (overlapStackCallProbe < 64u)
            {
                std::cerr << "[probe:overlap-stack-call] target=0x" << std::hex << targetPc
                          << " source=0x" << sourcePc << " sp=0x" << sp
                          << " kind=" << debugName << std::dec << '\n';
                ++overlapStackCallProbe;
            }
        }
    }

    if (targetPc == 0x185BF8u || targetPc == 0x1865E8u)
    {
        static uint32_t statusDispatchProbeCount = 0u;
        if (statusDispatchProbeCount++ < 64u)
        {
            std::cerr << "[probe:filectrl-dispatch] target=0x" << std::hex << targetPc
                      << " source=0x" << sourcePc << " fallthrough=0x" << fallthroughPc
                      << " pc=0x" << ctx->pc << std::dec << '\n';
        }
    }

    // Throwaway boot diagnostics (removed before contribution).
    {
        switch (targetPc)
        {
        case 0x16f9b0u: // load queue enqueue
        case 0x16fa28u: // load queue processor
        case 0x1862a8u: // dvd-open
        case 0x108520u: // boot resource loader
        case 0x16f960u: // queue wait/clear entry
        case 0x16fb30u: // queue clear
        case 0x1654d0u: // boot state helper
        case 0x165560u: // boot state controller
        case 0x165430u: // boot state controller sibling
        case 0x181b10u: // per-frame call 1
        case 0x167508u: // per-frame dispatcher
        case 0x1675b8u: // boot/task dispatcher
        case 0x167800u: // boot state machine handler
        case 0x167908u: // boot state machine reset handler
        case 0x167950u: // boot state machine task registration helper
        case 0x167968u: // boot state machine task registration body
        case 0x167998u: // boot callback gate helper
        case 0x1679f8u: // boot callback lookup helper
        case 0x167bd8u: // boot completion predicate
        case 0x16ce20u: // sound-ready predicate
        case 0x17f0a8u: // subsystem dispatch wrapper
        case 0x10385e8u: // seg2 tick
        case 0x1775f0u: // boot initial load step
        case 0x181058u: // dvd read api
        case 0x13d2c0u: // common file read api
        case 0x13d4c0u: // indirect boot load step
        case 0x175a18u: // frame subsystem a
        case 0x167fd8u: // frame subsystem b
        case 0x167f88u: // frame subsystem c
        case 0x177148u: // frame subsystem d
        case 0x176fc8u: // frame subsystem e (conditional)
        case 0x168008u: // frame subsystem f
        case 0x14dbb8u: // frame subsystem g
        case 0x177bf8u: // frame subsystem h
        case 0x16b1e0u: // frame subsystem i
        case 0x16b4e0u: // frame subsystem j
        case 0x17a4a0u: // frame subsystem k
        case 0x17a0c0u: // frame subsystem l
        case 0x18d338u: // audio tick a
        case 0x18da58u: // audio tick b
        case 0x1a7978u: // audio tick c
        case 0x1a4108u: // audio tick d
        case 0x1007a0u: // game module init step
        case 0x100da0u: // task registration metadata helper
        case 0x101af0u: // boot task setup wrapper
        case 0x150368u: // module system init a
        case 0x152cf8u: // module system init b
        case 0x175b08u: // AddTask
        case 0x175bf8u: // AddTask variant
        case 0x175d78u: // deferred task starter
        case 0x154578u: // boot task registration
        case 0x100978u: // game system init block
        case 0x16a8c8u: // init step 1
        case 0x16aef8u: // init step 2
        case 0x11aea8u: // init step 3
        case 0x108af0u: // init step 4
        case 0x1164b0u: // init step 5
        case 0x11cd18u: // init step 6
        case 0x100498u: // boot state machine tick
        case 0x1018f0u: // boot sm wrapper
        case 0x102c20u: // loader completion chain head
        case 0x1031e0u: // loader completion chain
        case 0x1034c0u: // loader completion chain caller
        case 0x103538u: // gate-B writer caller
        case 0x103578u: // gate-B writer wrapper
        case 0x103640u: // gate-B writer ([0x56118A]=11)
        case 0x100eb0u: // gate-A checker
        case 0x167bccu: // setter chain head
        case 0x167be8u: // setter chain
        case 0x167c08u: // setter caller ([0x5611CC]=1, a0=2)
        case 0x19f4a0u: // blocking call inside ticker pre-open path
        case 0x185c68u: // pump transfer helper?
        case 0x185c98u: // pump transfer helper?
        case 0x180588u: // pump callee
        case 0x180708u: // pump callee
        case 0x19a958u: // dvd-open callee
        case 0x169708u: // gate callee
        case 0x14d578u: // gate callee
        case 0x16d158u: // gate callee
        case 0x177cc8u: // gate callee
        case 0x100768u: // gate callee
        case 0x16769cu: // ticker dvd-open pre-path (dead?)
        case 0x1676a4u: // ticker dvd-open resume point
        case 0x185ae0u: // pump callee
        case 0x185b48u: // pump callee
        case 0x185ba0u: // pump callee
        case 0x186548u: // pump callee
        case 0x1865b0u: // pump callee
        case 0x1865e8u: // pump decoder-pump
        case 0x187f98u: // pump decoder-setup
        case 0x180780u: // pump callee
        case 0x19a104u: // pump callee
        case 0x19f070u: // pump callee
        case 0x19f150u: // pump callee
        case 0x17eed8u: // callback tree (fn=3 issuer?)
        case 0x188f08u: // callback tree (fn=3 issuer?)
        case 0x189610u: // callback tree (fn=3 issuer?)
        case 0x188818u: // callback tree (fn=3 issuer?)
        case 0x185078u: // callback tree (fn=3 issuer?)
        case 0x19f110u: // callback tree
        case 0x181ad8u: // callback tree
        case 0x180198u: // callback tree
        case 0x167388u: // post-pump frame step
        case 0x167420u: // frame-loop step
        case 0x1987b0u: // frame-loop step
        case 0x161500u: // boot sm caller 2
        case 0x1544f8u: // post task
        case 0x154518u: // task launcher
        case 0x175928u: // TEMP flow callee
        case 0x177bc8u: // TEMP teardown callee
        case 0x1696a0u: // TEMP teardown callee
        case 0x186250u: // TEMP teardown callee
        case 0x186290u: // TEMP teardown caller
        case 0x100de0u: // TEMP flow state
        case 0x100e30u: // TEMP flow state
        case 0x101008u: // TEMP flow state
        case 0x13d900u: // TEMP queue node
        case 0x13d960u: // TEMP queue node
        case 0x13d9b8u: // TEMP queue completion
        case 0x14d400u: // TEMP producer caller
        case 0x14d4a8u: // TEMP producer caller
        case 0x14d4b8u: // TEMP producer caller
        case 0x14d4f0u: // TEMP producer caller
        case 0x14d49cu: // TEMP producer caller
        case 0x14d4acu: // TEMP producer caller
        case 0x152d50u: // TEMP producer caller
        case 0x1088f0u: // TEMP producer caller
        case 0x189f18u: // TEMP pad poller caller
        case 0x189d58u: // TEMP pad area reader
        case 0x189fc0u: // TEMP pad reader
        case 0x17f538u: // TEMP UI tick
        case 0x167450u: // TEMP UI tick caller
        case 0x1673b8u: // TEMP pump ticker
        case 0x1674c0u: // TEMP walker caller?
        case 0x17f278u: // TEMP task slot installer
        case 0x17f000u: // TEMP task installer
        case 0x18f0e0u: // TEMP installed task handler
        {
            auto &count = m_bootWatchCounts[targetPc];
            ++count;
            if (count <= 3u)
            {
                std::cerr << "[probe:call] target=0x" << std::hex << targetPc
                          << " from=0x" << sourcePc << " kind=" << debugName << std::dec
                          << " n=" << count << std::endl;
            }
            break;
        }
        default:
            if (kind != GuestBranchKind::DirectCall && kind != GuestBranchKind::DirectJump)
            {
                auto &count = m_bootWatchCounts[targetPc | 0x80000000u];
                ++count;
                auto &total = m_bootWatchCounts[0xDEADBEEFu];
                if (count <= 2u && total < 120u)
                {
                    ++total;
                    std::cerr << "[probe:indirect] target=0x" << std::hex << targetPc
                              << " from=0x" << sourcePc << " kind=" << debugName << std::dec
                              << " n=" << count << std::endl;
                }
            }
            if (targetPc >= 0x165000u && targetPc < 0x167000u)
            {
                auto &count = m_bootWatchCounts[targetPc];
                ++count;
                if (count <= 3u)
                {
                    std::cerr << "[probe:state] target=0x" << std::hex << targetPc
                              << " from=0x" << sourcePc << " kind=" << debugName << std::dec
                              << " n=" << count << std::endl;
                }
            }
            break;
        }
    }

    // Every inter-function transfer is also a deterministic EE safe point.
    // Backward edges inside generated functions use eeCheckpointDue(), while
    // this charge bounds straight-line call chains that have no local loop.
    // A call must run to its callee so its return PC/stack continuation stays
    // coherent.  The callee and the generated caller already expose regular
    // safe points; interrupting here leaves the guest suspended at the call
    // boundary and can repeatedly replay initialization side effects.  Keep
    // the dispatch charge for non-call transfers, where there is no return
    // continuation to preserve.
    if (!isCall && m_eeScheduler && m_eeScheduler->checkpointDue(EeScheduler::kGuestDispatchCycles))
    {
        return false;
    }

    if (!isCall)
    {
        if (!hasFunction(targetPc))
        {
            reportMissingFunction(rdram, ctx, targetPc, sourcePc, kind, debugName);
        }

        ctx->pc = targetPc;
        return false;
    }

    if (!hasFunction(targetPc))
    {
        // A null indirect call is a legal state for optional callbacks in
        // retail middleware.  Treat it as an uninstalled callback and resume
        // after the delay slot instead of transferring the EE thread to
        // address zero, which otherwise strands the scheduler permanently.
        if (targetPc == 0u && kind == GuestBranchKind::IndirectCall)
        {
            reportMissingFunction(rdram, ctx, targetPc, sourcePc, kind, debugName);
            ctx->pc = fallthroughPc;
            return true;
        }

        reportMissingFunction(rdram, ctx, targetPc, sourcePc, kind, debugName);

        const MissingFunctionPolicy policy = missingFunctionPolicy();

        if (policy == MissingFunctionPolicy::SkipCallDebug && isCall)
        {
            ctx->pc = fallthroughPc;
            return true;
        }

        if (policy == MissingFunctionPolicy::ContinueToTarget)
        {
            ctx->pc = targetPc;
            return true;
        }

        return false;
    }

    RecompiledFunction targetFn = lookupFunction(targetPc);
    const uint32_t entryPc = ctx->pc;
    if (targetPc == 0x1A96D8u || targetPc == 0x1A96A0u)
    {
        static uint32_t callbackStateProbeCount = 0u;
        if (callbackStateProbeCount < 24u)
        {
            std::cerr << "[probe:callback-state] target=0x" << std::hex << targetPc
                      << " source=0x" << sourcePc
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " a1=0x" << getRegU32(ctx, 5)
                      << " a2=0x" << getRegU32(ctx, 6)
                      << " a3=0x" << getRegU32(ctx, 7)
                      << " global49c460=0x" << Ps2FastRead32(rdram, 0x49C460u)
                      << " global49c448=0x" << Ps2FastRead32(rdram, 0x49C448u)
                      << " global4a9e4c=0x" << Ps2FastRead32(rdram, 0x4A9E4Cu)
                      << std::dec << '\n';
            ++callbackStateProbeCount;
        }
    }
    if (targetPc == 0x175A18u || targetPc == 0x175B08u || targetPc == 0x175BF8u ||
        targetPc == 0x175D78u || targetPc == 0x154578u || targetPc == 0x154518u)
    {
        static uint32_t taskApiProbeCount = 0u;
        if (taskApiProbeCount < 96u)
        {
            std::cerr << "[probe:task-api] target=0x" << std::hex << targetPc
                      << " source=0x" << sourcePc
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " a1=0x" << getRegU32(ctx, 5)
                      << " a2=0x" << getRegU32(ctx, 6)
                      << " a3=0x" << getRegU32(ctx, 7)
                      << " gate0=0x" << static_cast<unsigned>(rdram[0xF6D278u])
                      << " gate1=0x" << static_cast<unsigned>(rdram[0xF6D279u])
                      << " head=0x" << Ps2FastRead32(rdram, 0xF6D280u)
                      << " slot0=";
            for (uint32_t i = 0; i < 0x20u; i += 4u)
                std::cerr << (i ? "," : "") << "0x" << Ps2FastRead32(rdram, 0xF6D280u + i);
            std::cerr << std::dec << '\n';
            ++taskApiProbeCount;
        }
    }
    // Snapshot the generic boot/task object's callback gate before invoking a
    // guest function.  Keep declarations above the diagnostic gotos below.
    const uint32_t bootObjectProbe = Ps2FastRead32(rdram, 0x101BDD4u) & PS2_RAM_MASK;
    const uint32_t bootStateCBefore = bootObjectProbe < PS2_RAM_SIZE
                                          ? Ps2FastRead32(rdram, bootObjectProbe + 0x0Cu)
                                          : 0u;
    // Observe indirect task dispatch targets without touching generated code.
    // These are the two generic task-list jump sites (17f07c/17f0c4); seeing
    // the actual function pointer is more useful than assuming the boot task
    // remains at 167800 after its state callback changes it.
    if (sourcePc == 0x17F07Cu || sourcePc == 0x17F0C4u)
    {
        static uint32_t taskDispatchProbeCount = 0u;
        if (taskDispatchProbeCount < 1024u)
        {
            const uint32_t state = Ps2FastRead32(rdram, 0x101BDD0u) & PS2_RAM_MASK;
            std::cerr << "[probe:task-dispatch] source=0x" << std::hex << sourcePc
                      << " target=0x" << targetPc
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " ra=0x" << getRegU32(ctx, 31)
                      << " state=0x" << state
                      << " state0=0x" << Ps2FastRead32(rdram, state + 0x00u)
                      << " state4=0x" << Ps2FastRead32(rdram, state + 0x04u)
                      << " state8=0x" << Ps2FastRead32(rdram, state + 0x08u)
                      << " stateC=0x" << static_cast<unsigned>(rdram[(state + 0x0Cu) & PS2_RAM_MASK])
                      << " state14=0x" << Ps2FastRead32(rdram, state + 0x14u)
                      << " state10=0x" << Ps2FastRead32(rdram, state + 0x10u)
                << " taskHead=0x" << Ps2FastRead32(rdram, 0x101BDD4u)
                << " d0=0x" << Ps2FastRead32(rdram, 0x5611D0u)
                << " busy=0x" << static_cast<unsigned>(rdram[0x101DB77u])
                << " idx=0x" << static_cast<unsigned>(rdram[0x101BDDFu])
                << " base=0x" << Ps2FastRead32(rdram, 0x101BDD8u)
                << " slot80=0x" << Ps2FastRead32(rdram, (state + 0x80u) & PS2_RAM_MASK)
                << " slot100=0x" << Ps2FastRead32(rdram, (state + 0x100u) & PS2_RAM_MASK)
                << " cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                << " cursor=0x" << Ps2FastRead32(rdram, 0x5611D4u)
                << " entry=0x" << Ps2FastRead32(rdram, (Ps2FastRead32(rdram, 0x5611D4u) + 8u) & PS2_RAM_MASK)
                << " entry3=0x" << Ps2FastRead32(rdram, (Ps2FastRead32(rdram, 0x5611D4u) + 20u) & PS2_RAM_MASK)
                << " phase=0x" << Ps2FastRead32(rdram, 0x5611C0u)
                << " q31=0x" << static_cast<unsigned>(rdram[0x987831u])
                << " ec0=0x" << Ps2FastRead32(rdram, 0x4A9EC0u)
                << " db10=0x" << Ps2FastRead32(rdram, 0x54DB10u)
                << " db14=0x" << Ps2FastRead32(rdram, 0x54DB14u)
                      << std::dec << '\n';
            ++taskDispatchProbeCount;
        }
    }
    if (sourcePc == 0x167688u || sourcePc == 0x1676F8u || sourcePc == 0x16773Cu)
    {
        static uint32_t bootCallbackProbeCount = 0u;
        if (bootCallbackProbeCount < 96u)
        {
            const uint32_t a0 = getRegU32(ctx, 4);
            std::cerr << "[probe:boot-callback] source=0x" << std::hex << sourcePc
                      << " target=0x" << targetPc
                      << " a0=0x" << a0
                      << " word0=0x" << Ps2FastRead32(rdram, a0)
                      << " word4=0x" << Ps2FastRead32(rdram, a0 + 4u)
                      << " word8=0x" << Ps2FastRead32(rdram, a0 + 8u)
                      << " g5611d4=0x" << Ps2FastRead32(rdram, 0x5611D4u)
                      << " g5611dc=0x" << Ps2FastRead32(rdram, 0x5611DCu)
                      << " g5611cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                      << std::dec << '\n';
            ++bootCallbackProbeCount;
        }
    }
    if (sourcePc >= 0x167600u && sourcePc < 0x167800u)
    {
        static uint32_t bootBodyDispatchProbeCount = 0u;
        if (bootBodyDispatchProbeCount < 160u)
        {
            std::cerr << "[probe:boot-body-dispatch] source=0x" << std::hex << sourcePc
                      << " target=0x" << targetPc
                      << " kind=" << debugName
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " a1=0x" << getRegU32(ctx, 5)
                      << " a2=0x" << getRegU32(ctx, 6)
                      << " a3=0x" << getRegU32(ctx, 7)
                      << " g5611c0=0x" << Ps2FastRead32(rdram, 0x5611C0u)
                      << " g5611cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                      << " g5611d0=0x" << Ps2FastRead32(rdram, 0x5611D0u)
                      << std::dec << '\n';
            ++bootBodyDispatchProbeCount;
        }
    }
    {
        // Narrow nested-call watch for the remaining stream mutations.  The
        // scheduler probe attributes changes to the enclosing callback; this
        // boundary identifies the exact dispatched callee without instrumenting
        // generated headers or rebuilding the translation output.
        const bool streamWatchTarget = targetPc == 0x19F250u || targetPc == 0x19F610u ||
                                       targetPc == 0x1A2818u || targetPc == 0x1A8828u ||
                                       targetPc == 0x1A8878u;
        if (streamWatchTarget)
        {
            static bool streamWatchArmed = false;
            static uint64_t streamWatchHash = 0u;
            const uint32_t streamBase = 0x1F0D080u;
            if (!streamWatchArmed && m_memory.getRDRAM()[0x1FCD080u] == 0x44u &&
                m_memory.getRDRAM()[0x1FCD081u] == 0x3Du && m_memory.getRDRAM()[0x1FCD082u] == 0x40u &&
                m_memory.getRDRAM()[0x1FCD083u] == 0x01u)
            {
                streamWatchArmed = true;
                streamWatchHash = 1469598103934665603ull;
                for (uint32_t i = 0u; i < 0x20000u; ++i)
                {
                    streamWatchHash ^= m_memory.getRDRAM()[streamBase + i];
                    streamWatchHash *= 1099511628211ull;
                }
            }
            if (streamWatchArmed)
            {
                targetFn(rdram, ctx, this);
                // The stream watch is diagnostic; hashing 128 KiB after every
                // nested call can cost more than the guest work itself. Keep
                // the call transparent and sample the payload sparsely.
                static uint32_t streamWatchSampleCounter = 0u;
                if ((streamWatchSampleCounter++ & 0xFFu) != 0u)
                {
                    goto dispatchGuestBranchAfterTarget;
                }
                uint64_t currentHash = 1469598103934665603ull;
                for (uint32_t i = 0u; i < 0x20000u; ++i)
                {
                    currentHash ^= m_memory.getRDRAM()[streamBase + i];
                    currentHash *= 1099511628211ull;
                }
                if (currentHash != streamWatchHash)
                {
                    std::cerr << "[probe:stream-nested-change] target=0x" << std::hex << targetPc
                              << " source=0x" << sourcePc << " old=0x" << streamWatchHash
                              << " new=0x" << currentHash << std::dec << '\n';
                    streamWatchHash = currentHash;
                }
                goto dispatchGuestBranchAfterTarget;
            }
        }
    }
#if PS2_TRACE_CALL_WATCH
    if (targetPc == 0x180540u || targetPc == 0x167D70u || targetPc == 0x1007E8u ||
        targetPc == 0x1665F0u || targetPc == 0x166890u || targetPc == 0x180588u ||
        targetPc == 0x1A0740u || targetPc == 0x185BA0u || targetPc == 0x185A30u ||
        targetPc == 0x1A2F30u || targetPc == 0x1A2D50u || targetPc == 0x1A2770u || targetPc == 0x1A3130u ||
        targetPc == 0x18D020u || targetPc == 0x187F10u || targetPc == 0x187F98u ||
        targetPc == 0x1675B8u || targetPc == 0x167800u || targetPc == 0x167908u ||
        targetPc == 0x16CE20u || targetPc == 0x17F0A8u ||
        targetPc == 0x188010u || targetPc == 0x19A958u || targetPc == 0x1A2818u ||
        targetPc == 0x186708u || targetPc == 0x1A8828u ||
        targetPc == 0x185E60u || targetPc == 0x185DE0u || targetPc == 0x185F80u ||
        targetPc == 0x180708u || targetPc == 0x181058u || targetPc == 0x186438u ||
        targetPc == 0x1862A8u || targetPc == 0x185C68u || targetPc == 0x180540u ||
        targetPc == 0x1808E0u || targetPc == 0x180F68u ||
        targetPc == 0x167998u || targetPc == 0x1679F8u || targetPc == 0x1679ECu ||
        targetPc == 0x167968u || targetPc == 0x100DA0u || targetPc == 0x100978u ||
        targetPc == 0x167BD8u ||
        targetPc == 0x175A18u || targetPc == 0x175D78u || targetPc == 0x1544F8u ||
        targetPc == 0x154518u || targetPc == 0x154578u ||
        targetPc == 0x149F48u || targetPc == 0x101AF0u || targetPc == 0x167A20u ||
        targetPc == 0x167A14u)
    {
        std::cerr << "[probe:call-watch] target=0x" << std::hex << targetPc
                  << " before source=0x" << sourcePc
                  << " fallthrough=0x" << fallthroughPc << " ra=0x" << getRegU32(ctx, 31)
                  << " sp=0x" << getRegU32(ctx, 29)
                  << " a0=0x" << getRegU32(ctx, 4)
                  << " a1=0x" << getRegU32(ctx, 5)
                  << " a2=0x" << getRegU32(ctx, 6)
                  << " a3=0x" << getRegU32(ctx, 7)
                  << " s0=0x" << getRegU32(ctx, 16)
                  << " s0c=0x" << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 16) + 0xCu)
                  << " a1c=0x" << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 5) + 0xCu)
                  << " gpstate=0x" << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 28) - 0x7F28u)
                  << " dvdBuf=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D418u)
                  << " streamGlobal=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x4A9E4Cu)
                  << " slot=0x" << Ps2FastRead64(m_memory.getRDRAM(), getRegU32(ctx, 29))
                  << ((targetPc == 0x1862A8u)
                          ? (std::string(" desc78=0x") + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead64(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x78u); return s.str(); }() +
                             " desc7c=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x7Cu); return s.str(); }() +
                             " desc84=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x84u); return s.str(); }() +
                             " desc98=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x98u); return s.str(); }())
                          : std::string())
                  << std::dec << std::endl;
    }
#endif
    if (targetPc == 0x186708u || targetPc == 0x187F98u || targetPc == 0x188010u ||
        targetPc == 0x1A8828u || targetPc == 0x1A8878u)
    {
        std::cerr << "[probe:decoder-call] target=0x" << std::hex << targetPc
                  << " before pc=0x" << ctx->pc << " ra=0x" << getRegU32(ctx, 31)
                  << " a0=0x" << getRegU32(ctx, 4) << " a1=0x" << getRegU32(ctx, 5)
                  << " a2=0x" << getRegU32(ctx, 6) << " a3=0x" << getRegU32(ctx, 7)
                  << " qwords=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D450u)
                  << " gpstate=0x" << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 28) - 0x7F28u)
                  << " ctl10=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D390u)
                  << " ctl84=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D404u)
                  << " ctl88=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D408u)
                  << " ctl90=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D410u)
                  << " ctlA8=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D428u])
                  << " ctlA9=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D429u])
                  << " ctlAF=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Fu])
                  << " state84=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D404u)
                  << " state88=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D408u)
                  << " state8c=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D40Cu)
                  << " state94=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D414u)
                  << " state98=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D418u)
                  << " statea0=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D420u)
                  << " statea4=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D424u)
                  << " stateac=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Cu])
                  << " statead=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Du])
                  << " stateaf=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Fu])
                  << " stateb0=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D430u)
                  << " global1e=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D45Eu])
                  << " report10=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D510u)
                  << " report24=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D524u)
                  << " report28=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D528u)
                  << ((targetPc == 0x186708u)
                          ? (std::string(" desc78=0x") + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead64(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x78u); return s.str(); }() +
                             " desc84=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x84u); return s.str(); }() +
                             " desc88=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x88u); return s.str(); }() +
                             " desc90=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x90u); return s.str(); }() +
                             " desc98=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x98u); return s.str(); }() +
                             " descA0=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0xA0u); return s.str(); }() +
                             " descA4=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0xA4u); return s.str(); }() +
                             " descA8=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[getRegU32(ctx, 4) + 0xA8u]); return s.str(); }() +
                             " descA9=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[getRegU32(ctx, 4) + 0xA9u]); return s.str(); }() +
                             " descAA=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead16(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0xAAu); return s.str(); }() +
                             " descAC=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[getRegU32(ctx, 4) + 0xACu]); return s.str(); }() +
                             " descAD=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[getRegU32(ctx, 4) + 0xADu]); return s.str(); }() +
                             " descAF=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[getRegU32(ctx, 4) + 0xAFu]); return s.str(); }() +
                             " descB0=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0xB0u); return s.str(); }() +
                             " global10=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), 0x101D390u); return s.str(); }() +
                             " globalAC=0x" + [&]() { std::ostringstream s; s << std::hex << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Cu]); return s.str(); }())
                          : std::string())
                  << ((targetPc == 0x188010u)
                          ? (std::string(" decState10=0x") + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), 0x101D450u); return s.str(); }() +
                             " decState14=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), 0x101D454u); return s.str(); }() +
                             " decState18=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), 0x101D458u); return s.str(); }() +
                             " decState1c=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead16(m_memory.getRDRAM(), 0x101D45Cu); return s.str(); }())
                          : std::string())
                  << std::dec << '\n';
    }
    if (targetPc == 0x1A2818u)
    {
        static uint32_t sifStateCallProbe = 0u;
        if (sifStateCallProbe < 64u)
        {
            const uint32_t a0 = getRegU32(ctx, 4);
            std::cerr << "[probe:sif-state-call] a0=0x" << std::hex << a0
                      << " ra=0x" << getRegU32(ctx, 31)
                      << " source=0x" << sourcePc;
            if (a0 >= 0x1E4D080u && a0 < 0x1FEE7F0u)
            {
                std::cerr << " OVERLAPS_STREAM";
            }
            std::cerr << std::dec << '\n';
            ++sifStateCallProbe;
        }
    }
    // Temporary async-I/O task probe. These entries are reached through the
    // guest task dispatcher rather than the decoder's direct call chain; keep
    // this outside PS2_TRACE_CALL_WATCH so the scheduler path is observable in
    // the normal diagnostic build.
    if (targetPc == 0x185E60u || targetPc == 0x185DE0u || targetPc == 0x185F80u)
    {
        static uint32_t asyncTaskProbeCount = 0u;
        if (asyncTaskProbeCount < 96u)
        {
            std::cerr << "[probe:async-task] target=0x" << std::hex << targetPc
                      << " source=0x" << sourcePc << " fallthrough=0x" << fallthroughPc
                      << " ra=0x" << getRegU32(ctx, 31)
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " a1=0x" << getRegU32(ctx, 5)
                      << " a2=0x" << getRegU32(ctx, 6)
                      << " ac=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Cu])
                      << " state10=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D390u)
                      << " state14=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D394u)
                      << " state15=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D395u])
                      << " state84=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D404u)
                      << " state90=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D410u)
                      << std::dec << '\n';
            ++asyncTaskProbeCount;
        }
    }
    if (targetPc == 0x1675F8u || targetPc == 0x167620u || targetPc == 0x167800u || targetPc == 0x167908u ||
        targetPc == 0x167A20u || targetPc == 0x167968u || targetPc == 0x16CE20u ||
        targetPc == 0x1038140u || targetPc == 0x10381A0u)
    {
        static uint32_t bootTaskProbeCount = 0u;
        if (bootTaskProbeCount < 48u)
        {
            const uint32_t state = Ps2FastRead32(rdram, 0x101BDD0u) & PS2_RAM_MASK;
            std::cerr << "[probe:boot-task] target=0x" << std::hex << targetPc
                      << " state=0x" << state
                      << " state0=0x" << Ps2FastRead32(rdram, state + 0x00u)
                      << " state4=0x" << Ps2FastRead32(rdram, state + 0x04u)
                      << " state8=0x" << Ps2FastRead32(rdram, state + 0x08u)
                      << " stateC=0x" << static_cast<unsigned>(rdram[(state + 0x0Cu) & PS2_RAM_MASK])
                      << " state14=0x" << Ps2FastRead32(rdram, state + 0x14u)
                      << " state10=0x" << Ps2FastRead32(rdram, state + 0x10u)
                      << " busy=0x" << static_cast<unsigned>(rdram[0x101DB77u])
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " ra=0x" << getRegU32(ctx, 31) << std::dec << '\n';
            ++bootTaskProbeCount;
        }
    }
    // Attribute task-object mutations to the dispatched guest function without
    // instrumenting generated output.  The boot object is initialized once at
    // 0x54dac0; its +0x0c callback/state gate must become non-zero before the
    // generic task dispatcher can invoke registered work.  Sampling at call
    // boundaries keeps this diagnostic cheap and applies to every title.
    targetFn(rdram, ctx, this);
    if (targetPc == 0x175A18u)
    {
        static uint32_t taskWalkerAfterProbeCount = 0u;
        if (taskWalkerAfterProbeCount < 96u)
        {
            const uint32_t taskBase = 0xF6D280u;
            std::cerr << "[probe:task-walker-after] v0=0x" << std::hex << getRegU32(ctx, 2)
                      << " pc=0x" << ctx->pc
                      << " gate0=0x" << static_cast<unsigned>(rdram[0xF6D278u])
                      << " gate1=0x" << static_cast<unsigned>(rdram[0xF6D279u])
                      << " head=0x" << Ps2FastRead32(rdram, taskBase)
                      << " next=0x" << Ps2FastRead32(rdram, taskBase + 4u)
                      << " cb10=0x" << Ps2FastRead32(rdram, taskBase + 0x10u)
                      << " cb14=0x" << Ps2FastRead32(rdram, taskBase + 0x14u)
                      << " next20=0x" << Ps2FastRead32(rdram, taskBase + 0x20u)
                      << std::dec << '\n';
            ++taskWalkerAfterProbeCount;
        }
    }
    if (targetPc == 0x1675F8u || targetPc == 0x167620u || targetPc == 0x167800u)
    {
        static uint32_t taskChainProbeCount = 0u;
        if (taskChainProbeCount < 48u)
        {
            const uint32_t head = Ps2FastRead32(rdram, 0x101BDD0u) & PS2_RAM_MASK;
            const uint32_t cb = head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x00u) : 0u;
            const uint32_t next = head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x04u) : 0u;
            std::cerr << "[probe:task-chain] target=0x" << std::hex << targetPc
                      << " head=0x" << head
                      << " cb=0x" << cb << " next=0x" << next
                      << " arg=0x" << (head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x08u) : 0u)
                      << " flags=0x" << (head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x0Cu) : 0u)
                      << " state10=0x" << (head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x10u) : 0u)
                      << " state14=0x" << (head < PS2_RAM_SIZE ? Ps2FastRead32(rdram, head + 0x14u) : 0u)
                      << " pc=0x" << ctx->pc << std::dec << '\n';
            ++taskChainProbeCount;
        }
    }
    if (targetPc == 0x167908u)
    {
        static uint32_t splitContinuationProbeCount = 0u;
        if (splitContinuationProbeCount < 8u)
        {
            std::cerr << "[probe:split-continuation] after=0x" << std::hex << ctx->pc
                        << " fallthrough=0x" << fallthroughPc
                        << " has=0x" << (hasFunction(ctx->pc) ? 1u : 0u)
                        << " kind=" << debugName << std::dec << '\n';
            ++splitContinuationProbeCount;
        }
    }
    // THROWAWAY-DIAGNOSTIC: dedicated counters for the boot-gate path
    // (shared cap would hide these rare calls behind the frame ticker).
    if (targetPc == 0x100EB0u || targetPc == 0x103640u || targetPc == 0x103578u ||
        targetPc == 0x167968u || targetPc == 0x167BE8u || targetPc == 0x167C08u ||
        targetPc == 0x102C20u || targetPc == 0x1031E0u || targetPc == 0x103538u ||
        targetPc == 0x1A2F30u || targetPc == 0x1A2D50u)
    {
        static uint32_t bootGatePathProbeCount = 0u;
        if (bootGatePathProbeCount < 500u)
        {
            std::cerr << "[probe:boot-gate-path] target=0x" << std::hex << targetPc
                      << " a0=0x" << getRegU32(ctx, 4)
                      << " a1=0x" << getRegU32(ctx, 5)
                      << " a2=0x" << getRegU32(ctx, 6)
                      << " a3=0x" << getRegU32(ctx, 7)
                      << " ra=0x" << getRegU32(ctx, 31)
                      << " g561188=0x" << static_cast<unsigned>(rdram[0x561188u])
                      << " g56118a=0x" << static_cast<unsigned>(rdram[0x56118Au])
                      << " g56119d=0x" << static_cast<unsigned>(rdram[0x56119Du])
                      << " cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                      << " b736b08=0x" << static_cast<unsigned>(rdram[0x736B08u])
                      << std::dec << '\n';
            ++bootGatePathProbeCount;
        }
    }
    if (bootObjectProbe < PS2_RAM_SIZE)
    {
        const uint32_t bootStateCAfter = Ps2FastRead32(rdram, bootObjectProbe + 0x0Cu);
        if (bootStateCAfter != bootStateCBefore)
        {
            static uint32_t bootStateMutationProbeCount = 0u;
            if (bootStateMutationProbeCount < 96u)
            {
                std::cerr << "[probe:boot-state-mutation] target=0x" << std::hex << targetPc
                          << " source=0x" << sourcePc
                          << " object=0x" << bootObjectProbe
                          << " before=0x" << bootStateCBefore
                          << " after=0x" << bootStateCAfter
                          << " pc=0x" << ctx->pc
                          << " ra=0x" << getRegU32(ctx, 31)
                          << std::dec << '\n';
                ++bootStateMutationProbeCount;
            }
        }
    }

    // Some stripped/retail images expose an address-taken entry in the
    // middle of a linear routine.  The generated slice can legitimately end
    // at the next registered block without a JR $ra; on hardware execution
    // simply falls through into that block.  This can happen for direct calls
    // as well as indirect calls when an address-taken entry split the source
    // routine.  Continue only when the next PC resolves to a *different*
    // registered function; a normal return lands exactly at fallthroughPc and
    // is therefore left to the caller's generated continuation.
    for (unsigned continuationDepth = 0u;
         (kind == GuestBranchKind::IndirectCall || kind == GuestBranchKind::DirectCall) &&
             continuationDepth < 16u;
         ++continuationDepth)
    {
        const uint32_t continuationPc = ctx->pc;
        if (continuationPc == fallthroughPc || continuationPc <= targetPc || !hasFunction(continuationPc))
        {
            break;
        }

        const RecompiledFunction continuationFn = lookupFunction(continuationPc);
        if (targetPc == 0x167908u)
        {
            static uint32_t splitContinuationInvokeProbeCount = 0u;
            if (splitContinuationInvokeProbeCount < 8u)
            {
                std::cerr << "[probe:split-continuation-invoke] pc=0x" << std::hex
                          << continuationPc << " same=0x" << (continuationFn == targetFn ? 1u : 0u)
                          << std::dec << '\n';
                ++splitContinuationInvokeProbeCount;
            }
        }
        if (continuationFn == targetFn)
        {
            break;
        }

        continuationFn(rdram, ctx, this);
        if (targetPc == 0x167908u)
        {
            static uint32_t splitContinuationResultProbeCount = 0u;
            if (splitContinuationResultProbeCount < 8u)
            {
                std::cerr << "[probe:split-continuation-result] pc=0x" << std::hex
                          << ctx->pc << " g5611cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                          << std::dec << '\n';
                ++splitContinuationResultProbeCount;
            }
        }
    }
dispatchGuestBranchAfterTarget:
    if (targetPc == 0x1675F8u || targetPc == 0x167620u || targetPc == 0x167800u || targetPc == 0x167908u ||
        targetPc == 0x167A20u || targetPc == 0x167968u || targetPc == 0x16CE20u ||
        targetPc == 0x1038140u || targetPc == 0x10381A0u ||
        targetPc == 0x100EB0u || targetPc == 0x103640u || targetPc == 0x103578u ||
        targetPc == 0x167BD8u || targetPc == 0x167D68u)
    {
        static uint32_t bootTaskAfterProbeCount = 0u;
        if (bootTaskAfterProbeCount < 48u)
        {
            const uint32_t state = Ps2FastRead32(rdram, 0x101BDD0u) & PS2_RAM_MASK;
            std::cerr << "[probe:boot-task-after] target=0x" << std::hex << targetPc
                      << " state=0x" << state
                      << " state14=0x" << Ps2FastRead32(rdram, state + 0x14u)
                      << " state10=0x" << Ps2FastRead32(rdram, state + 0x10u)
                      << " busy=0x" << static_cast<unsigned>(rdram[0x101DB77u])
                      << " v0=0x" << getRegU32(ctx, 2)
                      << " g5611c0=0x" << Ps2FastRead32(rdram, 0x5611C0u)
                      << " g5611cc=0x" << static_cast<unsigned>(rdram[0x5611CCu])
                      << " g5611d0=0x" << Ps2FastRead32(rdram, 0x5611D0u)
                      << " g5611d4=0x" << Ps2FastRead32(rdram, 0x5611D4u)
                      << " g5611e0=0x" << Ps2FastRead32(rdram, 0x5611E0u)
                      << " g5611e4=0x" << Ps2FastRead32(rdram, 0x5611E4u)
                      << " g5611e8=0x" << Ps2FastRead32(rdram, 0x5611E8u)
                      << " g5611ec=0x" << Ps2FastRead32(rdram, 0x5611ECu)
                      << " g5611dc=0x" << Ps2FastRead32(rdram, 0x5611DCu)
                      << " tbl27b380=0x" << Ps2FastRead32(rdram, 0x27B380u)
                      << " tbl27b390=0x" << Ps2FastRead32(rdram, 0x27B390u)
                      << " tbl27b398=0x" << Ps2FastRead32(rdram, 0x27B398u)
                      << " tbl27b39c=0x" << Ps2FastRead32(rdram, 0x27B39Cu)
                      << " tbl27b3a0=0x" << Ps2FastRead32(rdram, 0x27B3A0u)
                      << " bootFlag=0x" << static_cast<unsigned>(rdram[0x101D42Du])
                      << " sysBasePtr=0x" << Ps2FastRead32(rdram, 0x4A9E58u)
                      << " sys14dyn=0x" << static_cast<unsigned>(rdram[(Ps2FastRead32(rdram, 0x4A9E58u) + 0x14u) & PS2_RAM_MASK])
                      << " sys10dyn=0x" << Ps2FastRead32(rdram, (Ps2FastRead32(rdram, 0x4A9E58u) + 0x10u) & PS2_RAM_MASK)
                      << " sys18dyn=0x" << Ps2FastRead32(rdram, (Ps2FastRead32(rdram, 0x4A9E58u) + 0x18u) & PS2_RAM_MASK)
                      << " decAC=0x" << static_cast<unsigned>(rdram[0x101D42Cu])
                      << " decAD=0x" << static_cast<unsigned>(rdram[0x101D42Du])
                      << " decST=0x" << static_cast<unsigned>(rdram[0x101D45Eu])
                      << " r0=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(ctx->r[0], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(ctx->r[0], 0))
                      << " decRemain=0x" << Ps2FastRead32(rdram, 0x101D448u)
                      << " decOut=0x" << Ps2FastRead32(rdram, 0x101D44Cu)
                      << " decSrc=0x" << Ps2FastRead32(rdram, 0x101D450u)
                      << " decEnd=0x" << Ps2FastRead32(rdram, 0x101D454u)
                      << " g561188=0x" << static_cast<unsigned>(rdram[0x561188u])
                      << " g56118a=0x" << static_cast<unsigned>(rdram[0x56118Au])
                      << " g56119d=0x" << static_cast<unsigned>(rdram[0x56119Du])
                      << " b736b08=0x" << static_cast<unsigned>(rdram[0x736B08u])
                      << std::dec << '\n';
            ++bootTaskAfterProbeCount;
        }
    }
#if PS2_TRACE_CALL_WATCH
    if (targetPc == 0x180540u || targetPc == 0x167D70u || targetPc == 0x1007E8u ||
        targetPc == 0x1665F0u || targetPc == 0x166890u || targetPc == 0x180588u ||
        targetPc == 0x1A0740u || targetPc == 0x185BA0u || targetPc == 0x185A30u ||
        targetPc == 0x1A2F30u || targetPc == 0x1A2D50u || targetPc == 0x1A2770u || targetPc == 0x1A3130u ||
        targetPc == 0x18D020u || targetPc == 0x187F10u || targetPc == 0x187F98u ||
        targetPc == 0x1675B8u || targetPc == 0x167800u || targetPc == 0x167908u ||
        targetPc == 0x16CE20u || targetPc == 0x17F0A8u ||
        targetPc == 0x188010u || targetPc == 0x19A958u || targetPc == 0x1A2818u ||
        targetPc == 0x186708u || targetPc == 0x1A8828u ||
        targetPc == 0x185E60u || targetPc == 0x185DE0u || targetPc == 0x185F80u ||
        targetPc == 0x180708u || targetPc == 0x181058u || targetPc == 0x186438u ||
        targetPc == 0x1862A8u || targetPc == 0x185C68u || targetPc == 0x180540u ||
        targetPc == 0x1808E0u || targetPc == 0x180F68u)
    {
        std::cerr << "[probe:call-watch] target=0x" << std::hex << targetPc
                  << " after pc=0x" << ctx->pc
                  << " ra=0x" << getRegU32(ctx, 31) << " sp=0x" << getRegU32(ctx, 29)
                  << " v0=0x" << getRegU32(ctx, 2)
                  << " gpstate=0x" << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 28) - 0x7F28u)
                  << " dvdBuf=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D418u)
                  << " streamGlobal=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x4A9E4Cu)
                  << " slot=0x" << Ps2FastRead64(m_memory.getRDRAM(), getRegU32(ctx, 29))
                  << ((targetPc == 0x1862A8u)
                          ? (std::string(" desc78=0x") + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead64(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x78u); return s.str(); }() +
                             " desc84=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x84u); return s.str(); }() +
                             " desc98=0x" + [&]() { std::ostringstream s; s << std::hex << Ps2FastRead32(m_memory.getRDRAM(), getRegU32(ctx, 4) + 0x98u); return s.str(); }())
                          : std::string())
                  << std::dec << std::endl;
    }
#endif
    if (targetPc == 0x186708u || targetPc == 0x187F98u ||
        targetPc == 0x1A8828u || targetPc == 0x1A8878u)
    {
        std::cerr << "[probe:decoder-call] target=0x" << std::hex << targetPc
                  << " after pc=0x" << ctx->pc << " v0=0x" << getRegU32(ctx, 2)
                  << " ra=0x" << getRegU32(ctx, 31)
                  << " qwords=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D450u)
                  << " ctl10=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D390u)
                  << " ctl88=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D408u)
                  << " ctl90=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D410u)
                  << " ctlA8=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D428u])
                  << " ctlA9=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D429u])
                  << " ctlAF=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D42Fu])
                  << " status=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D45Eu])
                  << " cop0=0x" << ctx->cop0_status
                  << std::dec << '\n';
    }
    if (targetPc == 0x187F98u)
    {
        static uint32_t decoderInputHashProbeCount = 0u;
        if (decoderInputHashProbeCount++ < 4u)
        {
            const uint32_t input = getRegU32(ctx, 6);
            const uint32_t inputBytes = Ps2FastRead32(m_memory.getRDRAM(), 0x101D40Cu);
            uint64_t hash = 1469598103934665603ull;
            for (uint32_t i = 0u; i < inputBytes; ++i)
            {
                hash ^= m_memory.getRDRAM()[input + i];
                hash *= 1099511628211ull;
            }
            std::cerr << "[probe:decoder-input-hash] ptr=0x" << std::hex << input
                      << " bytes=0x" << inputBytes << " fnv=0x" << hash << std::dec << '\n';
            for (uint32_t segment = 0u; segment < inputBytes; segment += 0x20000u)
            {
                const uint32_t segmentBytes = std::min<uint32_t>(0x20000u, inputBytes - segment);
                uint64_t segmentHash = 1469598103934665603ull;
                for (uint32_t i = 0u; i < segmentBytes; ++i)
                {
                    segmentHash ^= m_memory.getRDRAM()[input + segment + i];
                    segmentHash *= 1099511628211ull;
                }
                std::cerr << "[probe:decoder-input-segment] n=0x" << std::hex << (segment / 0x20000u)
                          << " fnv=0x" << segmentHash << std::dec << '\n';
            }
        }
        std::cerr << "[probe:decoder-state] cap=0x" << std::hex
                  << Ps2FastRead32(m_memory.getRDRAM(), 0x101D448u)
                  << " out=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D44Cu)
                  << " qwords=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D450u)
                  << " input=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D454u)
                  << " base=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D458u)
                  << " bits=0x" << Ps2FastRead16(m_memory.getRDRAM(), 0x101D45Cu)
                  << " status=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D45Eu])
                  << std::dec << '\n';
    }
    if (targetPc == 0x186708u)
    {
        static uint32_t decoderPumpStateProbeCount = 0u;
        if (decoderPumpStateProbeCount++ < 96u)
        {
            std::cerr << "[probe:decoder-pump-state] state10=0x" << std::hex
                      << Ps2FastRead32(m_memory.getRDRAM(), 0x101D450u)
                      << " state14=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D454u)
                      << " state18=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D458u)
                      << " state1c=0x" << Ps2FastRead16(m_memory.getRDRAM(), 0x101D45Cu)
                      << " state1e=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D45Eu])
                      << " desc84=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D404u)
                      << " desc88=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D408u)
                      << " desc90=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x101D410u)
                      << " desca9=0x" << static_cast<unsigned>(m_memory.getRDRAM()[0x101D429u])
                      << std::dec << '\n';
        }
        // Throwaway decoder-output diagnostic: hash the decompressed output once
        // the DVD producer goes idle after serving the FILE_02 stream. This is
        // generic (reads guest RAM, no title address) and shows whether the
        // custom decoder made progress before the boot wait.
        {
            const uint8_t ac = m_memory.getRDRAM()[0x101D42Cu];
            const uint8_t ad = m_memory.getRDRAM()[0x101D42Du];
            const uint32_t outBase = Ps2FastRead32(m_memory.getRDRAM(), 0x101D44Cu);
            const uint32_t outCap = Ps2FastRead32(m_memory.getRDRAM(), 0x101D448u);
            static bool outHashDone = false;
            if (!outHashDone && ac == 0u && ad == 1u && outBase != 0u && outCap != 0u)
            {
                outHashDone = true;
                uint64_t h = 1469598103934665603ull;
                uint32_t nonZero = 0u;
                const uint32_t bytes = std::min<uint32_t>(outCap, 0x22c7f4u);
                for (uint32_t i = 0u; i < bytes; ++i)
                {
                    const uint8_t b = m_memory.getRDRAM()[outBase + i];
                    if (b != 0u)
                        ++nonZero;
                    h ^= b;
                    h *= 1099511628211ull;
                }
                std::cerr << "[probe:decoder-output] base=0x" << std::hex << outBase
                          << " cap=0x" << outCap << " fnv=0x" << h
                          << " nonzero=0x" << nonZero
                          << " q987900=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x987900u)
                          << " q987904=0x" << Ps2FastRead32(m_memory.getRDRAM(), 0x987904u) << " head=";
                for (uint32_t i = 0u; i < 16u && i < bytes; ++i)
                    std::cerr << static_cast<unsigned>(m_memory.getRDRAM()[outBase + i]) << ' ';
                std::cerr << " tail=";
                {
                    uint32_t tailNonZero = 0u;
                    uint64_t tailHash = 1469598103934665603ull;
                    const uint32_t tailBytes = std::min<uint32_t>(0x1000u, bytes);
                    for (uint32_t i = 0u; i < tailBytes; ++i)
                    {
                        const uint8_t b = m_memory.getRDRAM()[outBase + bytes - tailBytes + i];
                        if (b != 0u)
                            ++tailNonZero;
                        tailHash ^= b;
                        tailHash *= 1099511628211ull;
                    }
                    std::cerr << "nonzero=0x" << tailNonZero << " fnv=0x" << tailHash;
                }
                {
                    uint32_t lastNonZero = 0u;
                    for (uint32_t i = 0u; i < bytes; ++i)
                    {
                        if (m_memory.getRDRAM()[outBase + i] != 0u)
                            lastNonZero = i;
                    }
                    std::cerr << " lastnz=0x" << lastNonZero;
                }
                std::cerr << std::dec << '\n';
            }
        }
    }
    if (targetPc == 0x1865E8u)
    {
        const uint32_t control = 0x0101D380u;
        std::cerr << "[probe:decoder-pump] pc=0x" << std::hex << ctx->pc
                  << " v0=0x" << getRegU32(ctx, 2)
                  << " report10=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x10u)
                  << " desc84=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x84u)
                  << " desc88=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x88u)
                  << " desc90=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x90u)
                  << " stateac=0x" << static_cast<unsigned>(m_memory.getRDRAM()[control + 0xACu])
                  << " statead=0x" << static_cast<unsigned>(m_memory.getRDRAM()[control + 0xADu])
                  << std::dec << '\n';
    }
    if (targetPc == 0x185BF8u)
    {
        const uint32_t control = 0x0101D380u;
        std::cerr << "[probe:filectrl-status] pc=0x" << std::hex << ctx->pc
                  << " v0=0x" << getRegU32(ctx, 2)
                  << " report10=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x10u)
                  << " desc84=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x84u)
                  << " desc88=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x88u)
                  << " desc90=0x" << Ps2FastRead32(m_memory.getRDRAM(), control + 0x90u)
                  << " stateac=0x" << static_cast<unsigned>(m_memory.getRDRAM()[control + 0xACu])
                  << " statead=0x" << static_cast<unsigned>(m_memory.getRDRAM()[control + 0xADu])
                  << std::dec << '\n';
    }

    // THROWAWAY-DIAGNOSTIC: identify the call which leaves the guest PC on
    // the boot-state data pointer instead of its normal fallthrough/return.
    if (ctx->pc == 0x54DAC0u)
    {
        std::cerr << "[probe:pc54dac0] target=0x" << std::hex << targetPc
                  << " source=0x" << sourcePc << " fallthrough=0x" << fallthroughPc
                  << " kind=" << debugName << " ra=0x" << getRegU32(ctx, 31)
                  << " sp=0x" << getRegU32(ctx, 29) << std::dec << std::endl;
    }

    if (isStopRequested() || ctx->pc == 0u)
    {
        return false;
    }

    if (ctx->pc == entryPc)
    {
        ctx->pc = fallthroughPc;
    }

    return ctx->pc == fallthroughPc;
}

void PS2Runtime::SignalException(R5900Context *ctx, PS2Exception exception)
{
    if (exception == EXCEPTION_INTEGER_OVERFLOW)
    {
        HandleIntegerOverflow(ctx);
        return;
    }

    raiseCop0Exception(ctx, static_cast<uint32_t>(exception),
                       exception == EXCEPTION_TLB_REFILL);
}

void PS2Runtime::executeVU0Microprogram(uint8_t *rdram, R5900Context *ctx, uint32_t address)
{
    (void)rdram;

    uint8_t *const vu0Code = m_memory.getVU0Code();
    uint8_t *const vu0Data = m_memory.getVU0Data();
    const uint32_t startPC = address & ~0x7u;

    if (!vu0Code || !vu0Data || startPC + 8u > PS2_VU0_CODE_SIZE)
    {
        seedVu0IdleSuccess(ctx);
        return;
    }

    m_vu0.reset();
    copyVu0ContextToState(ctx, m_vu0.state());
    m_vu0.execute(vu0Code, PS2_VU0_CODE_SIZE,
                  vu0Data, PS2_VU0_DATA_SIZE,
                  m_gs, &m_memory,
                  startPC, 0u, ctx->vu0_itop, 4096);
    copyVu0StateToContext(m_vu0.state(), ctx);
}

void PS2Runtime::vu0StartMicroProgram(uint8_t *rdram, R5900Context *ctx, uint32_t address)
{
    // VCALLMS and VCALLMSR both route here.
    executeVU0Microprogram(rdram, ctx, address);
}

void PS2Runtime::handleSyscall(uint8_t *rdram, R5900Context *ctx)
{
    handleSyscall(rdram, ctx, 0);
}

void PS2Runtime::handleSyscall(uint8_t *rdram, R5900Context *ctx, uint32_t encodedSyscallId)
{
    if (ctx->in_delay_slot)
    {
        throw std::runtime_error("Attempted to execute a syscall inside a branch delay slot! "
                                 "This breaks the atomic basic block model and is structurally unsupported by the emulator.");
    }

    const uint32_t syscallId = (encodedSyscallId != 0u)
                                   ? encodedSyscallId
                                   : getRegU32(ctx, 3); // $v1 / $3 is the EE kernel syscall number

    if (ps2_syscalls::dispatchNumericSyscall(syscallId, rdram, ctx, this))
    {
        return;
    }

    // God help you
    ps2_syscalls::TODO(rdram, ctx, this, encodedSyscallId);
}

void PS2Runtime::handleBreak(uint8_t *rdram, R5900Context *ctx)
{
    raiseCop0Exception(ctx, EXCEPTION_BREAKPOINT);
}

void PS2Runtime::drainCompletedDmacHandlers(uint8_t *rdram)
{
    for (uint32_t cause : m_memory.consumeCompletedDmacCauses())
    {
        ps2_syscalls::dispatchDmacHandlersForCause(rdram, this, cause);
    }
}

void PS2Runtime::handleTrap(uint8_t *rdram, R5900Context *ctx)
{
    raiseCop0Exception(ctx, EXCEPTION_TRAP);
}

void PS2Runtime::handleTLBR(uint8_t *rdram, R5900Context *ctx)
{
    uint32_t vpn = 0;
    uint32_t pfn = 0;
    uint32_t mask = 0;
    bool valid = false;

    const uint32_t index = ctx->cop0_index & 0x3Fu;
    if (!m_memory.tlbRead(index, vpn, pfn, mask, valid))
    {
        raiseCop0Exception(ctx, EXCEPTION_RESERVED_INSTRUCTION);
        return;
    }

    // Preserve low ASID bits in EntryHi.
    ctx->cop0_entryhi = (ctx->cop0_entryhi & 0x00000FFFu) | (vpn & 0xFFFFF000u);
    ctx->cop0_entrylo0 = (ctx->cop0_entrylo0 & ~0x03FFFFC2u) |
                         ((pfn & 0x000FFFFFu) << 6) |
                         (valid ? 0x2u : 0u);
    ctx->cop0_pagemask = mask & 0x01FFE000u;
}

void PS2Runtime::handleTLBWI(uint8_t *rdram, R5900Context *ctx)
{
    const uint32_t index = ctx->cop0_index & 0x3Fu;
    const uint32_t vpn = ctx->cop0_entryhi & 0xFFFFF000u;
    const uint32_t pfn = (ctx->cop0_entrylo0 >> 6) & 0x000FFFFFu;
    const uint32_t mask = ctx->cop0_pagemask & 0x01FFE000u;
    const bool valid = (ctx->cop0_entrylo0 & 0x2u) != 0u;

    if (!m_memory.tlbWrite(index, vpn, pfn, mask, valid))
    {
        raiseCop0Exception(ctx, EXCEPTION_RESERVED_INSTRUCTION);
    }
}

void PS2Runtime::handleTLBWR(uint8_t *rdram, R5900Context *ctx)
{
    const uint32_t entryCount = static_cast<uint32_t>(m_memory.tlbEntryCount());
    if (entryCount == 0)
    {
        raiseCop0Exception(ctx, EXCEPTION_RESERVED_INSTRUCTION);
        return;
    }

    const uint32_t wired = std::min(ctx->cop0_wired, entryCount - 1);
    uint32_t random = ctx->cop0_random % entryCount;
    if (random < wired)
    {
        random = wired;
    }

    const uint32_t vpn = ctx->cop0_entryhi & 0xFFFFF000u;
    const uint32_t pfn = (ctx->cop0_entrylo0 >> 6) & 0x000FFFFFu;
    const uint32_t mask = ctx->cop0_pagemask & 0x01FFE000u;
    const bool valid = (ctx->cop0_entrylo0 & 0x2u) != 0u;

    if (!m_memory.tlbWrite(random, vpn, pfn, mask, valid))
    {
        raiseCop0Exception(ctx, EXCEPTION_RESERVED_INSTRUCTION);
        return;
    }

    // Keep COP0 bookkeeping in sync with the selected slot.
    ctx->cop0_index = (ctx->cop0_index & ~0x3Fu) | (random & 0x3Fu);
    ctx->cop0_random = (random <= wired) ? (entryCount - 1) : (random - 1);
}

void PS2Runtime::handleTLBP(uint8_t *rdram, R5900Context *ctx)
{
    const int32_t index = m_memory.tlbProbe(ctx->cop0_entryhi & 0xFFFFF000u);
    if (index >= 0)
    {
        ctx->cop0_index = (ctx->cop0_index & ~0x8000003Fu) |
                          (static_cast<uint32_t>(index) & 0x3Fu);
    }
    else
    {
        // MIPS sets probe failure bit (P) in Index[31].
        ctx->cop0_index |= 0x80000000u;
    }
}

void PS2Runtime::clearLLBit(R5900Context *ctx)
{
    // LL/SC reservation is tracked separately from COP0 Status.
    ctx->llbit = 0;
    ctx->lladdr = 0;
}

uint32_t PS2Runtime::alignGuestHeapValue(uint32_t value, uint32_t alignment)
{
    if (alignment == 0)
    {
        return value;
    }

    const uint32_t mask = alignment - 1u;
    if (value > (std::numeric_limits<uint32_t>::max() - mask))
    {
        return std::numeric_limits<uint32_t>::max();
    }
    return (value + mask) & ~mask;
}

bool PS2Runtime::isGuestHeapAlignmentValid(uint32_t alignment)
{
    return alignment != 0u && (alignment & (alignment - 1u)) == 0u;
}

uint32_t PS2Runtime::normalizeGuestHeapAlignment(uint32_t alignment)
{
    if (!isGuestHeapAlignmentValid(alignment))
    {
        return kGuestHeapDefaultAlignment;
    }
    return std::max(alignment, kGuestHeapDefaultAlignment);
}

uint32_t PS2Runtime::clampGuestHeapBase(uint32_t guestBase) const
{
    uint32_t normalized = guestBase;
    if (normalized >= PS2_RAM_SIZE)
    {
        normalized &= PS2_RAM_MASK;
    }
    const uint32_t hardLimit = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
    return std::min(normalized, hardLimit);
}

uint32_t PS2Runtime::clampGuestHeapLimit(uint32_t guestLimit) const
{
    const uint32_t hardLimit = std::min(kGuestHeapHardLimit, PS2_RAM_SIZE);
    if (guestLimit == 0u || guestLimit > hardLimit)
    {
        return hardLimit;
    }
    return guestLimit;
}

void PS2Runtime::resetGuestHeapLocked(uint32_t guestBase, uint32_t guestLimit)
{
    uint32_t base = alignGuestHeapValue(clampGuestHeapBase(guestBase), kGuestHeapDefaultAlignment);
    uint32_t limit = clampGuestHeapLimit(guestLimit);
    if (base == 0u)
    {
        const uint32_t fallbackBase = (m_guestHeapSuggestedBase != 0u) ? m_guestHeapSuggestedBase : kGuestHeapDefaultBase;
        base = alignGuestHeapValue(clampGuestHeapBase(fallbackBase), kGuestHeapDefaultAlignment);
    }

    if (limit <= base)
    {
        base = alignGuestHeapValue(clampGuestHeapBase(m_guestHeapSuggestedBase), kGuestHeapDefaultAlignment);
        limit = clampGuestHeapLimit(0u);
    }

    if (limit <= base)
    {
        base = 0u;
        limit = 0u;
    }

    m_guestHeapBlocks.clear();
    if (limit > base)
    {
        m_guestHeapBlocks.push_back({base, limit - base, true});
    }

    m_guestHeapBase = base;
    m_guestHeapEnd = base;
    m_guestHeapLimit = limit;
    m_guestHeapConfigured = true;
}

void PS2Runtime::ensureGuestHeapInitializedLocked()
{
    if (m_guestHeapConfigured)
    {
        return;
    }

    const uint32_t suggested = (m_guestHeapSuggestedBase == 0u) ? kGuestHeapDefaultBase : m_guestHeapSuggestedBase;
    resetGuestHeapLocked(suggested, clampGuestHeapLimit(0u));
}

int32_t PS2Runtime::findGuestHeapBlockIndexLocked(uint32_t guestAddr) const
{
    const uint32_t normalizedAddr = guestAddr & PS2_RAM_MASK;
    for (size_t i = 0; i < m_guestHeapBlocks.size(); ++i)
    {
        const GuestHeapBlock &block = m_guestHeapBlocks[i];
        if (!block.free && block.addr == normalizedAddr)
        {
            return static_cast<int32_t>(i);
        }
    }
    return -1;
}

uint32_t PS2Runtime::allocateGuestBlockLocked(uint32_t size, uint32_t alignment)
{
    if (size == 0u)
    {
        return 0u;
    }

    const uint32_t normalizedAlignment = normalizeGuestHeapAlignment(alignment);
    if (size > (std::numeric_limits<uint32_t>::max() - (kGuestHeapDefaultAlignment - 1u)))
    {
        return 0u;
    }

    const uint32_t allocSize = alignGuestHeapValue(size, kGuestHeapDefaultAlignment);
    if (allocSize == 0u)
    {
        return 0u;
    }

    for (size_t i = 0; i < m_guestHeapBlocks.size(); ++i)
    {
        const GuestHeapBlock block = m_guestHeapBlocks[i];
        if (!block.free)
        {
            continue;
        }

        const uint64_t blockStart = block.addr;
        const uint64_t blockEnd = blockStart + static_cast<uint64_t>(block.size);
        const uint32_t alignedAddr = alignGuestHeapValue(block.addr, normalizedAlignment);
        if (alignedAddr < block.addr)
        {
            continue;
        }

        const uint64_t alignedStart = alignedAddr;
        if (alignedStart > blockEnd)
        {
            continue;
        }

        const uint64_t allocEnd = alignedStart + static_cast<uint64_t>(allocSize);
        if (allocEnd > blockEnd)
        {
            continue;
        }

        const uint32_t prefixSize = static_cast<uint32_t>(alignedStart - blockStart);
        const uint32_t suffixSize = static_cast<uint32_t>(blockEnd - allocEnd);

        std::vector<GuestHeapBlock> replacement;
        replacement.reserve(3);
        if (prefixSize > 0u)
        {
            replacement.push_back({block.addr, prefixSize, true});
        }
        replacement.push_back({alignedAddr, allocSize, false});
        if (suffixSize > 0u)
        {
            replacement.push_back({static_cast<uint32_t>(allocEnd), suffixSize, true});
        }

        m_guestHeapBlocks.erase(m_guestHeapBlocks.begin() + static_cast<std::ptrdiff_t>(i));
        m_guestHeapBlocks.insert(m_guestHeapBlocks.begin() + static_cast<std::ptrdiff_t>(i),
                                 replacement.begin(),
                                 replacement.end());

        m_guestHeapEnd = std::max(m_guestHeapEnd, static_cast<uint32_t>(allocEnd));
        return alignedAddr;
    }

    return 0u;
}

void PS2Runtime::coalesceGuestHeapLocked()
{
    if (m_guestHeapBlocks.empty())
    {
        return;
    }

    size_t i = 1;
    while (i < m_guestHeapBlocks.size())
    {
        GuestHeapBlock &prev = m_guestHeapBlocks[i - 1];
        GuestHeapBlock &curr = m_guestHeapBlocks[i];
        const uint64_t prevEnd = static_cast<uint64_t>(prev.addr) + static_cast<uint64_t>(prev.size);
        if (prev.free && curr.free && prevEnd == curr.addr)
        {
            prev.size += curr.size;
            m_guestHeapBlocks.erase(m_guestHeapBlocks.begin() + static_cast<std::ptrdiff_t>(i));
            continue;
        }
        ++i;
    }
}

void PS2Runtime::freeGuestBlockLocked(uint32_t guestAddr)
{
    const int32_t index = findGuestHeapBlockIndexLocked(guestAddr);
    if (index < 0)
    {
        return;
    }

    m_guestHeapBlocks[static_cast<size_t>(index)].free = true;
    coalesceGuestHeapLocked();
}

void PS2Runtime::configureGuestHeap(uint32_t guestBase, uint32_t guestLimit)
{
    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    uint32_t normalizedBase = alignGuestHeapValue(clampGuestHeapBase(guestBase), kGuestHeapDefaultAlignment);
    if (normalizedBase == 0u)
    {
        normalizedBase = (m_guestHeapSuggestedBase != 0u) ? m_guestHeapSuggestedBase : kGuestHeapDefaultBase;
    }
    m_guestHeapSuggestedBase = normalizedBase;
    resetGuestHeapLocked(normalizedBase, guestLimit);
}

uint32_t PS2Runtime::guestMalloc(uint32_t size, uint32_t alignment)
{
    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    ensureGuestHeapInitializedLocked();
    return allocateGuestBlockLocked(size, alignment);
}

uint32_t PS2Runtime::guestCalloc(uint32_t count, uint32_t size, uint32_t alignment)
{
    if (count == 0u || size == 0u)
    {
        return 0u;
    }
    if (count > (std::numeric_limits<uint32_t>::max() / size))
    {
        return 0u;
    }

    const uint32_t totalSize = count * size;
    const uint32_t guestAddr = guestMalloc(totalSize, alignment);
    if (guestAddr != 0u)
    {
        uint8_t *rdram = m_memory.getRDRAM();
        if (rdram)
        {
            uint32_t physAddr = guestAddr & PS2_RAM_MASK;
            if (physAddr + totalSize <= PS2_RAM_SIZE)
                std::memset(rdram + physAddr, 0, totalSize);
        }
    }

    return guestAddr;
}

uint32_t PS2Runtime::guestRealloc(uint32_t guestAddr, uint32_t newSize, uint32_t alignment)
{
    if (guestAddr == 0u)
    {
        return guestMalloc(newSize, alignment);
    }
    if (newSize == 0u)
    {
        guestFree(guestAddr);
        return 0u;
    }

    if (newSize > (std::numeric_limits<uint32_t>::max() - (kGuestHeapDefaultAlignment - 1u)))
    {
        return 0u;
    }

    const uint32_t normalizedAlignment = normalizeGuestHeapAlignment(alignment);
    const uint32_t requestedSize = alignGuestHeapValue(newSize, kGuestHeapDefaultAlignment);

    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    ensureGuestHeapInitializedLocked();

    const int32_t index = findGuestHeapBlockIndexLocked(guestAddr);
    if (index < 0)
    {
        return 0u;
    }

    const size_t blockIndex = static_cast<size_t>(index);
    const uint32_t oldAddr = m_guestHeapBlocks[blockIndex].addr;
    const uint32_t oldSize = m_guestHeapBlocks[blockIndex].size;

    if (requestedSize <= oldSize)
    {
        if (requestedSize < oldSize)
        {
            const uint32_t tailAddr = oldAddr + requestedSize;
            const uint32_t tailSize = oldSize - requestedSize;
            m_guestHeapBlocks[blockIndex].size = requestedSize;
            m_guestHeapBlocks.insert(m_guestHeapBlocks.begin() + static_cast<std::ptrdiff_t>(blockIndex + 1u),
                                     GuestHeapBlock{tailAddr, tailSize, true});
            coalesceGuestHeapLocked();
        }
        return oldAddr;
    }

    if (blockIndex + 1u < m_guestHeapBlocks.size())
    {
        GuestHeapBlock &next = m_guestHeapBlocks[blockIndex + 1u];
        const uint64_t blockEnd = static_cast<uint64_t>(m_guestHeapBlocks[blockIndex].addr) +
                                  static_cast<uint64_t>(m_guestHeapBlocks[blockIndex].size);
        if (next.free && blockEnd == next.addr)
        {
            const uint64_t combined = static_cast<uint64_t>(m_guestHeapBlocks[blockIndex].size) +
                                      static_cast<uint64_t>(next.size);
            if (combined >= requestedSize)
            {
                const uint32_t extraNeeded = requestedSize - m_guestHeapBlocks[blockIndex].size;
                m_guestHeapBlocks[blockIndex].size = requestedSize;
                if (next.size == extraNeeded)
                {
                    m_guestHeapBlocks.erase(m_guestHeapBlocks.begin() + static_cast<std::ptrdiff_t>(blockIndex + 1u));
                }
                else
                {
                    next.addr += extraNeeded;
                    next.size -= extraNeeded;
                }
                m_guestHeapEnd = std::max(m_guestHeapEnd, oldAddr + requestedSize);
                return oldAddr;
            }
        }
    }

    const uint32_t newAddr = allocateGuestBlockLocked(newSize, normalizedAlignment);
    if (newAddr == 0u)
    {
        return 0u;
    }

    uint8_t *rdram = m_memory.getRDRAM();
    if (rdram)
    {
        const uint32_t copyBytes = std::min(oldSize, newSize);
        uint32_t dstPhys = newAddr & PS2_RAM_MASK;
        uint32_t srcPhys = oldAddr & PS2_RAM_MASK;
        if (dstPhys + copyBytes <= PS2_RAM_SIZE && srcPhys + copyBytes <= PS2_RAM_SIZE)
            std::memmove(rdram + dstPhys, rdram + srcPhys, copyBytes);
    }

    freeGuestBlockLocked(oldAddr);
    return newAddr;
}

void PS2Runtime::guestFree(uint32_t guestAddr)
{
    if (guestAddr == 0u)
    {
        return;
    }

    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    ensureGuestHeapInitializedLocked();
    freeGuestBlockLocked(guestAddr);
}

uint32_t PS2Runtime::guestHeapBase() const
{
    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    return m_guestHeapConfigured ? m_guestHeapBase : m_guestHeapSuggestedBase;
}

uint32_t PS2Runtime::guestHeapEnd() const
{
    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    return m_guestHeapConfigured ? m_guestHeapEnd : m_guestHeapSuggestedBase;
}

uint32_t PS2Runtime::guestHeapLimit() const
{
    std::lock_guard<std::mutex> lock(m_guestHeapMutex);
    return m_guestHeapConfigured ? m_guestHeapLimit : m_guestHeapSuggestedBase;
}

uint32_t PS2Runtime::reserveAsyncCallbackStack(uint32_t size, uint32_t alignment)
{
    if (size == 0u)
    {
        return 0u;
    }

    const uint32_t normalizedAlignment = normalizeGuestHeapAlignment(alignment);
    const uint32_t allocSize = alignGuestHeapValue(size, kGuestHeapDefaultAlignment);
    if (allocSize == 0u)
    {
        return 0u;
    }

    // Callback frames are guest-visible memory and must participate in the
    // same interval allocator as guest malloc.  The old top-of-RAM bump
    // region could overlap an otherwise valid DMA destination supplied by the
    // game (the callback stack was then writing into the compressed stream).
    // Allocating from the tracked heap gives every runtime-owned guest range a
    // single non-overlapping ownership model, independent of game layout.
    {
        std::lock_guard<std::mutex> lock(m_guestHeapMutex);
        ensureGuestHeapInitializedLocked();
        const uint32_t base = allocateGuestBlockLocked(allocSize, normalizedAlignment);
        if (base != 0u)
        {
            return base + allocSize - 0x10u;
        }
    }

    // Preserve the legacy pool as a last-resort compatibility path for
    // runtimes that deliberately configure an empty heap (some focused tests
    // and minimal hosts do this).  Normal loaded games take the tracked path
    // above, which is the one that prevents DMA/stack overlap.
    std::lock_guard<std::mutex> lock(m_asyncCallbackStackMutex);
    if (m_guestHeapSuggestedBase > (kGuestHeapDefaultBase + kGuestHeapSafetyPad))
    {
        // Loaded games with an empty configured heap still have a safe,
        // unclaimed interval between the image end and the heap hard limit.
        // Grow the callback pool upward through that interval so it cannot
        // overlap game-owned data near the top of RAM.
        uint32_t base = alignGuestHeapValue(m_asyncCallbackStackFloor, normalizedAlignment);
        const uint32_t limit = std::min(m_asyncCallbackStackTop, PS2_RAM_SIZE);
        const uint64_t end = static_cast<uint64_t>(base) + allocSize;
        if (base < limit && end <= limit)
        {
            m_asyncCallbackStackFloor = static_cast<uint32_t>(end);
            return static_cast<uint32_t>(end) - 0x10u;
        }
        return 0u;
    }

    // Minimal hosts without a loaded image retain the historical top-down
    // pool as a compatibility fallback.
    uint32_t top = m_asyncCallbackStackTop;
    if (top > PS2_RAM_SIZE)
    {
        top = PS2_RAM_SIZE;
    }
    top &= ~(kGuestHeapDefaultAlignment - 1u);
    if (top <= allocSize)
    {
        return 0u;
    }
    uint32_t base = top - allocSize;
    base &= ~(normalizedAlignment - 1u);
    if (base < m_asyncCallbackStackFloor || base >= top)
    {
        return 0u;
    }
    m_asyncCallbackStackTop = base;
    return top - 0x10u;
}

uint8_t PS2Runtime::Load8(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr)
{
    try
    {
        return m_memory.read8(vaddr);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_LOAD);
        return 0;
    }
}

uint16_t PS2Runtime::Load16(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr)
{
    try
    {
        return m_memory.read16(vaddr);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_LOAD);
        return 0;
    }
}

uint32_t PS2Runtime::Load32(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr)
{
    try
    {
        return m_memory.read32(vaddr);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_LOAD);
        return 0;
    }
}

uint64_t PS2Runtime::Load64(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr)
{
    try
    {
        return m_memory.read64(vaddr);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_LOAD);
        return 0;
    }
}

__m128i PS2Runtime::Load128(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr)
{
    try
    {
        return m_memory.read128(vaddr);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_LOAD);
        return _mm_setzero_si128();
    }
}

void PS2Runtime::Store8(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr, uint8_t value)
{
    // Temporary post-load task probe: the boot task waits on its completion
    // byte at state+0x0c (state is normally 0x54dac0). Keep this in the
    // implementation so it does not perturb the generated headers.
    const uint32_t physicalAddr = vaddr & PS2_RAM_MASK;
    if ((physicalAddr == 0x54DACCu || physicalAddr == 0x5611CCu ||
         (physicalAddr >= 0x101BDDCu && physicalAddr <= 0x101BDE4u)) &&
        ctx != nullptr)
    {
        static std::atomic<uint32_t> taskCompletionWrites{0u};
        if (taskCompletionWrites.fetch_add(1u, std::memory_order_relaxed) < 64u)
        {
            std::cerr << "[probe:task-completion-write] addr=0x" << std::hex << vaddr
                      << " value=0x" << static_cast<unsigned>(value)
                      << " pc=0x" << ctx->pc
                      << " ra=0x" << getRegU32(ctx, 31) << std::dec << '\n';
        }
    }
    ps2TraceGuestWrite(rdram, vaddr, 1u, value, 0u, "WRITE8", ctx);
    try
    {
        m_memory.write8(vaddr, value);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_STORE);
    }
}

void PS2Runtime::Store16(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr, uint16_t value)
{
    ps2TraceGuestWrite(rdram, vaddr, 2u, value, 0u, "WRITE16", ctx);
    try
    {
        m_memory.write16(vaddr, value);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_STORE);
    }
}

void PS2Runtime::Store32(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr, uint32_t value)
{
    // TEMP-EXPERIMENT: watch boot-idx word writes. Revert.
    {
        const uint32_t physicalAddr = vaddr & PS2_RAM_MASK;
        if ((physicalAddr == 0x101BDE0u) && ctx != nullptr)
        {
            static std::atomic<uint32_t> idxWrites{0u};
            if (idxWrites.fetch_add(1u, std::memory_order_relaxed) < 32u)
            {
                std::cerr << "[probe:idx-write] addr=0x" << std::hex << vaddr
                          << " value=0x" << value
                          << " pc=0x" << ctx->pc
                          << " ra=0x" << getRegU32(ctx, 31) << std::dec << '\n';
            }
        }
    }
    ps2TraceGuestWrite(rdram, vaddr, 4u, value, 0u, "WRITE32", ctx);
    try
    {
        m_memory.write32(vaddr, value);
        drainCompletedDmacHandlers(rdram);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_STORE);
    }
}

void PS2Runtime::Store64(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr, uint64_t value)
{
    ps2TraceGuestWrite(rdram, vaddr, 8u, value, 0u, "WRITE64", ctx);
    try
    {
        m_memory.write64(vaddr, value);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_STORE);
    }
}

void PS2Runtime::Store128(uint8_t *rdram, R5900Context *ctx, uint32_t vaddr, __m128i value)
{
    alignas(16) uint64_t _parts[2];
    _mm_storeu_si128(reinterpret_cast<__m128i *>(_parts), value);
    ps2TraceGuestWrite(rdram, vaddr, 16u, _parts[0], _parts[1], "WRITE128", ctx);
    try
    {
        m_memory.write128(vaddr, value);
    }
    catch (const std::exception &)
    {
        SignalException(ctx, EXCEPTION_ADDRESS_ERROR_STORE);
    }
}

void PS2Runtime::kickGifDmaChainFromMMIO(uint8_t *rdram,
                                         R5900Context *ctx,
                                         uint32_t dPcrValue,
                                         uint32_t dStatValue,
                                         uint32_t tadr,
                                         uint32_t chcr)
{
    constexpr uint32_t D_PCR = 0x1000E020u;
    constexpr uint32_t D_STAT = 0x1000E010u;
    constexpr uint32_t GIF_TADR = 0x1000A030u;
    constexpr uint32_t GIF_CHCR = 0x1000A000u;

    ps2TraceGuestWrite(rdram, D_PCR, 4u, dPcrValue, 0u, "WRITE32", ctx);
    m_memory.writeIORegister(D_PCR, dPcrValue);
    ps2TraceGuestWrite(rdram, D_STAT, 4u, dStatValue, 0u, "WRITE32", ctx);
    m_memory.writeIORegister(D_STAT, dStatValue);
    ps2TraceGuestWrite(rdram, GIF_TADR, 4u, tadr, 0u, "WRITE32", ctx);
    m_memory.writeIORegister(GIF_TADR, tadr);
    ps2TraceGuestWrite(rdram, GIF_CHCR, 4u, chcr, 0u, "WRITE32", ctx);
    if (m_memory.tryProcessNativeGifImageUploadChain(m_gs, tadr, chcr))
    {
        drainCompletedDmacHandlers(rdram);
        return;
    }
    if (m_memory.tryProcessNativeGifPackedChain(m_gs, tadr, chcr))
    {
        drainCompletedDmacHandlers(rdram);
        return;
    }
    m_memory.writeIORegister(GIF_CHCR, chcr);
    m_memory.processPendingTransfers();
    drainCompletedDmacHandlers(rdram);
}

void PS2Runtime::requestStop()
{
    m_stopRequested.store(true, std::memory_order_relaxed);
    if (m_eeScheduler)
    {
        m_eeScheduler->requestStop();
    }
}

bool PS2Runtime::isStopRequested() const
{
    return m_stopRequested.load(std::memory_order_relaxed);
}

EeScheduler &PS2Runtime::eeScheduler()
{
    return *m_eeScheduler;
}

const EeScheduler &PS2Runtime::eeScheduler() const
{
    return *m_eeScheduler;
}

void PS2Runtime::postEeEvent(EeEvent event)
{
    m_eeScheduler->postEvent(event);
}

bool PS2Runtime::eeCheckpointDue(uint32_t cycles) noexcept
{
    // Generated backward edges call this helper for every small guest slice
    // (normally 32 cycles).  Crossing the C++ scheduler boundary that often
    // dominates long linear decoders, while the EE only needs scheduling
    // decisions at instruction/event granularity.  Batch the accounting and
    // event check into a short guest-time quantum; this preserves cycle order
    // and keeps event latency bounded without changing guest code.
    thread_local uint32_t accumulatedCycles = 0u;
    // Keep long linear generated slices out of the scheduler hot path while
    // retaining a sub-second guest-time upper bound for pending events.
    constexpr uint32_t kCheckpointQuantum = 1u << 30;
    accumulatedCycles += cycles;
    if (accumulatedCycles < kCheckpointQuantum)
    {
        return false;
    }
    const uint32_t elapsed = accumulatedCycles;
    accumulatedCycles = 0u;
    return m_eeScheduler->checkpointDue(elapsed);
}

[[noreturn]] void PS2Runtime::eeWaitVSyncTicks(uint32_t ticks, uint32_t resumePc)
{
    const uint64_t currentTick = m_eeScheduler->currentVSyncTick();
    const uint64_t waitTicks = std::max<uint64_t>(1u, ticks);
    m_eeScheduler->waitVSync(currentTick + waitTicks - 1u,
                             0,
                             [resumePc](R5900Context &context)
                             {
                                 context.pc = resumePc;
                             });
}

void PS2Runtime::addEeExitHandler(int threadId, uint32_t function, uint32_t argument)
{
    std::lock_guard lock(m_eeKernelStateMutex);
    m_eeExitHandlers[threadId].push_back({function, argument});
}

std::vector<PS2Runtime::EeExitHandlerRegistration> PS2Runtime::takeEeExitHandlers(int threadId)
{
    std::lock_guard lock(m_eeKernelStateMutex);
    auto it = m_eeExitHandlers.find(threadId);
    if (it == m_eeExitHandlers.end())
    {
        return {};
    }
    auto handlers = std::move(it->second);
    m_eeExitHandlers.erase(it);
    return handlers;
}

void PS2Runtime::removeEeExitHandlers(int threadId)
{
    std::lock_guard lock(m_eeKernelStateMutex);
    m_eeExitHandlers.erase(threadId);
}

bool PS2Runtime::findEeSyscallOverride(uint32_t syscallNumber, uint32_t &handler) const
{
    std::lock_guard lock(m_eeKernelStateMutex);
    const auto it = m_eeSyscallOverrides.find(syscallNumber);
    if (it == m_eeSyscallOverrides.end())
    {
        return false;
    }
    handler = it->second;
    return true;
}

void PS2Runtime::setEeSyscallOverride(uint8_t *rdram, uint32_t syscallNumber, uint32_t handler)
{
    constexpr uint32_t kTableBase = 0x80011F80u & 0x1FFFFFFFu;
    constexpr uint32_t kMirrorLimit = 0x00080000u;
    const int64_t offset = static_cast<int64_t>(static_cast<int32_t>(syscallNumber)) * 4;
    const int64_t address = static_cast<int64_t>(kTableBase) + offset;

    std::lock_guard lock(m_eeKernelStateMutex);
    if (handler == 0u)
    {
        m_eeSyscallOverrides.erase(syscallNumber);
    }
    else
    {
        m_eeSyscallOverrides[syscallNumber] = handler;
    }
    if (!rdram || address < 0 || address + 4 > kMirrorLimit)
    {
        return;
    }
    const uint32_t guestAddress = static_cast<uint32_t>(address);
    std::memcpy(rdram + guestAddress, &handler, sizeof(handler));
    if (handler == 0u)
    {
        m_eeSyscallMirrorAddresses.erase(guestAddress);
    }
    else
    {
        m_eeSyscallMirrorAddresses.insert(guestAddress);
    }
}

void PS2Runtime::initializeEeKernelState(uint8_t *rdram)
{
    if (!rdram)
    {
        return;
    }
    constexpr uint32_t kTableGuestBase = 0x80011F80u;
    constexpr uint32_t kTableBase = kTableGuestBase & 0x1FFFFFFFu;
    constexpr uint32_t kMirrorLimit = 0x00080000u;
    constexpr uint32_t kProbeBase = 0x000002F0u;

    std::lock_guard lock(m_eeKernelStateMutex);
    for (const uint32_t address : m_eeSyscallMirrorAddresses)
    {
        const uint32_t zero = 0u;
        std::memcpy(rdram + address, &zero, sizeof(zero));
    }
    m_eeSyscallMirrorAddresses.clear();
    const uint32_t high = kTableGuestBase >> 16;
    const uint32_t low = kTableGuestBase & 0xFFFFu;
    std::memcpy(rdram + kProbeBase, &high, sizeof(high));
    std::memcpy(rdram + kProbeBase + 8u, &low, sizeof(low));
    m_eeSyscallMirrorAddresses.insert(kProbeBase);
    m_eeSyscallMirrorAddresses.insert(kProbeBase + 8u);

    for (const auto &[syscallNumber, handler] : m_eeSyscallOverrides)
    {
        const int64_t offset = static_cast<int64_t>(static_cast<int32_t>(syscallNumber)) * 4;
        const int64_t address = static_cast<int64_t>(kTableBase) + offset;
        if (address < 0 || address + 4 > kMirrorLimit)
        {
            continue;
        }
        const uint32_t guestAddress = static_cast<uint32_t>(address);
        std::memcpy(rdram + guestAddress, &handler, sizeof(handler));
        m_eeSyscallMirrorAddresses.insert(guestAddress);
    }
}

void PS2Runtime::HandleIntegerOverflow(R5900Context *ctx)
{
    raiseCop0Exception(ctx, EXCEPTION_INTEGER_OVERFLOW);
}

void PS2Runtime::run()
{
    using ProbePcCount = std::pair<uint32_t, uint64_t>;
    static constexpr struct
    {
        bool operator()(const ProbePcCount &l, const ProbePcCount &r) const
        {
            return l.second > r.second;
        }
    } ProbePcCountGreater{};
    m_stopRequested.store(false, std::memory_order_relaxed);
    ps2_stubs::resetSifState();
    resetIop();
    ps2_stubs::resetAudioStubState();
    ps2_stubs::resetMpegStubState();
    initializeEeKernelState(m_memory.getRDRAM());
    m_cpuContext.r[4] = _mm_setzero_si128();
    m_cpuContext.r[5] = _mm_setzero_si128();
    m_cpuContext.r[29] = _mm_set_epi64x(0, static_cast<int64_t>(PS2_RAM_SIZE - 0x10u));
    m_debugPc.store(m_cpuContext.pc, std::memory_order_relaxed);
    m_debugRa.store(static_cast<uint32_t>(_mm_extract_epi32(m_cpuContext.r[31], 0)), std::memory_order_relaxed);
    m_debugSp.store(static_cast<uint32_t>(_mm_extract_epi32(m_cpuContext.r[29], 0)), std::memory_order_relaxed);
    m_debugGp.store(static_cast<uint32_t>(_mm_extract_epi32(m_cpuContext.r[28], 0)), std::memory_order_relaxed);

    RUNTIME_LOG("Starting execution at address 0x" << std::hex << m_cpuContext.pc << std::dec);

    // A blank image to use as a framebuffer
    Image blank = GenImageColor(FB_WIDTH, FB_HEIGHT, BLANK);
    Texture2D frameTex = LoadTextureFromImage(blank);
    UnloadImage(blank);

    std::atomic<bool> gameThreadFinished{false};

    std::thread gameThread([&]()
                           {
        ThreadNaming::SetCurrentThreadName("GameThread");
        try
        {
            m_eeScheduler->reset(m_memory.getRDRAM(), m_cpuContext);
            m_eeScheduler->run();
            uint32_t pc = m_debugPc.load(std::memory_order_relaxed);
            RUNTIME_LOG("Game thread returned. PC=0x" << std::hex << pc
                      << " RA=0x" << static_cast<uint32_t>(_mm_extract_epi32(m_cpuContext.r[31], 0)) << std::dec << std::endl);
        }
        catch (const std::exception &e)
        {
            std::cerr << "Error during program execution: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "Error during program execution: unknown exception" << std::endl;
        }
        gameThreadFinished.store(true, std::memory_order_release); });

    uint64_t tick = 0;
    uint64_t probeSample = 0;
    std::map<uint32_t, uint64_t> probePcCounts;
    while (!isStopRequested() && !gameThreadFinished.load(std::memory_order_acquire))
    {
        PS2_IF_AGRESSIVE_LOGS({
            tick++;
            if (true)
            {
                const auto eeSnapshot = m_eeScheduler->snapshot();

                for (const auto &probeThread : eeSnapshot.threads)
                {
                    // THROWAWAY-DIAGNOSTIC: account every thread so the boot
                    // snapshot (probe:boot / probe:top) refreshes while the
                    // game runs even when thread 1 is idle. Revert before
                    // contribution.
                    probePcCounts[probeThread.pc & ~0xFu]++;
                    ++probeSample;
                }
                if ((probeSample % 60u) == 0u)
                {
                    const uint8_t *probeRdram = m_memory.getRDRAM();
                    auto probeRead32 = [probeRdram](uint32_t address) -> uint32_t
                    {
                        if (!probeRdram || address >= PS2_RAM_SIZE)
                        {
                            return 0u;
                        }
                        uint32_t value = 0u;
                        std::memcpy(&value, probeRdram + address, sizeof(value));
                        return value;
                    };
                    // Throwaway boot diagnostics (removed before contribution).
                    const uint32_t probeQueueCount = probeRead32(0x00987900u);
                    const uint32_t probeFctrlState = probeRead32(0x0101D42Cu);
                    const uint32_t probeTaskFlag = probeRead32(0x00F6D278u);
                    const uint32_t probeTaskHead = probeRead32(0x00F6D280u);
                    const uint32_t probeTaskTick0 = probeRead32(0x00F6D290u);
                    const uint32_t probeTaskTick1 = probeRead32(0x00F6D2B0u);
                    const uint32_t probeTaskTick2 = probeRead32(0x00F6D2D0u);
                    const uint32_t probeSysPtrRaw = probeRead32(0x004A9E58u);
                    const uint32_t probeSysPtr = probeSysPtrRaw & 0x1FFFFFFFu;
                    uint32_t probeSys14 = 0u;
                    uint32_t probeSys37 = 0u;
                    uint32_t probeSnd37 = 0u;
                    uint32_t probeBootState = 0u;
                    const uint32_t probeSndPtrRaw = probeRead32(0x004A9E80u);
                    const uint32_t probeSndPtr = probeSndPtrRaw & 0x1FFFFFFFu;
                    if (probeSysPtr != 0u && probeSysPtr < PS2_RAM_SIZE - 0x40u)
                    {
                        probeSys14 = probeRead32(probeSysPtr + 0x14u) & 0xFFu;
                        probeSys37 = (probeRead32(probeSysPtr + 0x34u) >> 24u) & 0xFFu;
                    }
                    if (probeSndPtr != 0u && probeSndPtr < PS2_RAM_SIZE - 0x40u)
                    {
                        probeSnd37 = (probeRead32(probeSndPtr + 0x34u) >> 24u) & 0xFFu;
                    }
                    const uint32_t probeSmBase = probeRead32(0x0101BDD0u) & 0x1FFFFFFFu;
                    if (probeSmBase != 0u && probeSmBase < PS2_RAM_SIZE - 0x20u)
                    {
                        probeBootState = probeRead32(probeSmBase + 0x14u);
                    }
                    RUNTIME_LOG("[probe:boot] queue=" << probeQueueCount
                              << " fctrlAcAd=" << std::hex << probeFctrlState
                              << " bootState=" << probeBootState
                              << " smBase=" << probeSmBase
                              << " ticks=" << probeTaskTick0 << ","
                              << probeTaskTick1 << "," << probeTaskTick2
                              << " sysPtr=" << probeSysPtrRaw
                              << " sys14=" << probeSys14
                              << " sys37=" << probeSys37
                              << " sndPtr=" << probeSndPtrRaw
                              << " snd37=" << probeSnd37
                              << std::dec << std::endl);
                    std::vector<ProbePcCount> probeTop(probePcCounts.begin(), probePcCounts.end());
                    std::sort(probeTop.begin(), probeTop.end(),
                              ProbePcCountGreater);
                    RUNTIME_LOG("[probe:top] samples=" << probeSample << std::endl);
                    for (size_t i = 0; i < probeTop.size() && i < 12; ++i)
                    {
                        RUNTIME_LOG("[probe:top] pc=0x" << std::hex << probeTop[i].first
                                  << " n=" << std::dec << probeTop[i].second << std::endl);
                    }
                }
            }
        });
        uint32_t presentWidth = FB_WIDTH;
        uint32_t presentHeight = DEFAULT_DISPLAY_HEIGHT;
        UploadFrame(frameTex, this, presentWidth, presentHeight);

        BeginDrawing();
        ClearBackground(BLACK);
        const float srcWidth = static_cast<float>(std::max<uint32_t>(1u, presentWidth));
        const float srcHeight = static_cast<float>(std::max<uint32_t>(1u, presentHeight));
        const float screenWidth = static_cast<float>(GetScreenWidth());
        const float screenHeight = static_cast<float>(GetScreenHeight());
        const float scale = std::min(screenWidth / srcWidth, screenHeight / srcHeight);
        const float dstWidth = srcWidth * scale;
        const float dstHeight = srcHeight * scale;
        const Rectangle srcRect{0.0f, 0.0f, srcWidth, srcHeight};
        const Rectangle dstRect{
            (screenWidth - dstWidth) * 0.5f,
            (screenHeight - dstHeight) * 0.5f,
            dstWidth,
            dstHeight};
        DrawTexturePro(frameTex, srcRect, dstRect, Vector2{0.0f, 0.0f}, 0.0f, WHITE);
        if (m_debugUiInitialized && m_debugUiDrawCallback)
        {
            m_debugUiDrawCallback(*this, m_debugUiUserData);
        }
        EndDrawing();

        // Host-driven pad autopoll: the real SIO2 refreshes pad buffers
        // every vsync, but the emulated VBlank event does not fire for all
        // games yet. Poll at display rate so the guest pad area never goes
        // stale while the window loop runs.
        {
            static auto lastPoll = std::chrono::steady_clock::now();
            const auto now = std::chrono::steady_clock::now();
            if (now - lastPoll >= std::chrono::milliseconds(16))
            {
                lastPoll = now;
                notifyIopVBlank();
            }
        }

        if (WindowShouldClose())
        {
            RUNTIME_LOG("[run] window close requested, breaking out of loop");
            requestStop();
            break;
        }
    }

    requestStop();
    if (gameThread.joinable())
    {
        gameThread.join();
    }

    if (m_debugUiInitialized && m_debugUiShutdownCallback)
    {
        m_debugUiShutdownCallback(*this, m_debugUiUserData);
        m_debugUiInitialized = false;
    }
    UnloadTexture(frameTex);
    CloseWindow();

    RUNTIME_LOG("[run] exiting loop");
}
