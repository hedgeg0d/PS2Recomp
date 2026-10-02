#include "runtime/ps2_memory.h"
#include "runtime/ps2_address.h"
#include "runtime/gs/gs_frontend.h"
#include "ps2_log.h"
#include <atomic>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>

namespace
{
    inline void inRange(uint32_t offset, size_t bytes, size_t regionSize, const char *op, uint32_t address)
    {
        if (static_cast<uint64_t>(offset) + static_cast<uint64_t>(bytes) > static_cast<uint64_t>(regionSize))
        {
            throw std::runtime_error(std::string(op) + " out-of-bounds at address: 0x" + std::to_string(address));
        }
    }

    template <typename T>
    inline T loadScalar(const uint8_t *base, uint32_t offset, size_t regionSize, const char *op, uint32_t address)
    {
        inRange(offset, sizeof(T), regionSize, op, address);
        T value{};
        std::memcpy(&value, base + offset, sizeof(T));
        return value;
    }

    template <typename T>
    inline void storeScalar(uint8_t *base, uint32_t offset, size_t regionSize, T value, const char *op, uint32_t address)
    {
        inRange(offset, sizeof(T), regionSize, op, address);
        std::memcpy(base + offset, &value, sizeof(T));
    }

    inline bool isGsPrivReg(uint32_t addr)
    {
        return Ps2AddressInRange(addr, PS2_GS_PRIV_REG_BASE, PS2_GS_PRIV_REG_SIZE);
    }

    inline bool isIoRegister(uint32_t addr)
    {
        return Ps2AddressInRange(addr, PS2_IO_BASE, PS2_IO_SIZE);
    }

    inline uint64_t *gsRegPtr(GSRegisters &gs, uint32_t addr)
    {
        // Support both 64-bit base offsets and +4 dword aliases.
        uint32_t off = (addr - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        switch (off)
        {
        case 0x0000:
            return &gs.pmode;
        case 0x0010:
            return &gs.smode1;
        case 0x0020:
            return &gs.smode2;
        case 0x0030:
            return &gs.srfsh;
        case 0x0040:
            return &gs.synch1;
        case 0x0050:
            return &gs.synch2;
        case 0x0060:
            return &gs.syncv;
        case 0x0070:
            return &gs.dispfb1;
        case 0x0080:
            return &gs.display1;
        case 0x0090:
            return &gs.dispfb2;
        case 0x00A0:
            return &gs.display2;
        case 0x00B0:
            return &gs.extbuf;
        case 0x00C0:
            return &gs.extdata;
        case 0x00D0:
            return &gs.extwrite;
        case 0x00E0:
            return &gs.bgcolor;
        // CSR (offset 0x1000) is intentionally not handled here: it is
        // std::atomic<uint64_t> and no longer converts to uint64_t*. Callers must
        // check for offset 0x1000 themselves and go through writeCsrHalf/
        // writeCsrFull/gs.csr.load() instead of gsRegPtr().
        case 0x1010:
            return &gs.imr;
        case 0x1040:
            return &gs.busdir;
        case 0x1080:
            return &gs.siglblid;
        default:
            return nullptr;
        }
    }

    constexpr uint32_t kGsCsrRegOffset = 0x1000u;

    // Atomically apply a 32-bit write to one half (off=0 low dword, off=4 high
    // dword) of the GS CSR register. Bits 0..1 of the low dword (SIGNAL/FINISH) are
    // write-one-to-clear; everything else is a plain merge. Uses compare_exchange
    // so the whole read-modify-write is a single atomic step -- this register is
    // also touched by the vsync worker (FIELD bit) and the GIF (SIGNAL/FINISH) on
    // other threads, so a load-then-store here would race with them.
    inline void writeCsrHalf(std::atomic<uint64_t> &csr, uint32_t off, uint32_t value)
    {
        constexpr uint32_t kW1cMask = 0x3u;
        uint64_t expected = csr.load();
        uint64_t desired;
        do
        {
            if (off == 0u)
            {
                uint32_t oldLow = static_cast<uint32_t>(expected & 0xFFFFFFFFull);
                uint32_t mergedLow = (oldLow & kW1cMask) | (value & ~kW1cMask);
                desired = (expected & 0xFFFFFFFF00000000ull) | static_cast<uint64_t>(mergedLow);
                desired &= ~static_cast<uint64_t>(value & kW1cMask);
            }
            else
            {
                uint64_t mask = 0xFFFFFFFFull << (off * 8u);
                desired = (expected & ~mask) | (static_cast<uint64_t>(value) << (off * 8u));
            }
        } while (!csr.compare_exchange_weak(expected, desired));
    }

    // Same as writeCsrHalf but for a full 64-bit CSR write (bits 0..1 are still
    // write-one-to-clear against the current value).
    inline void writeCsrFull(std::atomic<uint64_t> &csr, uint64_t value)
    {
        constexpr uint64_t kW1cMask = 0x3ull;
        uint64_t expected = csr.load();
        uint64_t desired;
        do
        {
            desired = (expected & kW1cMask) | (value & ~kW1cMask);
            desired &= ~(value & kW1cMask);
        } while (!csr.compare_exchange_weak(expected, desired));
    }

    constexpr std::array<uint32_t, 4> kEeTimerBases = {
        0x10000000u,
        0x10000800u,
        0x10001000u,
        0x10001800u,
    };
    constexpr uint32_t kEeTimerCountOffset = 0x00u;
    constexpr uint32_t kEeTimerModeOffset = 0x10u;
    constexpr uint32_t kEeTimerCompareOffset = 0x20u;
    constexpr uint32_t kEeTimerHoldOffset = 0x30u;
    constexpr uint32_t kEeTimerModeClksMask = 0x3u;
    constexpr uint32_t kEeTimerModeConfigMask = 0x3FFu;
    constexpr uint32_t kEeTimerModeStatusMask = 0xC00u;
    constexpr uint32_t kEeTimerModeZret = 1u << 6;
    constexpr uint32_t kEeTimerModeCue = 1u << 7;
    constexpr uint32_t kEeTimerModeCmpe = 1u << 8;
    constexpr uint32_t kEeTimerModeOvfe = 1u << 9;
    constexpr uint32_t kEeTimerModeEquf = 1u << 10;
    constexpr uint32_t kEeTimerModeOvff = 1u << 11;
    constexpr uint64_t kEeClockHz = 294912000ull;
    constexpr std::array<uint64_t, 4> kEeTimerClockHz = {
        147456000ull,
        9216000ull,
        576000ull,
        15734ull,
    };

    inline bool decodeEeTimerRegister(uint32_t address, size_t &timerIndex, uint32_t &offset)
    {
        for (size_t index = 0; index < kEeTimerBases.size(); ++index)
        {
            const uint32_t candidateOffset = address - kEeTimerBases[index];
            if (candidateOffset == kEeTimerCountOffset ||
                candidateOffset == kEeTimerModeOffset ||
                candidateOffset == kEeTimerCompareOffset ||
                (index < 2u && candidateOffset == kEeTimerHoldOffset))
            {
                timerIndex = index;
                offset = candidateOffset;
                return true;
            }
        }
        return false;
    }

    constexpr uint64_t ticksUntilMatch(uint32_t count, uint32_t target)
    {
        const uint32_t distance = (target - count) & 0xFFFFu;
        return distance == 0u ? 0x10000ull : static_cast<uint64_t>(distance);
    }

    struct DmaTagView
    {
        uint16_t qwc = 0;
        uint8_t id = 0;
        bool irq = false;
        uint32_t addr = 0;
        uint32_t upper = 0;
    };

    inline DmaTagView decodeDmaTag(uint64_t tag)
    {
        DmaTagView out{};
        out.qwc = static_cast<uint16_t>(tag & 0xFFFFu);
        out.id = static_cast<uint8_t>((tag >> 28u) & 0x7u);
        out.irq = ((tag >> 31u) & 0x1ull) != 0ull;
        out.addr = static_cast<uint32_t>((tag >> 32u) & 0x7FFFFFFFu);
        out.upper = static_cast<uint32_t>((tag >> 16u) & 0xFFFFu);
        return out;
    }

    inline uint32_t gifTagNloop(uint64_t tagLo)
    {
        return static_cast<uint32_t>(tagLo & 0x7FFFu);
    }

    inline uint8_t gifTagFlg(uint64_t tagLo)
    {
        return static_cast<uint8_t>((tagLo >> 58u) & 0x3u);
    }

    inline uint32_t gifTagNreg(uint64_t tagLo)
    {
        uint32_t nreg = static_cast<uint32_t>((tagLo >> 60u) & 0xFu);
        return nreg == 0u ? 16u : nreg;
    }

}

// Helpers for GS VRAM addressing (PSMCT32 path).
static inline uint32_t gs_vram_offset(uint32_t basePage, uint32_t x, uint32_t y, uint32_t fbw)
{
    // basePage is in 2048-byte units; fbw is in blocks of 64 pixels.
    uint32_t strideBytes = fbw * 64 * 4;
    return basePage * 2048 + y * strideBytes + x * 4;
}

PS2Memory::PS2Memory()
    : m_rdram(nullptr), m_scratchpad(nullptr), iop_ram(nullptr), m_seenGifCopy(false), m_gsVRAM(nullptr)
{
    ps2SetScratchpadHostPtr(nullptr);
}

PS2Memory::~PS2Memory()
{
    if (m_rdram)
    {
        delete[] m_rdram;
        m_rdram = nullptr;
    }

    if (m_scratchpad)
    {
        ps2SetScratchpadHostPtr(nullptr);
        delete[] m_scratchpad;
        m_scratchpad = nullptr;
    }

    if (m_gsVRAM)
    {
        delete[] m_gsVRAM;
        m_gsVRAM = nullptr;
    }

    if (m_vu1Code)
    {
        delete[] m_vu1Code;
        m_vu1Code = nullptr;
    }
    if (m_vu1Data)
    {
        delete[] m_vu1Data;
        m_vu1Data = nullptr;
    }
    if (m_vu0Code)
    {
        delete[] m_vu0Code;
        m_vu0Code = nullptr;
    }
    if (m_vu0Data)
    {
        delete[] m_vu0Data;
        m_vu0Data = nullptr;
    }

    if (iop_ram)
    {
        delete[] iop_ram;
        iop_ram = nullptr;
    }
}

bool PS2Memory::initialize(size_t ramSize)
{
    auto cleanup = [this]()
    {
        delete[] m_rdram;
        delete[] m_scratchpad;
        delete[] iop_ram;
        delete[] m_gsVRAM;
        delete[] m_vu0Code;
        delete[] m_vu0Data;
        delete[] m_vu1Code;
        delete[] m_vu1Data;
        m_rdram = nullptr;
        m_scratchpad = nullptr;
        ps2SetScratchpadHostPtr(nullptr);
        iop_ram = nullptr;
        m_gsVRAM = nullptr;
        m_vu0Code = nullptr;
        m_vu0Data = nullptr;
        m_vu1Code = nullptr;
        m_vu1Data = nullptr;
    };

    cleanup();
    m_seenGifCopy = false;
    m_dmaStartCount.store(0, std::memory_order_relaxed);
    m_gifCopyCount.store(0, std::memory_order_relaxed);
    m_gsWriteCount.store(0, std::memory_order_relaxed);
    m_vifWriteCount.store(0, std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lock(m_completedDmacMutex);
        m_completedDmacCauses.clear();
    }
    m_codeRegions.clear();
    m_path3Masked = false;
    m_path3MaskedFifo.clear();
    m_vif1PendingPath2ImageQwc = 0u;
    m_vif1PendingPath2DirectHl = false;
    resetEeTimers();

    try
    {
        // Allocate main RAM
        m_rdram = new uint8_t[ramSize];
        std::memset(m_rdram, 0, ramSize);

        // Allocate scratchpad
        m_scratchpad = new uint8_t[PS2_SCRATCHPAD_SIZE];
        std::memset(m_scratchpad, 0, PS2_SCRATCHPAD_SIZE);
        ps2SetScratchpadHostPtr(m_scratchpad);

        // Initialize EE TLB entries (R5900 has 48 entries).
        m_tlbEntries.assign(48, TLBEntry{0, 0, 0, false});

        // Allocate IOP RAM
        iop_ram = new uint8_t[2 * 1024 * 1024]; // 2MB

        // Initialize IOP RAM with zeros
        std::memset(iop_ram, 0, 2 * 1024 * 1024);

        // Initialize I/O registers
        m_ioRegisters.clear();

        // Initialize GS registers
        memset(&gs_regs, 0, sizeof(gs_regs));
        // memset zero-fills std::atomic<uint64_t>::csr's bytes, which is not itself
        // a guaranteed-valid atomic store; make the zero-initialization explicit.
        gs_regs.csr.store(0);
        gs_regs.dispfb1 = (0ULL << 0) | (10ULL << 9) | (0ULL << 15) | (0ULL << 32) | (0ULL << 43);
        gs_regs.display1 = (0ULL << 0) | (0ULL << 12) | (0ULL << 23) | (0ULL << 27) | (639ULL << 32) | (447ULL << 44);
        gs_regs.dispfb2 = gs_regs.dispfb1;
        gs_regs.display2 = gs_regs.display1;

        // Allocate GS VRAM (4MB)
        m_gsVRAM = new uint8_t[PS2_GS_VRAM_SIZE];
        std::memset(m_gsVRAM, 0, PS2_GS_VRAM_SIZE);

        m_vu0Code = new uint8_t[PS2_VU0_CODE_SIZE];
        m_vu0Data = new uint8_t[PS2_VU0_DATA_SIZE];
        std::memset(m_vu0Code, 0, PS2_VU0_CODE_SIZE);
        std::memset(m_vu0Data, 0, PS2_VU0_DATA_SIZE);

        m_vu1Code = new uint8_t[PS2_VU1_CODE_SIZE];
        m_vu1Data = new uint8_t[PS2_VU1_DATA_SIZE];
        std::memset(m_vu1Code, 0, PS2_VU1_CODE_SIZE);
        std::memset(m_vu1Data, 0, PS2_VU1_DATA_SIZE);
        markVU0CodeModified();
        markVU1CodeModified();

        // Initialize VIF registers
        memset(&vif0_regs, 0, sizeof(vif0_regs));
        memset(&vif1_regs, 0, sizeof(vif1_regs));

        // Initialize DMA registers
        memset(dma_regs, 0, sizeof(dma_regs));

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error initializing PS2 memory: " << e.what() << std::endl;
        cleanup();
        return false;
    }
}

void PS2Memory::resetEeTimers() noexcept
{
    m_eeTimers = {};
}

uint32_t PS2Memory::advanceEeTimers(uint64_t eeCycles) noexcept
{
    if (eeCycles == 0u)
    {
        return 0u;
    }

    constexpr uint32_t kGifStat = 0x10003020u;
    constexpr uint32_t kGifFqcMask = 0x1F000000u;
    auto gifStatIt = m_ioRegisters.find(kGifStat);
    if (gifStatIt != m_ioRegisters.end())
        gifStatIt->second &= ~kGifFqcMask;

    uint32_t interruptMask = 0u;
    for (size_t index = 0; index < m_eeTimers.size(); ++index)
    {
        EeTimer &timer = m_eeTimers[index];
        if ((timer.mode & kEeTimerModeCue) == 0u)
        {
            continue;
        }

        const uint64_t clockHz = kEeTimerClockHz[timer.mode & kEeTimerModeClksMask];
        const uint64_t wholeSeconds = eeCycles / kEeClockHz;
        const uint64_t remainingCycles = eeCycles % kEeClockHz;
        const uint64_t scaled = remainingCycles * clockHz + timer.clockRemainder;
        const uint64_t ticks = wholeSeconds * clockHz + scaled / kEeClockHz;
        timer.clockRemainder = scaled % kEeClockHz;
        if (ticks == 0u)
        {
            continue;
        }

        const uint32_t oldCount = timer.count & 0xFFFFu;
        const uint32_t compare = timer.compare & 0xFFFFu;
        const uint64_t compareDistance = ticksUntilMatch(oldCount, compare);
        const uint64_t overflowDistance = 0x10000ull - oldCount;
        const bool zeroReturn = (timer.mode & kEeTimerModeZret) != 0u;
        const bool compareReached = ticks >= compareDistance;
        bool overflowReached = false;

        if (zeroReturn)
        {
            overflowReached = ticks >= overflowDistance && overflowDistance <= compareDistance;
            if (compareReached)
            {
                const uint64_t remaining = ticks - compareDistance;
                timer.count = compare == 0u
                                  ? static_cast<uint32_t>(remaining & 0xFFFFu)
                                  : static_cast<uint32_t>(remaining % compare);
            }
            else
            {
                timer.count = static_cast<uint32_t>((oldCount + ticks) & 0xFFFFu);
            }
        }
        else
        {
            overflowReached = ticks >= overflowDistance;
            timer.count = static_cast<uint32_t>((oldCount + ticks) & 0xFFFFu);
        }

        if (compareReached && (timer.mode & kEeTimerModeCmpe) != 0u && (timer.mode & kEeTimerModeEquf) == 0u)
        {
            timer.mode |= kEeTimerModeEquf;
            interruptMask |= 1u << index;
        }
        if (overflowReached && (timer.mode & kEeTimerModeOvfe) != 0u && (timer.mode & kEeTimerModeOvff) == 0u)
        {
            timer.mode |= kEeTimerModeOvff;
            interruptMask |= 1u << index;
        }
    }
    return interruptMask;
}

uint64_t PS2Memory::cyclesUntilNextEeTimerInterrupt() const noexcept
{
    uint64_t nearest = std::numeric_limits<uint64_t>::max();
    for (const EeTimer &timer : m_eeTimers)
    {
        if ((timer.mode & kEeTimerModeCue) == 0u)
        {
            continue;
        }

        const uint32_t count = timer.count & 0xFFFFu;
        const uint32_t compare = timer.compare & 0xFFFFu;
        const uint64_t compareDistance = ticksUntilMatch(count, compare);
        const uint64_t overflowDistance = 0x10000ull - count;
        uint64_t eventTicks = std::numeric_limits<uint64_t>::max();

        if ((timer.mode & kEeTimerModeCmpe) != 0u &&
            (timer.mode & kEeTimerModeEquf) == 0u)
        {
            eventTicks = compareDistance;
        }
        const bool overflowCanOccur = (timer.mode & kEeTimerModeZret) == 0u ||
                                      overflowDistance <= compareDistance;
        if (overflowCanOccur &&
            (timer.mode & kEeTimerModeOvfe) != 0u &&
            (timer.mode & kEeTimerModeOvff) == 0u)
        {
            eventTicks = std::min(eventTicks, overflowDistance);
        }
        if (eventTicks == std::numeric_limits<uint64_t>::max())
        {
            continue;
        }

        const uint64_t clockHz = kEeTimerClockHz[timer.mode & kEeTimerModeClksMask];
        const uint64_t numerator = eventTicks * kEeClockHz - timer.clockRemainder;
        const uint64_t cycles = (numerator + clockHz - 1u) / clockHz;
        nearest = std::min(nearest, std::max<uint64_t>(1u, cycles));
    }
    return nearest;
}

bool PS2Memory::isScratchpad(uint32_t address) const
{
    return ps2IsScratchpadAddress(address);
}

uint8_t *PS2Memory::mapVuMemory(uint32_t physAddr, uint32_t size, uint32_t &offset, uint32_t &limit)
{
    return const_cast<uint8_t *>(static_cast<const PS2Memory *>(this)->mapVuMemory(physAddr, size, offset, limit));
}

const uint8_t *PS2Memory::mapVuMemory(uint32_t physAddr, uint32_t size, uint32_t &offset, uint32_t &limit) const
{
    auto mapRange = [&](uint32_t base, uint32_t rangeSize, const uint8_t *ptr) -> const uint8_t *
    {
        if (!ptr || physAddr < base)
        {
            return nullptr;
        }
        const uint32_t local = physAddr - base;
        if (local >= rangeSize || size > (rangeSize - local))
        {
            return nullptr;
        }
        offset = local;
        limit = rangeSize;
        return ptr;
    };

    if (const uint8_t *ptr = mapRange(PS2_VU0_CODE_BASE, PS2_VU0_CODE_SIZE, m_vu0Code))
    {
        return ptr;
    }
    if (const uint8_t *ptr = mapRange(PS2_VU0_DATA_BASE, PS2_VU0_DATA_SIZE, m_vu0Data))
    {
        return ptr;
    }
    if (const uint8_t *ptr = mapRange(PS2_VU1_CODE_BASE, PS2_VU1_CODE_SIZE, m_vu1Code))
    {
        return ptr;
    }
    return mapRange(PS2_VU1_DATA_BASE, PS2_VU1_DATA_SIZE, m_vu1Data);
}

uint32_t PS2Memory::translateAddress(uint32_t virtualAddress)
{
    if (isScratchpad(virtualAddress))
    {
        return ps2ScratchpadOffset(virtualAddress);
    }

    // EE uncached aliases of main RAM (per PS2 memory map):
    //   0x20000000-0x3FFFFFFF -> 32MB mirror of RDRAM
    // This includes the accelerated window rooted at 0x30100000.
    if (Ps2IsUncachedRamMirrorAddress(virtualAddress))
    {
        return virtualAddress & PS2_RAM_MASK;
    }

    // KSEG0/KSEG1 direct-mapped window.
    if (Ps2IsKseg01Address(virtualAddress))
    {
        return Ps2DirectMappedPhysicalAddress(virtualAddress);
    }

    // In this runtime, low segments are treated as physical-style addresses already.
    if (virtualAddress < 0x80000000)
    {
        return virtualAddress;
    }

    // KSEG2/KSEG3 are TLB mapped.
    if (Ps2IsKseg23Address(virtualAddress))
    {
        for (const auto &entry : m_tlbEntries)
        {
            if (entry.valid)
            {
                // PageMask uses bits [24:13]. Build an address-level mask (plus 4KB base page bits).
                const uint32_t mask = entry.mask & 0x01FFE000u;
                const uint32_t compareMask = ~(mask | 0xFFFu);
                if ((virtualAddress & compareMask) == (entry.vpn & compareMask))
                {
                    // TLB hit
                    const uint32_t pageOffsetMask = mask | 0xFFFu;
                    const uint32_t physBase = entry.pfn << 12;
                    return physBase | (virtualAddress & pageOffsetMask);
                }
            }
        }
        throw std::runtime_error("TLB miss for address: 0x" + std::to_string(virtualAddress));
    }

    return virtualAddress;
}

bool PS2Memory::tlbRead(uint32_t index, uint32_t &vpn, uint32_t &pfn, uint32_t &mask, bool &valid) const
{
    if (index >= m_tlbEntries.size())
    {
        return false;
    }

    const TLBEntry &entry = m_tlbEntries[index];
    vpn = entry.vpn;
    pfn = entry.pfn;
    mask = entry.mask;
    valid = entry.valid;
    return true;
}

bool PS2Memory::tlbWrite(uint32_t index, uint32_t vpn, uint32_t pfn, uint32_t mask, bool valid)
{
    if (index >= m_tlbEntries.size())
    {
        return false;
    }

    TLBEntry &entry = m_tlbEntries[index];
    entry.vpn = vpn & 0xFFFFF000u;
    entry.pfn = pfn & 0x000FFFFFu;
    entry.mask = mask & 0x01FFE000u;
    entry.valid = valid;
    return true;
}

int32_t PS2Memory::tlbProbe(uint32_t vpn) const
{
    const uint32_t normalizedVpn = vpn & 0xFFFFF000u;
    for (uint32_t i = 0; i < static_cast<uint32_t>(m_tlbEntries.size()); ++i)
    {
        const TLBEntry &entry = m_tlbEntries[i];
        if (!entry.valid)
        {
            continue;
        }

        const uint32_t mask = entry.mask & 0x01FFE000u;
        const uint32_t compareMask = ~(mask | 0xFFFu);
        if ((normalizedVpn & compareMask) == (entry.vpn & compareMask))
        {
            return static_cast<int32_t>(i);
        }
    }

    return -1;
}

uint8_t PS2Memory::read8(uint32_t address)
{
    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        return m_scratchpad[physAddr];
    }
    if (isIoRegister(physAddr))
    {
        uint32_t regAddr = physAddr & ~0x3;
        uint32_t value = readIORegister(regAddr);
        uint32_t shift = (physAddr & 3) * 8;
        return static_cast<uint8_t>((value >> shift) & 0xFF);
    }
    if (physAddr < PS2_RAM_SIZE)
    {
        return m_rdram[physAddr];
    }
    uint32_t vuOffset = 0;
    uint32_t vuLimit = 0;
    if (const uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint8_t), vuOffset, vuLimit))
    {
        (void)vuLimit;
        return vuMem[vuOffset];
    }

    return 0;
}

uint16_t PS2Memory::read16(uint32_t address)
{
    if (address & 1)
    {
        throw std::runtime_error("Unaligned 16-bit read at address: 0x" + std::to_string(address));
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        return loadScalar<uint16_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, "read16 scratchpad", address);
    }
    if (isIoRegister(physAddr))
    {
        uint32_t regAddr = physAddr & ~0x3;
        uint32_t value = readIORegister(regAddr);
        uint32_t shift = (physAddr & 2) * 8;
        return static_cast<uint16_t>((value >> shift) & 0xFFFF);
    }
    if (physAddr < PS2_RAM_SIZE)
    {
        return loadScalar<uint16_t>(m_rdram, physAddr, PS2_RAM_SIZE, "read16 rdram", address);
    }
    uint32_t vuOffset = 0;
    uint32_t vuLimit = 0;
    if (const uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint16_t), vuOffset, vuLimit))
    {
        return loadScalar<uint16_t>(vuMem, vuOffset, vuLimit, "read16 vu", address);
    }

    return 0;
}

uint32_t PS2Memory::read32(uint32_t address)
{
    if (address & 3)
    {
        throw std::runtime_error("Unaligned 32-bit read at address: 0x" + std::to_string(address));
    }

    if (isGsPrivReg(address))
    {
        uint32_t off = address & 7;
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            uint64_t val = gs_regs.csr.load();
            return (uint32_t)(val >> (off * 8));
        }
        uint64_t *reg = gsRegPtr(gs_regs, address);
        if (!reg)
            return 0;
        uint64_t val = *reg;
        return (uint32_t)(val >> (off * 8));
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        return loadScalar<uint32_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, "read32 scratchpad", address);
    }
    if (isIoRegister(physAddr))
    {
        return readIORegister(physAddr);
    }
    if (physAddr < PS2_RAM_SIZE)
    {
        return loadScalar<uint32_t>(m_rdram, physAddr, PS2_RAM_SIZE, "read32 rdram", address);
    }
    uint32_t vuOffset = 0;
    uint32_t vuLimit = 0;
    if (const uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint32_t), vuOffset, vuLimit))
    {
        return loadScalar<uint32_t>(vuMem, vuOffset, vuLimit, "read32 vu", address);
    }

    return 0;
}

uint64_t PS2Memory::read64(uint32_t address)
{
    if (address & 7)
    {
        throw std::runtime_error("Unaligned 64-bit read at address: 0x" + std::to_string(address));
    }

    if (isGsPrivReg(address))
    {
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            return gs_regs.csr.load();
        }
        uint64_t *reg = gsRegPtr(gs_regs, address);
        return reg ? *reg : 0;
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        return loadScalar<uint64_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, "read64 scratchpad", address);
    }
    // 64-bit IO read: compose from the two adjacent 32-bit IO register slots
    // to avoid any side-effects from read32 handlers.
    if (isIoRegister(physAddr))
    {
        if (physAddr == 0x10002010u || physAddr == 0x10002020u ||
            physAddr == 0x10002030u)
        {
            // These registers are computed from live IPU state. The generic
            // IO mirror is not updated when the decoder advances the stream.
            const uint32_t lo = readIORegister(physAddr);
            const uint32_t hi = physAddr == 0x10002030u && m_ipu.busy
                                    ? 0x80000000u : 0u;
            return static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32u);
        }
        if (physAddr == 0x10002000u)
        {
            // CMD observes the BUSY transient (see writeIpuCommand); the
            // mirror slots stay coherent for the side-effect-free compose.
            observeIpuRead();
            syncIpuCmdMirror();
        }
        size_t timerIndex = 0u;
        uint32_t timerOffset = 0u;
        if (decodeEeTimerRegister(physAddr, timerIndex, timerOffset))
        {
            uint32_t lo = readIORegister(physAddr);
            uint32_t hi = readIORegister(physAddr + 4);
            return static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32);
        }
        uint32_t lo = m_ioRegisters.count(physAddr) ? m_ioRegisters[physAddr] : 0u;
        uint32_t hi = m_ioRegisters.count(physAddr + 4) ? m_ioRegisters[physAddr + 4] : 0u;
        return static_cast<uint64_t>(lo) | (static_cast<uint64_t>(hi) << 32);
    }
    if (physAddr < PS2_RAM_SIZE)
    {
        return loadScalar<uint64_t>(m_rdram, physAddr, PS2_RAM_SIZE, "read64 rdram", address);
    }
    uint32_t vuOffset = 0;
    uint32_t vuLimit = 0;
    if (const uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint64_t), vuOffset, vuLimit))
    {
        return loadScalar<uint64_t>(vuMem, vuOffset, vuLimit, "read64 vu", address);
    }

    return static_cast<uint64_t>(read32(address)) |
           (static_cast<uint64_t>(read32(address + 4u)) << 32u);
}

__m128i PS2Memory::read128(uint32_t address)
{
    if (address & 15)
    {
        throw std::runtime_error("Unaligned 128-bit read at address: 0x" + std::to_string(address));
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        inRange(physAddr, sizeof(__m128i), PS2_SCRATCHPAD_SIZE, "read128 scratchpad", address);
        return _mm_loadu_si128(reinterpret_cast<__m128i *>(&m_scratchpad[physAddr]));
    }
    if (physAddr < PS2_RAM_SIZE)
    {
        inRange(physAddr, sizeof(__m128i), PS2_RAM_SIZE, "read128 rdram", address);
        return _mm_loadu_si128(reinterpret_cast<__m128i *>(&m_rdram[physAddr]));
    }
    uint32_t vuOffset = 0;
    uint32_t vuLimit = 0;
    if (const uint8_t *vuMem = mapVuMemory(physAddr, sizeof(__m128i), vuOffset, vuLimit))
    {
        inRange(vuOffset, sizeof(__m128i), vuLimit, "read128 vu", address);
        return _mm_loadu_si128(reinterpret_cast<const __m128i *>(vuMem + vuOffset));
    }

    // 128-bit reads are primarily for quad-word loads in the EE, which are only valid for RAM areas
    // Return zeroes for unsupported areas
    return _mm_setzero_si128();
}

void PS2Memory::write8(uint32_t address, uint8_t value)
{
    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        m_scratchpad[physAddr] = value;
    }
    else if (physAddr < PS2_RAM_SIZE)
    {
        m_rdram[physAddr] = value;
    }
    else
    {
        uint32_t vuOffset = 0;
        uint32_t vuLimit = 0;
        if (uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint8_t), vuOffset, vuLimit))
        {
            (void)vuLimit;
            vuMem[vuOffset] = value;
            if (vuMem == m_vu0Code)
                markVU0CodeModified();
            else if (vuMem == m_vu1Code)
                markVU1CodeModified();
            return;
        }
    }
    if (isIoRegister(physAddr))
    {
        // IO registers - handle byte writes by modifying the appropriate byte in the word
        uint32_t regAddr = physAddr & ~0x3;
        uint32_t shift = (physAddr & 3) * 8;
        uint32_t mask = ~(0xFF << shift);
        uint32_t newValue = (m_ioRegisters[regAddr] & mask) | ((uint32_t)value << shift);
        writeIORegister(regAddr, newValue);
    }
}

void PS2Memory::write16(uint32_t address, uint16_t value)
{
    if (address & 1)
    {
        throw std::runtime_error("Unaligned 16-bit write at address: 0x" + std::to_string(address));
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        storeScalar<uint16_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, value, "write16 scratchpad", address);
    }
    else if (physAddr < PS2_RAM_SIZE)
    {
        storeScalar<uint16_t>(m_rdram, physAddr, PS2_RAM_SIZE, value, "write16 rdram", address);
    }
    else
    {
        uint32_t vuOffset = 0;
        uint32_t vuLimit = 0;
        if (uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint16_t), vuOffset, vuLimit))
        {
            storeScalar<uint16_t>(vuMem, vuOffset, vuLimit, value, "write16 vu", address);
            if (vuMem == m_vu0Code)
                markVU0CodeModified();
            else if (vuMem == m_vu1Code)
                markVU1CodeModified();
            return;
        }
    }
    if (isIoRegister(physAddr))
    {
        uint32_t regAddr = physAddr & ~0x3;
        uint32_t shift = (physAddr & 2) * 8;
        uint32_t mask = ~(0xFFFF << shift);
        uint32_t newValue = (m_ioRegisters[regAddr] & mask) | ((uint32_t)value << shift);
        writeIORegister(regAddr, newValue);
    }
}

void PS2Memory::syncIpuCmdMirror()
{
    // 64-bit CMD reads compose {BUSY, DATA} from these slots without
    // side effects, so keep them coherent on every state change.
    m_ioRegisters[0x10002000u] = m_ipu.cmdData;
    m_ioRegisters[0x10002004u] = m_ipu.busy ? 0x80000000u : 0u;
}

uint32_t PS2Memory::readIpuCtrl() const
{
    uint32_t ifc = static_cast<uint32_t>(m_ipu.inFifo.size() / 4u);
    uint32_t ofc = static_cast<uint32_t>(m_ipu.outFifo.size() / 4u);
    if (ifc > 8u)
        ifc = 8u;
    if (ofc > 8u)
        ofc = 8u;
    uint32_t val = m_ipu.ctrlKeep | (ifc << 0) | (ofc << 4) |
                   (static_cast<uint32_t>(m_ipu.codedBlockPattern) << 8u);
    if (m_ipu.scd)
        val |= (1u << 15);
    if (m_ipu.busy)
        val |= (1u << 31);
    return val;
}

uint32_t PS2Memory::readIpuBp() const
{
    uint32_t ifc = static_cast<uint32_t>(m_ipu.inFifo.size() / 4u);
    if (ifc > 8u)
        ifc = 8u;
    // Consumed words are removed individually, but DMA advances by whole
    // quadwords. Account for the remaining partial quadword as the internal
    // bitstream buffer (FP), otherwise guest save/restore loses 16 bytes.
    const uint32_t fp = (m_ipu.inFifo.size() % 4u) != 0u ? 1u : 0u;
    return (m_ipu.bitPos & 0x7Fu) | (ifc << 8) | (fp << 16);
}

void PS2Memory::observeIpuRead()
{
    // Any IPU register read observes the BUSY transient (see
    // writeIpuCommand); the worker delay is discretized to polls.
    if (m_ipu.busy && ++m_ipu.busyPolls >= 2u)
    {
        m_ipu.busy = false;
        syncIpuCmdMirror();
    }
}

void PS2Memory::writeIpuCommand(uint32_t value)
{
    static std::atomic<uint32_t> bdecCount{0u};
    static std::atomic<uint32_t> fdecCount{0u};
    static std::atomic<uint32_t> vdecCount{0u};
    static std::atomic<uint32_t> commandCount{0u};
    const uint32_t cmd = (value >> 28) & 0xFu;
    const uint32_t commandIndex = commandCount.fetch_add(1u, std::memory_order_relaxed);
    if (commandIndex < 10000u)
    {
        std::cerr << "[probe:ipu-command] n=" << commandIndex
                  << " cmd=0x" << std::hex << cmd
                  << " value=0x" << value
                  << " bit=0x" << m_ipu.bitPos
                  << " frag=0x" << m_ipu.fragBits
                  << std::dec << '\n';
    }
    m_ipu.cmdData = value;
    m_ipu.scd = false;
    m_ipu.ctrlKeep &= ~(1u << 14); // ECD is cleared by every new command.
    // Data commands (IDEC/BDEC/VDEC/FDEC) run through the BUSY transient so
    // the guest observes BUSY once and runs its wait/feed path; setup
    // commands complete instantly (hardware/PCSX2 behavior).
    const bool dataCmd = cmd == 0x1u || cmd == 0x2u || cmd == 0x3u || cmd == 0x4u;
    m_ipu.busy = dataCmd;
    m_ipu.busyPolls = 0u;
    syncIpuCmdMirror();

    auto refillInput = [this]() {
        if ((m_ioRegisters[0x1000B400u] & 0x100u) != 0u)
            runIpuInDma(0x1000B400u);
    };
    auto normalizeInput = [this, &refillInput]() {
        refillInput();
        while (m_ipu.fragBits >= 32u && !m_ipu.inFifo.empty())
        {
            m_ipu.inFifo.pop_front();
            m_ipu.fragBits -= 32u;
            refillInput();
        }
        return m_ipu.fragBits < 32u;
    };
    auto advanceInput = [this, &refillInput, &normalizeInput](uint32_t bits) {
        if (!normalizeInput())
            return false;
        while (bits != 0u)
        {
            if (m_ipu.inFifo.empty())
            {
                refillInput();
                if (m_ipu.inFifo.empty())
                    return false;
            }
            const uint32_t avail = 32u - m_ipu.fragBits;
            const uint32_t take = bits < avail ? bits : avail;
            m_ipu.fragBits += take;
            m_ipu.bitPos += take;
            bits -= take;
            if (m_ipu.fragBits == 32u)
            {
                m_ipu.inFifo.pop_front();
                m_ipu.fragBits = 0u;
                refillInput();
            }
        }
        return true;
    };
    auto peekInput = [this, &normalizeInput](uint32_t bits, uint32_t &result) {
        if (bits > 32u || !normalizeInput())
            return false;
        result = 0u;
        for (uint32_t bit = 0u; bit < bits; ++bit)
        {
            const uint32_t absolute = m_ipu.fragBits + bit;
            const size_t wordIndex = absolute / 32u;
            if (wordIndex >= m_ipu.inFifo.size())
                return false;
            const uint32_t withinWord = absolute % 32u;
            const uint32_t byteIndex = withinWord / 8u;
            const uint32_t bitInByte = withinWord % 8u;
            const uint8_t byte = static_cast<uint8_t>(m_ipu.inFifo[wordIndex] >> (byteIndex * 8u));
            result = (result << 1u) | ((byte >> (7u - bitInByte)) & 1u);
        }
        return true;
    };

    switch (cmd)
    {
    case 0x0u: // BCLR: clear input FIFO and set the bit pointer (0..127).
        m_ipu.inFifo.clear();
        m_ipu.outFifo.clear();
        m_ipu.bitPos = value & 0x7Fu;
        // Values above 31 skip complete words after DMA refills the FIFO.
        m_ipu.fragBits = value & 0x7Fu;
        m_ipu.bdecWordsLeft = 0u;
        m_ipu.bdecOutputPos = 0u;
        m_ipu.codedBlockPattern = 0u;
        m_ipu.topData = 0u;
        m_ipu.busy = false;
        break;
    case 0x4u: // FDEC: skip bits, report a start code at the new position.
    {
        advanceInput(value & 0x3Fu);
        refillInput();
        // FDEC returns the next 32 MPEG bits in stream order. FIFO words are
        // little-endian host words, while bits inside each stream byte are
        // consumed MSB first.
        uint32_t decoded = 0u;
        bool haveDecoded = true;
        for (uint32_t bit = 0u; bit < 32u; ++bit)
        {
            const uint32_t absolute = m_ipu.fragBits + bit;
            const size_t wordIndex = absolute / 32u;
            if (wordIndex >= m_ipu.inFifo.size())
            {
                haveDecoded = false;
                break;
            }
            const uint32_t withinWord = absolute % 32u;
            const uint32_t byteIndex = withinWord / 8u;
            const uint32_t bitInByte = withinWord % 8u;
            const uint8_t byte = static_cast<uint8_t>(m_ipu.inFifo[wordIndex] >> (byteIndex * 8u));
            decoded = (decoded << 1u) | ((byte >> (7u - bitInByte)) & 1u);
        }
        if (haveDecoded)
        {
            m_ipu.cmdData = decoded;
            m_ipu.topData = decoded;
            // Some direct-IPU MPEG consumers use the status bit as the
            // boundary notification after FDEC rather than after BDEC. Keep
            // this opt-in until validated against more streams.
            const char *fdecEcd = std::getenv("PS2X_IPU_FDEC_ECD");
            if (fdecEcd != nullptr && fdecEcd[0] != '\0' &&
                m_ipu.sawSliceCode &&
                (decoded & 0xFFFFFF00u) == 0x00000100u)
            {
                m_ipu.ctrlKeep |= 1u << 14u;
            }
            if ((decoded & 0xFFFFFF00u) == 0x00000100u &&
                (decoded & 0xFFu) >= 0x01u && (decoded & 0xFFu) <= 0xAFu)
                m_ipu.sawSliceCode = true;
        }

        // Keep a byte-oriented diagnostic view of the same position.
        uint8_t peek[7] = {0u, 0u, 0u, 0u, 0u, 0u, 0u};
        uint32_t peekBytes = 0u;
        {
            uint64_t acc = 0u;
            uint32_t accBits = 0u;
            size_t wi = 0u;
            uint32_t frag = m_ipu.fragBits;
            while (peekBytes < sizeof(peek) && wi < m_ipu.inFifo.size())
            {
                acc |= static_cast<uint64_t>(m_ipu.inFifo[wi] >> frag) << accBits;
                const uint32_t got = 32u - frag;
                accBits += got;
                ++wi;
                frag = 0u;
                while (accBits >= 8u && peekBytes < sizeof(peek))
                {
                    peek[peekBytes++] = static_cast<uint8_t>(acc & 0xFFu);
                    acc >>= 8;
                    accBits -= 8u;
                }
            }
        }
        const uint32_t n = fdecCount.fetch_add(1u, std::memory_order_relaxed);
        if (n < 2000u)
        {
            std::cerr << "[probe:ipu-fdec] n=" << n
                      << " bit=0x" << std::hex << m_ipu.bitPos
                      << " skip=0x" << std::hex << (value & 0x3Fu)
                      << " scd=0x" << (m_ipu.scd ? 1u : 0u)
                      << " data=0x" << m_ipu.cmdData
                      << " inQw=0x" << m_ipu.inFifo.size() / 4u
                      << " peek=0x" << peekBytes
                      << std::dec << '\n';
        }
        break;
    }
    case 0x2u: // BDEC: decode one macroblock.
    {
        // BDEC returns planar 16-bit YCbCr: 256 Y + 64 Cb + 64 Cr samples.
        // Unlike frame-level FFmpeg HLE, this also advances the exact MPEG VLC
        // bit count and preserves non-intra residual samples for guest motion
        // compensation.
        const uint32_t commandStartBit = m_ipu.bitPos;
        advanceInput(value & 0x3Fu);
        const uint32_t decodeStartBit = m_ipu.bitPos;

        IpuMpeg2Decoder::BdecConfig config{};
        config.qScaleCode = static_cast<uint8_t>((value >> 16u) & 0x1Fu);
        config.interlacedDct = (value & (1u << 25u)) != 0u;
        config.dcReset = (value & (1u << 26u)) != 0u;
        config.macroblockIntra = (value & (1u << 27u)) != 0u;
        config.intraDcPrecision = static_cast<uint8_t>((m_ipu.ctrlKeep >> 16u) & 0x3u);
        config.alternateScan = (m_ipu.ctrlKeep & (1u << 20u)) != 0u;
        config.intraVlcFormat = (m_ipu.ctrlKeep & (1u << 21u)) != 0u;
        config.nonlinearQScale = (m_ipu.ctrlKeep & (1u << 22u)) != 0u;
        config.mpeg1 = (m_ipu.ctrlKeep & (1u << 23u)) != 0u;

        std::array<uint16_t, IpuMpeg2Decoder::kMacroblockSamples> samples{};
        const bool decoded = m_ipu.mpeg2.decodeMacroblock(
            config, peekInput, advanceInput, samples, m_ipu.codedBlockPattern);
        const uint32_t decodeEndBit = m_ipu.bitPos;
        uint32_t nextByte = 0u;
        uint32_t scanPrefix = 0u;
        if (decoded)
        {
            for (uint32_t i = 0u; i < IpuState::kBdecOutWords; ++i)
            {
                m_ipu.bdecOutput[i] = static_cast<uint32_t>(samples[i * 2u]) |
                                      (static_cast<uint32_t>(samples[i * 2u + 1u]) << 16u);
            }
            m_ipu.bdecOutputPos = 0u;
            m_ipu.bdecWordsLeft = IpuState::kBdecOutWords;

            // Hardware aligns and scans zero bytes after a macroblock. Leave
            // the cursor at a 00 00 01 prefix and expose it through SCD/TOP.
            if (peekInput(8u, nextByte) && nextByte == 0u)
            {
                const uint32_t alignBits = (8u - (m_ipu.fragBits & 7u)) & 7u;
                advanceInput(alignBits);
                while (peekInput(24u, scanPrefix) && scanPrefix == 0u)
                    advanceInput(8u);
                if (scanPrefix == 1u)
                    m_ipu.scd = true;
                else if (scanPrefix != 0u)
                    m_ipu.ctrlKeep |= 1u << 14u; // ECD: next start code is not a slice.
            }
            uint32_t top = 0u;
            if (peekInput(32u, top))
            {
                m_ipu.topData = top;
                // BDEC leaves CMD as a bitstream peek; only FDEC/VDEC
                // expose a latched command result instead.
                m_ipu.cmdData = top;
            }

        }
        else
        {
            m_ipu.ctrlKeep |= 1u << 14u;
            m_ipu.bdecOutput.fill(0u);
            m_ipu.bdecOutputPos = 0u;
            m_ipu.bdecWordsLeft = IpuState::kBdecOutWords;
        }
        fillIpuOutFifo();
        const uint32_t n = bdecCount.fetch_add(1u, std::memory_order_relaxed);
        if (n < 2048u)
        {
            const uint32_t nonzero = static_cast<uint32_t>(std::count_if(
                samples.begin(), samples.end(), [](uint16_t sample) { return sample != 0u; }));
            std::cerr << "[probe:ipu-bdec] n=" << n
                      << " cmd=0x" << std::hex << value
                      << " ok=0x" << (decoded ? 1u : 0u)
                      << " samples=" << std::dec << nonzero
                      << " y0=" << samples[0] << " cb0=" << samples[256] << " cr0=" << samples[320]
                      << std::hex
                      << " cbp=0x" << static_cast<uint32_t>(m_ipu.codedBlockPattern)
                      << " begin=0x" << commandStartBit
                      << " decode=0x" << decodeStartBit << "-0x" << decodeEndBit
                      << " end=0x" << m_ipu.bitPos
                      << " scd=0x" << (m_ipu.scd ? 1u : 0u)
                      << " ecd=0x" << ((m_ipu.ctrlKeep >> 14u) & 1u)
                      << " next=0x" << nextByte << " prefix=0x" << scanPrefix
                      << " top=0x" << m_ipu.topData
                      << " inQw=0x" << m_ipu.inFifo.size() / 4u
                      << " outQw=0x" << m_ipu.outFifo.size() / 4u
                      << std::dec << '\n';
        }
        resumeIpuOutDma();
        break;
    }
    case 0x3u: // VDEC: decode a variable-length MPEG field.
    {
        advanceInput(value & 0x3Fu);
        uint32_t code16 = 0u;
        uint32_t result = 0u;
        uint32_t length = 0u;
        const uint32_t table = (value >> 26) & 0x3u;

        // Table 0: macroblock_address_increment (ISO/IEC 13818-2 B-1).
        // The compact ranges below are the expanded hardware lookup table.
        if (table == 0u && peekInput(16u, code16))
        {
            const uint32_t code5 = code16 >> 11u;
            const uint32_t code11 = code16 >> 5u;
            uint32_t increment = 0u;
            if (code5 >= 2u)
            {
                if (code5 >= 16u) { increment = 1u; length = 1u; }
                else if (code5 >= 12u) { increment = 2u; length = 3u; }
                else if (code5 >= 8u) { increment = 3u; length = 3u; }
                else if (code5 >= 6u) { increment = 4u; length = 4u; }
                else if (code5 >= 4u) { increment = 5u; length = 4u; }
                else if (code5 == 3u) { increment = 6u; length = 5u; }
                else { increment = 7u; length = 5u; }
            }
            else if (code11 >= 24u)
            {
                if (code11 <= 35u) { increment = 57u - code11; length = 11u; }
                else if (code11 <= 47u) { increment = 21u - ((code11 - 36u) / 2u); length = 10u; }
                else if (code11 <= 55u) { increment = 15u; length = 8u; }
                else if (code11 <= 63u) { increment = 14u; length = 8u; }
                else if (code11 <= 71u) { increment = 13u; length = 8u; }
                else if (code11 <= 79u) { increment = 12u; length = 8u; }
                else if (code11 <= 87u) { increment = 11u; length = 8u; }
                else if (code11 <= 95u) { increment = 10u; length = 8u; }
                else if (code11 <= 111u) { increment = 9u; length = 7u; }
                else { increment = 8u; length = 7u; }
            }
            else if (code11 == 8u)
            {
                // macroblock_escape
                increment = 0x23u;
                length = 11u;
            }
            else if (code11 == 15u && (m_ipu.ctrlKeep & (1u << 23)) != 0u)
            {
                // MPEG-1 macroblock_stuffing
                increment = 0x22u;
                length = 11u;
            }

            if (length != 0u)
            {
                advanceInput(length);
                result = increment | (length << 16u);
            }
        }
        else if (table == 1u && peekInput(16u, code16))
        {
            // Macroblock type. VDEC uses frame-predicted DCT mode, matching
            // the IPU's standalone VLC operation.
            constexpr uint32_t kIntra = 1u;
            constexpr uint32_t kPattern = 2u;
            constexpr uint32_t kBackward = 4u;
            constexpr uint32_t kForward = 8u;
            constexpr uint32_t kQuant = 16u;
            constexpr uint32_t kMcFrame = 128u;
            const uint32_t pictureType = (m_ipu.ctrlKeep >> 24u) & 0x7u;
            const uint32_t code6 = code16 >> 10u;
            uint32_t modes = 0u;

            if (pictureType <= 1u)
            {
                const uint32_t code2 = code16 >> 14u;
                if (code2 == 1u) { modes = kIntra | kQuant; length = 2u; }
                else if (code2 >= 2u) { modes = kIntra; length = 1u; }
            }
            else if (pictureType == 2u && code6 != 0u)
            {
                if (code6 == 1u) { modes = kIntra | kQuant; length = 6u; }
                else if (code6 <= 3u) { modes = kPattern | kQuant; length = 5u; }
                else if (code6 <= 5u) { modes = kForward | kPattern | kQuant; length = 5u; }
                else if (code6 <= 7u) { modes = kIntra; length = 5u; }
                else if (code6 <= 15u) { modes = kForward; length = 3u; }
                else if (code6 <= 31u) { modes = kPattern; length = 2u; }
                else { modes = kForward | kPattern; length = 1u; }
                if ((modes & kForward) != 0u)
                    modes |= kMcFrame;
            }
            else if (pictureType == 3u && code6 != 0u)
            {
                if (code6 == 1u) { modes = kIntra | kQuant; length = 6u; }
                else if (code6 == 2u) { modes = kBackward | kPattern | kQuant; length = 6u; }
                else if (code6 == 3u) { modes = kForward | kPattern | kQuant; length = 6u; }
                else if (code6 <= 5u) { modes = kForward | kBackward | kPattern | kQuant; length = 5u; }
                else if (code6 <= 7u) { modes = kIntra; length = 5u; }
                else if (code6 <= 11u) { modes = kForward; length = 4u; }
                else if (code6 <= 15u) { modes = kForward | kPattern; length = 4u; }
                else if (code6 <= 23u) { modes = kBackward; length = 3u; }
                else if (code6 <= 31u) { modes = kBackward | kPattern; length = 3u; }
                else if (code6 <= 47u) { modes = kForward | kBackward; length = 2u; }
                else { modes = kForward | kBackward | kPattern; length = 2u; }
                modes |= kMcFrame;
            }

            if (length != 0u)
            {
                advanceInput(length);
                result = modes;
                if (pictureType == 3u)
                    result |= length << 16u;
            }
        }
        else if (table == 2u && peekInput(16u, code16))
        {
            // Motion code (Table B-10).
            if ((code16 & 0x8000u) != 0u)
            {
                advanceInput(1u);
                result = 0x00010000u;
            }
            else
            {
                uint32_t delta = 0u;
                const uint32_t code4 = code16 >> 12u;
                if ((code16 & 0xF000u) != 0u || (code16 & 0xFC00u) == 0x0C00u)
                {
                    if (code4 == 0u) { delta = 3u; length = 6u; }
                    else if (code4 == 1u) { delta = 2u; length = 4u; }
                    else if (code4 <= 3u) { delta = 1u; length = 3u; }
                    else { delta = 0u; length = 2u; }
                }
                else
                {
                    const uint32_t code10 = code16 >> 6u;
                    if (code10 <= 11u) { delta = 0u; length = 10u; }
                    else if (code10 <= 17u) { delta = 27u - code10; length = 10u; }
                    else if (code10 <= 19u) { delta = 9u; length = 9u; }
                    else if (code10 <= 21u) { delta = 8u; length = 9u; }
                    else if (code10 <= 23u) { delta = 7u; length = 9u; }
                    else if (code10 <= 31u) { delta = 6u; length = 7u; }
                    else if (code10 <= 39u) { delta = 5u; length = 7u; }
                    else if (code10 <= 47u) { delta = 4u; length = 7u; }
                }
                if (length != 0u)
                {
                    advanceInput(length);
                    uint32_t sign = 0u;
                    peekInput(1u, sign);
                    advanceInput(1u);
                    const int32_t magnitude = static_cast<int32_t>(delta + 1u);
                    const int32_t signedDelta = sign != 0u ? -magnitude : magnitude;
                    result = static_cast<uint32_t>(signedDelta) | (length << 16u);
                }
            }
        }
        else if (table == 3u)
        {
            // Dual-prime motion vector (Table B-11).
            uint32_t code2 = 0u;
            if (peekInput(2u, code2))
            {
                int32_t dmv = 0;
                length = code2 < 2u ? 1u : 2u;
                if (code2 == 2u) dmv = 1;
                else if (code2 == 3u) dmv = -1;
                advanceInput(length);
                result = static_cast<uint32_t>(dmv) | (length << 16u);
            }
        }

        m_ipu.cmdData = result;
        if (result == 0u)
            m_ipu.ctrlKeep |= 1u << 14;
        uint32_t top = 0u;
        if (peekInput(32u, top))
            m_ipu.topData = top;
        const uint32_t n = vdecCount.fetch_add(1u, std::memory_order_relaxed);
        if (n < 24u)
        {
            std::cerr << "[probe:ipu-vdec] n=" << n
                      << " table=0x" << std::hex << table
                      << " code=0x" << code16
                      << " result=0x" << result
                      << " top=0x" << m_ipu.topData
                      << " bp=0x" << (m_ipu.bitPos & 0x7Fu)
                      << std::dec << '\n';
        }
        break;
    }
    case 0x5u: // SETIQ: skip bits, then consume one 8x8 quantization matrix.
    {
        advanceInput(value & 0x3Fu);
        std::array<uint8_t, 64> matrix{};
        bool complete = true;
        for (uint8_t &entry : matrix)
        {
            uint32_t byte = 0u;
            if (!peekInput(8u, byte) || !advanceInput(8u))
            {
                complete = false;
                break;
            }
            entry = static_cast<uint8_t>(byte);
        }
        if (complete)
            m_ipu.mpeg2.setQuantMatrix((value & (1u << 27u)) == 0u, matrix);
        else
            m_ipu.ctrlKeep |= 1u << 14u;
        m_ipu.busy = false;
        break;
    }
    case 0x7u: // CSC: planar 4:2:0 YCbCr macroblocks to RGB.
    {
        const uint32_t macroblocks = value & 0x7FFu;
        const bool dither = (value & (1u << 26u)) != 0u;
        const bool rgb16 = (value & (1u << 27u)) != 0u;
        std::cerr << "[probe:ipu-csc-start] mb=" << macroblocks
                  << " inQwc=0x" << std::hex << m_ioRegisters[0x1000B420u]
                  << " inChcr=0x" << m_ioRegisters[0x1000B400u]
                  << " inMadr=0x" << m_ioRegisters[0x1000B410u]
                  << " inFirst=0x" << (m_ipu.inFifo.empty() ? 0u : m_ipu.inFifo.front())
                  << " outMadr=0x" << m_ioRegisters[0x1000B010u]
                  << " outQwc=0x" << m_ioRegisters[0x1000B020u]
                  << " outChcr=0x" << m_ioRegisters[0x1000B000u]
                  << " inWords=0x" << m_ipu.inFifo.size()
                  << " outWords=0x" << m_ipu.outFifo.size() << std::dec << '\n';
        bool complete = true;
        uint32_t mb = 0u;
        for (; mb < macroblocks && complete; ++mb)
        {
            std::array<uint8_t, 384> ycbcr{};
            for (uint8_t &sample : ycbcr)
            {
                uint32_t byte = 0u;
                if (!peekInput(8u, byte) || !advanceInput(8u))
                {
                    complete = false;
                    break;
                }
                sample = static_cast<uint8_t>(byte);
            }
            if (!complete)
                break;

            auto emit = [this](uint32_t word) {
                if (m_ipu.outFifo.size() >= IpuState::kFifoWords)
                    resumeIpuOutDma();
                if (m_ipu.outFifo.size() >= IpuState::kFifoWords)
                    return false;
                m_ipu.outFifo.push_back(word);
                resumeIpuOutDma();
                return true;
            };
            for (uint32_t y = 0u; y < 16u && complete; ++y)
            {
                for (uint32_t x = 0u; x < 16u; ++x)
                {
                    const int luma = (0x95 * std::max(0, static_cast<int>(ycbcr[y * 16u + x]) - 16)) >> 6;
                    const int cb = static_cast<int>(ycbcr[256u + (y / 2u) * 8u + x / 2u]) - 128;
                    const int cr = static_cast<int>(ycbcr[320u + (y / 2u) * 8u + x / 2u]) - 128;
                    int r = std::clamp((luma + ((0xCC * cr) >> 6) + 1) >> 1, 0, 255);
                    int g = std::clamp((luma + ((-0x68 * cr) >> 6) + ((-0x32 * cb) >> 6) + 1) >> 1, 0, 255);
                    int b = std::clamp((luma + ((0x102 * cb) >> 6) + 1) >> 1, 0, 255);
                    if (!rgb16)
                    {
                        complete = emit(static_cast<uint32_t>(r) |
                                        (static_cast<uint32_t>(g) << 8u) |
                                        (static_cast<uint32_t>(b) << 16u) | 0x80000000u);
                    }
                    else if ((x & 1u) != 0u)
                    {
                        auto pack16 = [dither](int pr, int pg, int pb, uint32_t px, uint32_t py) {
                            static constexpr int matrix[4][4] = {
                                {-4, 0, -3, 1}, {2, -2, 3, -1},
                                {-3, 1, -4, 0}, {3, -1, 2, -2}};
                            const int d = dither ? matrix[py & 3u][px & 3u] : 0;
                            pr = std::clamp(pr + d, 0, 255);
                            pg = std::clamp(pg + d, 0, 255);
                            pb = std::clamp(pb + d, 0, 255);
                            return static_cast<uint16_t>((pr >> 3) | ((pg >> 3) << 5) |
                                                         ((pb >> 3) << 10) | 0x8000);
                        };
                        const uint32_t px0 = x - 1u;
                        const int y0 = (0x95 * std::max(0, static_cast<int>(ycbcr[y * 16u + px0]) - 16)) >> 6;
                        const int cb0 = static_cast<int>(ycbcr[256u + (y / 2u) * 8u + px0 / 2u]) - 128;
                        const int cr0 = static_cast<int>(ycbcr[320u + (y / 2u) * 8u + px0 / 2u]) - 128;
                        const int r0 = std::clamp((y0 + ((0xCC * cr0) >> 6) + 1) >> 1, 0, 255);
                        const int g0 = std::clamp((y0 + ((-0x68 * cr0) >> 6) + ((-0x32 * cb0) >> 6) + 1) >> 1, 0, 255);
                        const int b0 = std::clamp((y0 + ((0x102 * cb0) >> 6) + 1) >> 1, 0, 255);
                        complete = emit(static_cast<uint32_t>(pack16(r0, g0, b0, px0, y)) |
                                        (static_cast<uint32_t>(pack16(r, g, b, x, y)) << 16u));
                    }
                    if (!complete)
                        break;
                }
            }
        }
        if (!complete)
            m_ipu.ctrlKeep |= 1u << 14u;
        std::cerr << "[probe:ipu-csc-end] mb=" << mb
                  << " complete=" << complete
                  << " bit=0x" << std::hex << m_ipu.bitPos
                  << " inQwc=0x" << m_ioRegisters[0x1000B420u]
                  << " inChcr=0x" << m_ioRegisters[0x1000B400u]
                  << " outMadr=0x" << m_ioRegisters[0x1000B010u]
                  << " outQwc=0x" << m_ioRegisters[0x1000B020u]
                  << " outChcr=0x" << m_ioRegisters[0x1000B000u]
                  << " inWords=0x" << m_ipu.inFifo.size()
                  << " outWords=0x" << m_ipu.outFifo.size() << std::dec << '\n';
        if (complete && !rgb16)
        {
            const uint32_t bytes = macroblocks * 1024u;
            const uint32_t end = m_ioRegisters[0x1000B010u] & 0x1FFFFFFu;
            if (end >= bytes && end <= PS2_RAM_SIZE)
            {
                const uint8_t *pixels = m_rdram + end - bytes;
                uint32_t colored = 0u;
                uint32_t opaque = 0u;
                for (uint32_t i = 0u; i < bytes; i += 4u)
                {
                    colored += (pixels[i] | pixels[i + 1u] | pixels[i + 2u]) != 0u;
                    opaque += pixels[i + 3u] == 0x80u;
                }
                std::cerr << "[probe:ipu-csc-frame] rgb=0x" << std::hex << (end - bytes)
                          << " bytes=0x" << bytes << std::dec
                          << " colored=" << colored << " opaque=" << opaque << '\n';
                if (const char *dump = std::getenv("PS2X_IPU_CSC_DUMP"))
                {
                    static uint32_t dumpIndex = 0u;
                    const uint32_t frameIndex = dumpIndex++;
                    if ((frameIndex < 4u || (frameIndex < 1000u && frameIndex % 10u == 0u)) && *dump != '\0')
                    {
                        const std::string path = std::string(dump) + "-" +
                                                 std::to_string(frameIndex) + ".rgba";
                        if (std::FILE *file = std::fopen(path.c_str(), "wb"))
                        {
                            std::fwrite(pixels, 1u, bytes, file);
                            std::fclose(file);
                        }
                    }
                }
            }
        }
        m_ipu.busy = false;
        break;
    }
    case 0x1u: // IDEC
    case 0x6u: // SETVQ
    case 0x8u: // PACK
    case 0x9u: // SETTH
    default:
        // Latched only for now; the BUSY transient below still applies.
        break;
    }
    // No synchronous clear: BUSY stays set until observed twice via 64-bit
    // CMD reads (see read64), giving the guest its wait/feed window.
    syncIpuCmdMirror();

}

void PS2Memory::runIpuInDma(uint32_t channelBase)
{
    uint32_t madr = m_ioRegisters[channelBase + 0x10u];
    uint32_t qwc = m_ioRegisters[channelBase + 0x20u];
    uint32_t chcr = m_ioRegisters[channelBase + 0x00u];
    const uint32_t mode = (chcr >> 2) & 0x3u;
    bool transferEnded = false;
    static std::atomic<uint32_t> inDmaCount{0u};
    const uint32_t n = inDmaCount.fetch_add(1u, std::memory_order_relaxed);
    if (n < 2048u)
    {
        std::cerr << "[probe:ipu-indma] n=" << n
                  << " madr=0x" << std::hex << madr
                  << " qwc=0x" << qwc << " mode=0x" << mode
                  << std::dec << '\n';
    }
    auto appendQwords = [&](uint32_t source, uint32_t count) -> uint32_t
    {
        const uint32_t freeQwc = static_cast<uint32_t>((IpuState::kFifoWords - m_ipu.inFifo.size()) / 4u);
        count = std::min(count, freeQwc);
        if (count == 0u)
            return 0u;
        // In DMAC MADR/TADR, bit 31 selects scratchpad. This is distinct
        // from the EE's 0x70000000 scratchpad virtual-address window.
        const bool scratch = (source & 0x80000000u) != 0u || isScratchpad(source);
        uint32_t phys = 0u;
        try
        {
            phys = scratch ? (source & 0x3FF0u) : translateAddress(source);
        }
        catch (const std::exception &)
        {
            return 0u;
        }
        const uint8_t *base = scratch ? m_scratchpad : m_rdram;
        const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
        const uint64_t bytes = static_cast<uint64_t>(count) * 16u;
        if (phys > limit || bytes > static_cast<uint64_t>(limit - phys))
            return 0u;
        for (uint32_t qw = 0u; qw < count; ++qw)
        {
            for (uint32_t i = 0u; i < 4u; ++i)
            {
                uint32_t w = 0u;
                std::memcpy(&w, base + phys + qw * 16u + i * 4u, sizeof(w));
                m_ipu.inFifo.push_back(w);
            }
        }
        return count;
    };

    if (mode == 0u)
    {
        m_ipu.inputChainStarted = false;
        m_ipu.inputChainEndPending = false;
        const uint32_t moved = appendQwords(madr, qwc);
        madr += moved * 16u;
        qwc -= moved;
    }
    else if (mode == 1u)
    {
        uint32_t tagAddr = m_ioRegisters[channelBase + 0x30u];
        uint32_t asr0 = m_ioRegisters[channelBase + 0x40u];
        uint32_t asr1 = m_ioRegisters[channelBase + 0x50u];
        uint32_t asp = (chcr >> 4u) & 0x3u;
        const bool tie = (chcr & 0x80u) != 0u;
        uint32_t tags = 0u;
        while (m_ipu.inFifo.size() + 4u <= IpuState::kFifoWords && tags < 64u)
        {
            if (qwc == 0u)
            {
                // Finish the previous tag only after its payload drained.
                if (m_ipu.inputChainStarted && m_ipu.inputChainEndPending)
                {
                    transferEnded = true;
                    break;
                }

                const bool tagScratch = (tagAddr & 0x80000000u) != 0u || isScratchpad(tagAddr);
                uint32_t tagPhys = 0u;
                try { tagPhys = tagScratch ? (tagAddr & 0x3FF0u) : translateAddress(tagAddr); }
                catch (const std::exception &) { break; }
                const uint8_t *tagBase = tagScratch ? m_scratchpad : m_rdram;
                const uint32_t tagLimit = tagScratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
                if (tagPhys + 16u > tagLimit)
                    break;
                uint64_t tag = 0u;
                std::memcpy(&tag, tagBase + tagPhys, sizeof(tag));
                qwc = static_cast<uint32_t>(tag & 0xFFFFu);
                const uint32_t id = static_cast<uint32_t>((tag >> 28u) & 0x7u);
                const uint32_t addr = static_cast<uint32_t>((tag >> 32u) & 0x7FFFFFFFu);
                madr = tagAddr + 16u;
                uint32_t nextTag = madr + qwc * 16u;
                switch (id)
                {
                case 0u: madr = addr; nextTag = tagAddr + 16u; break;              // REFE
                case 1u: break;                                                    // CNT
                case 2u: nextTag = addr; break;                                    // NEXT
                case 3u:                                                          // REF
                case 4u: madr = addr; nextTag = tagAddr + 16u; break;              // REFS
                case 5u:                                                          // CALL
                    if (asp == 0u) { asr0 = nextTag; asp = 1u; }
                    else if (asp == 1u) { asr1 = nextTag; asp = 2u; }
                    nextTag = addr;
                    break;
                case 6u:                                                          // RET
                    if (asp == 2u) { nextTag = asr1; asp = 1u; }
                    else if (asp == 1u) { nextTag = asr0; asp = 0u; }
                    break;
                case 7u: break;                                                    // END
                }
                tagAddr = nextTag;
                chcr = (chcr & 0x0000FFFFu) | (static_cast<uint32_t>(tag) & 0xFFFF0000u);
                m_ipu.inputChainStarted = true;
                m_ipu.inputChainEndPending =
                    id == 0u || id == 7u || (tie && (tag & 0x80000000u) != 0u);
                ++tags;

                if (tags <= 8u)
                {
                    std::cerr << "[probe:ipu-tag] n=" << tags
                              << " id=" << id << " qwc=0x" << std::hex << qwc
                              << " madr=0x" << madr << " next=0x" << tagAddr
                              << std::dec << '\n';
                }

                if (qwc == 0u && (id == 0u || id == 7u || (tie && (tag & 0x80000000u))))
                {
                    transferEnded = true;
                    break;
                }
            }

            const uint32_t moved = appendQwords(madr, qwc);
            if (moved == 0u)
                break;
            madr += moved * 16u;
            qwc -= moved;
        }
        m_ioRegisters[channelBase + 0x30u] = tagAddr;
        m_ioRegisters[channelBase + 0x40u] = asr0;
        m_ioRegisters[channelBase + 0x50u] = asr1;
        static std::atomic<uint32_t> chainLog{0u};
        if (chainLog.fetch_add(1u, std::memory_order_relaxed) < 32u)
            std::cerr << "[probe:ipu-chain] tags=" << tags
                      << " qwc=" << qwc << " words=" << m_ipu.inFifo.size()
                      << " ended=" << transferEnded << '\n';
        chcr = (chcr & ~(0x3u << 4u)) | ((asp & 0x3u) << 4u);
        if (transferEnded)
        {
            m_ipu.inputChainStarted = false;
            m_ipu.inputChainEndPending = false;
        }
    }
    else
    {
        static std::atomic<uint32_t> chainWarn{0u};
        if (chainWarn.fetch_add(1u, std::memory_order_relaxed) < 4u)
            std::cerr << "[probe:ipu-dma] unsupported mode=" << mode << '\n';
        qwc = 0u;
    }
    m_ioRegisters[channelBase + 0x10u] = madr;
    m_ioRegisters[channelBase + 0x20u] = qwc;
    if (mode == 0u ? qwc == 0u : transferEnded)
        chcr &= ~0x100u;
    m_ioRegisters[channelBase + 0x00u] = chcr;
}

void PS2Memory::fillIpuOutFifo()
{
    while (m_ipu.bdecWordsLeft != 0u && m_ipu.outFifo.size() < IpuState::kFifoWords)
    {
        m_ipu.outFifo.push_back(m_ipu.bdecOutput[m_ipu.bdecOutputPos++]);
        --m_ipu.bdecWordsLeft;
    }
}

void PS2Memory::runIpuOutDma(uint32_t channelBase)
{
    uint32_t madr = m_ioRegisters[channelBase + 0x10u];
    uint32_t qwc = m_ioRegisters[channelBase + 0x20u];
    const uint32_t chcr = m_ioRegisters[channelBase + 0x00u];
    static std::atomic<uint32_t> outDmaProbes{0u};
    const uint32_t outDmaProbe = outDmaProbes.fetch_add(1u, std::memory_order_relaxed);
    if (outDmaProbe < 12u)
        std::cerr << "[probe:ipu-outdma-start] n=" << outDmaProbe
                  << " madr=0x" << std::hex << madr << " qwc=0x" << qwc
                  << " words=0x" << m_ipu.outFifo.size()
                  << " first=0x" << (m_ipu.outFifo.empty() ? 0u : m_ipu.outFifo.front())
                  << std::dec << '\n';
    const uint32_t mode = (chcr >> 2) & 0x3u;
    if (mode != 0u)
    {
        static std::atomic<uint32_t> chainWarn{0u};
        if (chainWarn.fetch_add(1u, std::memory_order_relaxed) < 4u)
            std::cerr << "[probe:ipu-dma] chain mode on 0x" << std::hex << channelBase << std::dec << '\n';
        m_ioRegisters[channelBase + 0x00u] = chcr & ~0x100u;
        return;
    }
    while (qwc != 0u)
    {
        // A decoded macroblock is larger than the eight-qword hardware FIFO.
        // Refill it as DMA drains, until this macroblock is fully emitted.
        fillIpuOutFifo();
        // DMA transfers complete 16-byte quadwords. CSC may produce pixels
        // one word at a time; do not count an incomplete quadword as sent.
        if (m_ipu.outFifo.size() < 4u)
            break;
        const bool scratch = (madr & 0x80000000u) != 0u || isScratchpad(madr);
        uint32_t phys = 0u;
        try
        {
            phys = scratch ? (madr & 0x3FF0u) : translateAddress(madr);
        }
        catch (const std::exception &)
        {
            break;
        }
        uint8_t *base = scratch ? m_scratchpad : m_rdram;
        const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
        if (phys + 16u > limit)
            break;
        for (uint32_t i = 0u; i < 4u; ++i)
        {
            const uint32_t w = m_ipu.outFifo.front();
            m_ipu.outFifo.pop_front();
            std::memcpy(base + phys + i * 4u, &w, sizeof(w));
        }
        madr += 16u;
        --qwc;
    }
    // If the programmed DMA size ended mid-macroblock, retain the remainder
    // in the output FIFO for the next OUT transfer.
    fillIpuOutFifo();
    m_ioRegisters[channelBase + 0x10u] = madr;
    m_ioRegisters[channelBase + 0x20u] = qwc;
    if (qwc == 0u)
        m_ioRegisters[channelBase + 0x00u] = chcr & ~0x100u;
}

void PS2Memory::resumeIpuOutDma()
{
    fillIpuOutFifo();
    const uint32_t chcr = m_ioRegisters[0x1000B000u];
    if ((chcr & 0x100u) != 0u && !m_ipu.outFifo.empty())
        runIpuOutDma(0x1000B000u);
}

void PS2Memory::write32(uint32_t address, uint32_t value)
{
    if (address & 3)
    {
        throw std::runtime_error("Unaligned 32-bit write at address: 0x" + std::to_string(address));
    }

    if (isGsPrivReg(address))
    {
        uint32_t off = address & 7;
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            // CSR: bits 0..1 of the low dword are write-one-to-clear status bits.
            // Done as a single atomic RMW -- see writeCsrHalf's comment.
            writeCsrHalf(gs_regs.csr, off, value);
        }
        else if (uint64_t *reg = gsRegPtr(gs_regs, address))
        {
            uint64_t mask = 0xFFFFFFFFULL << (off * 8);
            uint64_t newVal = (*reg & ~mask) | ((uint64_t)value << (off * 8));
            *reg = newVal;
        }
        return;
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        storeScalar<uint32_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, value, "write32 scratchpad", address);
    }
    else if (physAddr < PS2_RAM_SIZE)
    {
        // Check if this might be code modification
        markModified(address, 4);

        storeScalar<uint32_t>(m_rdram, physAddr, PS2_RAM_SIZE, value, "write32 rdram", address);
    }
    else
    {
        uint32_t vuOffset = 0;
        uint32_t vuLimit = 0;
        if (uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint32_t), vuOffset, vuLimit))
        {
            storeScalar<uint32_t>(vuMem, vuOffset, vuLimit, value, "write32 vu", address);
            if (vuMem == m_vu0Code)
                markVU0CodeModified();
            else if (vuMem == m_vu1Code)
                markVU1CodeModified();
            return;
        }
    }
    if (isIoRegister(physAddr))
    {
        writeIORegister(physAddr, value);
    }
}

void PS2Memory::write64(uint32_t address, uint64_t value)
{
    if (address & 7)
    {
        throw std::runtime_error("Unaligned 64-bit write at address: 0x" + std::to_string(address));
    }

    if (isGsPrivReg(address))
    {
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            // CSR: bits 0..1 are write-one-to-clear status bits. Done as a single
            // atomic RMW -- see writeCsrFull's comment.
            writeCsrFull(gs_regs.csr, value);
        }
        else if (uint64_t *reg = gsRegPtr(gs_regs, address))
        {
            *reg = value;
        }
        return;
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (scratch)
    {
        storeScalar<uint64_t>(m_scratchpad, physAddr, PS2_SCRATCHPAD_SIZE, value, "write64 scratchpad", address);
    }
    else if (physAddr < PS2_RAM_SIZE)
    {
        markModified(address, 8);
        storeScalar<uint64_t>(m_rdram, physAddr, PS2_RAM_SIZE, value, "write64 rdram", address);
    }
    else
    {
        uint32_t vuOffset = 0;
        uint32_t vuLimit = 0;
        if (uint8_t *vuMem = mapVuMemory(physAddr, sizeof(uint64_t), vuOffset, vuLimit))
        {
            storeScalar<uint64_t>(vuMem, vuOffset, vuLimit, value, "write64 vu", address);
            if (vuMem == m_vu0Code)
                markVU0CodeModified();
            else if (vuMem == m_vu1Code)
                markVU1CodeModified();
            return;
        }
    }
    if (isIoRegister(physAddr))
    {
        write32(address, (uint32_t)value);
        write32(address + 4, (uint32_t)(value >> 32));
    }
}

void PS2Memory::write128(uint32_t address, __m128i value)
{
    if (address & 15)
    {
        throw std::runtime_error("Unaligned 128-bit write at address: 0x" + std::to_string(address));
    }

    const bool scratch = isScratchpad(address);
    uint32_t physAddr = translateAddress(address);

    if (!scratch && physAddr == 0x10004000u) // VIF0_FIFO
    {
        alignas(16) uint8_t fifoData[16];
        _mm_storeu_si128(reinterpret_cast<__m128i *>(fifoData), value);
        processVIF0Data(fifoData, sizeof(fifoData));
        return;
    }
    if (!scratch && physAddr == 0x10005000u) // VIF1_FIFO
    {
        alignas(16) uint8_t fifoData[16];
        _mm_storeu_si128(reinterpret_cast<__m128i *>(fifoData), value);
        processVIF1Data(fifoData, sizeof(fifoData));
        return;
    }
    // 128-bit stores to IPU_CMD feed the input FIFO (hardware behavior);
    // only 32-bit stores are commands.
    if (physAddr == 0x10002000u)
    {
        alignas(16) uint32_t words[4];
        _mm_store_si128(reinterpret_cast<__m128i *>(words), value);
        for (uint32_t i = 0u; i < 4u; ++i)
        {
            if (m_ipu.inFifo.size() < IpuState::kFifoWords)
                m_ipu.inFifo.push_back(words[i]);
        }
        static std::atomic<uint32_t> directCount{0u};
        const uint32_t n = directCount.fetch_add(1u, std::memory_order_relaxed);
        if (n < 8u)
        {
            std::cerr << "[probe:ipu-direct] n=" << n
                      << " inQw=0x" << std::hex << m_ipu.inFifo.size() / 4u
                      << std::dec << '\n';
        }
        return;
    }

    if (scratch)
    {
        inRange(physAddr, sizeof(__m128i), PS2_SCRATCHPAD_SIZE, "write128 scratchpad", address);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(&m_scratchpad[physAddr]), value);
    }
    else if (physAddr < PS2_RAM_SIZE)
    {
        markModified(address, 16);
        inRange(physAddr, sizeof(__m128i), PS2_RAM_SIZE, "write128 rdram", address);
        _mm_storeu_si128(reinterpret_cast<__m128i *>(&m_rdram[physAddr]), value);
    }
    else
    {
        uint32_t vuOffset = 0;
        uint32_t vuLimit = 0;
        if (uint8_t *vuMem = mapVuMemory(physAddr, sizeof(__m128i), vuOffset, vuLimit))
        {
            inRange(vuOffset, sizeof(__m128i), vuLimit, "write128 vu", address);
            _mm_storeu_si128(reinterpret_cast<__m128i *>(vuMem + vuOffset), value);
            if (vuMem == m_vu0Code)
                markVU0CodeModified();
            else if (vuMem == m_vu1Code)
                markVU1CodeModified();
            return;
        }
    }
    if (isIoRegister(physAddr))
    {
        // Non-RAM 128-bit stores are modeled as two 64-bit stores.
        uint64_t lo = _mm_extract_epi64(value, 0);
        uint64_t hi = _mm_extract_epi64(value, 1);

        write64(address, lo);
        write64(address + 8, hi);
    }
}

bool PS2Memory::writeIORegister(uint32_t address, uint32_t value)
{
    size_t timerIndex = 0u;
    uint32_t timerOffset = 0u;
    if (decodeEeTimerRegister(address, timerIndex, timerOffset))
    {
        EeTimer &timer = m_eeTimers[timerIndex];
        switch (timerOffset)
        {
        case kEeTimerCountOffset:
            timer.count = value & 0xFFFFu;
            timer.clockRemainder = 0u;
            break;
        case kEeTimerModeOffset:
        {
            const uint32_t previousMode = timer.mode;
            const uint32_t status = (previousMode & kEeTimerModeStatusMask) & ~(value & kEeTimerModeStatusMask);
            timer.mode = (value & kEeTimerModeConfigMask) | status;
            if (((previousMode ^ timer.mode) & (kEeTimerModeClksMask | kEeTimerModeCue)) != 0u)
            {
                timer.clockRemainder = 0u;
            }
            break;
        }
        case kEeTimerCompareOffset:
            timer.compare = value & 0xFFFFu;
            break;
        case kEeTimerHoldOffset:
            timer.hold = value & 0xFFFFu;
            break;
        default:
            return false;
        }
        return true;
    }

    if (isGsPrivReg(address))
    {
        // NB: unreachable from write8/16/32/64 today since those all funnel IO
        // register writes through addresses in PS2_IO_BASE's range, which is
        // disjoint from PS2_GS_PRIV_REG_BASE; kept correct for direct callers.
        m_ioRegisters[address] = value;
        const uint32_t off = address & 7u;
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            writeCsrHalf(gs_regs.csr, off, value);
        }
        else if (uint64_t *reg = gsRegPtr(gs_regs, address))
        {
            const uint64_t mask = 0xFFFFFFFFull << (off * 8u);
            *reg = (*reg & ~mask) | (static_cast<uint64_t>(value) << (off * 8u));
        }
        m_gsWriteCount.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    if (address >= 0x10002000 && address <= 0x10002030)
    {
        // Bounded diagnostics for the IPU command/status path.  This remains
        // middleware-level instrumentation (no guest address knowledge) and
        // is intentionally capped so long-running decoders do not flood logs.
        static std::atomic<uint32_t> ipuWrites{0u};
        const uint32_t n = ipuWrites.fetch_add(1u, std::memory_order_relaxed);
        if (n < 128u)
        {
            std::cerr << "[probe:ipu-write] addr=0x" << std::hex << address
                      << " value=0x" << value << std::dec << '\n';
        }
        if (address == 0x10002000u)
        {
            writeIpuCommand(value);
        }
        else if (address == 0x10002010u)
        {
            // CTRL write mask mirrors hardware (PCSX2 formula): only the
            // setup field is settable, status bits (IFC/OFC/CBP/ECD/SCD)
            // are read-only. RST (bit 30) resets the FIFOs/bit pointer.
            const bool reset = (value & (1u << 30u)) != 0u;
            // RST is a write trigger, not persistent state. Hardware clears
            // it during soft reset; exposing it on the next CTRL read makes
            // guest read-modify-write sequences reset the IPU forever.
            m_ipu.ctrlKeep = value & 0x07F30000u;
            if (reset)
            {
                m_ipu.inFifo.clear();
                m_ipu.outFifo.clear();
                m_ipu.bitPos = 0u;
                m_ipu.fragBits = 0u;
                m_ipu.busy = false;
                m_ipu.scd = false;
                m_ipu.bdecWordsLeft = 0u;
                m_ipu.bdecOutputPos = 0u;
                m_ipu.codedBlockPattern = 0u;
                m_ipu.sawSliceCode = false;
                m_ipu.mpeg2.reset();
            }
            syncIpuCmdMirror();
        }
        else
        {
            m_ioRegisters[address] = value;
        }
        return true;
    }

    if (address == 0x1000E010u)
    {
        const uint32_t current = m_ioRegisters.count(address) ? m_ioRegisters[address] : 0u;
        uint32_t status = current & 0x3FFu;
        uint32_t mask = (current >> 16) & 0x3FFu;

        // D_STAT low bits are W1C status, high bits [16..25] toggle masks on write-one.
        status &= ~(value & 0x3FFu);
        mask ^= ((value >> 16) & 0x3FFu);

        uint32_t next = (current & ~((0x3FFu) | (0x3FFu << 16) | (1u << 31)));
        next |= status | (mask << 16);
        if ((status & mask) != 0u)
            next |= (1u << 31);
        m_ioRegisters[address] = next;
        return true;
    }

    m_ioRegisters[address] = value;

    if (address >= 0x10003C00u && address < 0x10003E00u)
    {
        m_vifWriteCount.fetch_add(1, std::memory_order_relaxed);

        switch (address)
        {
        case 0x10003C10u:     // VIF1_FBRST
            if (value & 0x1u) // RST
            {
                const bool wasPath3Masked = m_path3Masked;
                std::memset(&vif1_regs, 0, sizeof(vif1_regs));
                m_vif1PendingPath2ImageQwc = 0u;
                m_vif1PendingPath2DirectHl = false;
                m_path3Masked = false;
                if (wasPath3Masked)
                    flushMaskedPath3Packets();
            }
            if (value & 0x8u) // STC
            {
                vif1_regs.stat &= ~((1u << 8) | (1u << 9) | (1u << 10) | (1u << 11) | (1u << 12) | (1u << 13));
            }
            break;
        case 0x10003C30u:
            vif1_regs.mark = value & 0xFFFFu;
            vif1_regs.stat &= ~(1u << 6); // clear MRK flag on CPU write
            break;
        case 0x10003C40u:
            vif1_regs.cycle = value & 0xFFFFu;
            break;
        case 0x10003C50u:
            vif1_regs.mode = value & 0x3u;
            break;
        case 0x10003C60u:
            vif1_regs.num = value & 0xFFu;
            break;
        case 0x10003C70u:
            vif1_regs.mask = value;
            break;
        case 0x10003C80u:
            vif1_regs.code = value;
            break;
        case 0x10003C90u:
            vif1_regs.itops = value & 0x3FFu;
            break;
        case 0x10003CA0u:
            vif1_regs.base = value & 0x3FFu;
            break;
        case 0x10003CB0u:
            vif1_regs.ofst = value & 0x3FFu;
            break;
        case 0x10003CC0u:
            vif1_regs.tops = value & 0x3FFu;
            break;
        case 0x10003CD0u:
            vif1_regs.itop = value & 0x3FFu;
            break;
        case 0x10003CE0u:
            vif1_regs.top = value & 0x3FFu;
            break;
        default:
            break;
        }

        return true;
    }

    if (address >= 0x10003800u && address < 0x10003A00u)
    {
        m_vifWriteCount.fetch_add(1, std::memory_order_relaxed);
        return true;
    }

    if (address >= 0x10008000 && address < 0x1000F000)
    {
        if ((address & 0xFF) == 0x00 && (value & 0x100))
        {
            const auto dctrlIt = m_ioRegisters.find(0x1000E000u);
            const bool dmacEnabled = (dctrlIt == m_ioRegisters.end()) || ((dctrlIt->second & 0x1u) != 0u);
            if (!dmacEnabled)
            {
                return true;
            }

            const uint32_t channelBase = address & 0xFFFFFF00;
            const uint32_t madr = m_ioRegisters[channelBase + 0x10];
            const uint32_t qwc = m_ioRegisters[channelBase + 0x20];
            const uint32_t tadr = m_ioRegisters[channelBase + 0x30];
            m_dmaStartCount.fetch_add(1, std::memory_order_relaxed);

            if (tryProcessScratchpadDma(channelBase, value))
            {
                return true;
            }

            // IPU FIFO channels run synchronously: the guest polls QWC/STR,
            // so completing inline keeps stream flow without interrupts.
            if (channelBase == 0x1000B000u)
            {
                runIpuOutDma(channelBase);
            }
            else if (channelBase == 0x1000B400u)
            {
                runIpuInDma(channelBase);
            }

            if (channelBase == 0x1000D000u || channelBase == 0x1000D400u)
            {
                const bool toSpr = channelBase == 0x1000D400u;
                uint32_t memoryAddr = madr;
                uint32_t sprAddr = m_ioRegisters[channelBase + 0x80u] & 0x3FF0u;
                uint32_t remaining = qwc & 0xFFFFu;
                uint32_t tagAddr = tadr;
                const uint32_t mode = (value >> 2u) & 3u;
                auto transfer = [&](uint32_t count) {
                    for (uint32_t i = 0; i < count; ++i)
                    {
                        const uint32_t phys = translateAddress(memoryAddr);
                        inRange(phys, 16u, PS2_RAM_SIZE, "SPR DMA", memoryAddr);
                        if (toSpr)
                            std::memcpy(m_scratchpad + sprAddr, m_rdram + phys, 16u);
                        else
                            std::memcpy(m_rdram + phys, m_scratchpad + sprAddr, 16u);
                        memoryAddr += 16u;
                        sprAddr = (sprAddr + 16u) & 0x3FFFu;
                    }
                };
                if (mode == 0u)
                    transfer(remaining);
                else if (mode == 2u)
                {
                    const uint32_t sqwc = m_ioRegisters[0x1000E030u];
                    const uint32_t block = (sqwc >> 16u) & 0xFFu;
                    while (remaining != 0u)
                    {
                        const uint32_t count = block ? std::min(block, remaining) : remaining;
                        transfer(count);
                        memoryAddr += (sqwc & 0xFFu) * 16u;
                        remaining -= count;
                    }
                }
                else if (mode == 1u && toSpr)
                {
                    transfer(remaining);
                    bool done = false;
                    uint32_t asp = (value >> 4u) & 3u;
                    for (uint32_t tags = 0u; !done && tags < 65536u; ++tags)
                    {
                        const uint32_t phys = translateAddress(tagAddr);
                        const uint64_t tag = loadScalar<uint64_t>(m_rdram, phys, PS2_RAM_SIZE, "SPR DMA tag", tagAddr);
                        const uint32_t id = static_cast<uint32_t>(tag >> 28u) & 7u;
                        const uint32_t count = static_cast<uint32_t>(tag) & 0xFFFFu;
                        const uint32_t target = static_cast<uint32_t>(tag >> 32u) & 0x7FFFFFF0u;
                        const uint32_t next = tagAddr + 16u;
                        memoryAddr = next;
                        tagAddr = next + count * 16u;
                        switch (id)
                        {
                        case 0u: memoryAddr = target; tagAddr = next; done = true; break;
                        case 1u: break;
                        case 2u: tagAddr = target; break;
                        case 3u:
                        case 4u: memoryAddr = target; tagAddr = next; break;
                        case 5u:
                            if (asp >= 2u) throw std::runtime_error("SPR DMA CALL stack overflow");
                            m_ioRegisters[channelBase + 0x40u + asp * 16u] = tagAddr;
                            ++asp;
                            tagAddr = target;
                            break;
                        case 6u:
                            if (asp == 0u) done = true;
                            else tagAddr = m_ioRegisters[channelBase + 0x40u + --asp * 16u];
                            break;
                        case 7u: done = true; break;
                        }
                        transfer(count);
                        if ((value & 0x80u) && (tag & 0x80000000u)) done = true;
                        m_ioRegisters[channelBase] = (value & 0xFFFFu & ~0x30u) |
                            (asp << 4u) | (static_cast<uint32_t>(tag) & 0xFFFF0000u);
                    }
                    if (!done) throw std::runtime_error("SPR DMA chain exceeds tag limit");
                }
                else
                    throw std::runtime_error("Unsupported SPR DMA transfer mode");

                m_ioRegisters[channelBase + 0x10u] = memoryAddr;
                m_ioRegisters[channelBase + 0x20u] = 0u;
                m_ioRegisters[channelBase + 0x30u] = tagAddr;
                m_ioRegisters[channelBase + 0x80u] = sprAddr;
                m_ioRegisters[channelBase] &= ~0x100u;
                m_ioRegisters[0x1000E010u] |= 1u << (toSpr ? 9u : 8u);
                static uint32_t sprProbes = 0u;
                if (sprProbes++ < 16u)
                    std::cerr << "[probe:spr-dma] ch=0x" << std::hex << channelBase
                              << " mode=" << mode << " madr=0x" << madr << " end=0x" << memoryAddr
                              << " sadr=0x" << sprAddr << std::dec << '\n';
            }

            if ((channelBase == 0x1000A000u || channelBase == 0x10009000u || channelBase == 0x10008000u) &&
                (m_gsVRAM || channelBase == 0x10008000u))
            {
                auto enqueueTransfer = [&](uint32_t srcAddr, uint32_t qwCount)
                {
                    if (qwCount == 0)
                        return;
                    const bool scratch = isScratchpad(srcAddr);
                    PendingTransfer pt;
                    pt.fromScratchpad = scratch;
                    pt.srcAddr = srcAddr;
                    pt.qwc = qwCount;
                    if (channelBase == 0x1000A000u)
                        m_pendingGifTransfers.push_back(pt);
                    else if (channelBase == 0x10009000u)
                        m_pendingVif1Transfers.push_back(pt);
                    else if (channelBase == 0x10008000u)
                        m_pendingVif0Transfers.push_back(pt);
                };

                uint32_t chcr = value;
                uint32_t mode = (chcr >> 2) & 0x3;

                if (mode == 0 && qwc > 0)
                {
                    enqueueTransfer(madr, qwc);
                }
                else if (mode == 1)
                {
                    uint32_t tagAddr = m_ioRegisters[channelBase + 0x30];
                    uint32_t asr0 = m_ioRegisters[channelBase + 0x40];
                    uint32_t asr1 = m_ioRegisters[channelBase + 0x50];
                    uint32_t asp = (chcr >> 4) & 0x3u;
                    const bool tieEnabled = (chcr & (1u << 7)) != 0u;
                    const int kMaxChainTags = 4096;
                    std::vector<uint8_t> chainBuf;

                    auto appendData = [&](uint32_t srcAddr, uint32_t qwCount)
                    {
                        const uint64_t bytes64 = static_cast<uint64_t>(qwCount) * 16ull;
                        uint32_t bytes = (bytes64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes64);
                        const bool scratch = isScratchpad(srcAddr);
                        uint32_t src = 0;
                        src = translateAddress(srcAddr);
                        const uint8_t *base2;
                        uint32_t maxSz2;
                        if (scratch)
                        {
                            base2 = m_scratchpad;
                            maxSz2 = PS2_SCRATCHPAD_SIZE;
                        }
                        else
                        {
                            base2 = m_rdram;
                            maxSz2 = PS2_RAM_SIZE;
                        }

                        while (bytes > 0)
                        {
                            if (src >= maxSz2)
                                src = 0;
                            uint32_t chunk = bytes;
                            if (src + chunk > maxSz2)
                                chunk = maxSz2 - src;
                            if (chunk == 0)
                                break;
                            chainBuf.insert(chainBuf.end(), base2 + src, base2 + src + chunk);
                            bytes -= chunk;
                            src += chunk;
                        }
                    };

                    auto appendVifTagData = [&](uint32_t localTagAddr)
                    {
                        uint32_t tagPhys = 0u;
                        const bool tagScratch = isScratchpad(localTagAddr);
                        tagPhys = translateAddress(localTagAddr);

                        const uint8_t *localBase = tagScratch ? m_scratchpad : m_rdram;
                        const uint32_t localMax = tagScratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
                        if (tagPhys + 16u > localMax)
                            return;

                        // CHCR.TTE sends the DMAtag's upper 64 bits to the channel before
                        // the tag payload. VIF chains use those bytes for two VIFcodes.
                        chainBuf.insert(chainBuf.end(), localBase + tagPhys + 8u, localBase + tagPhys + 16u);
                    };

                    const bool isVifChannel =
                        channelBase == 0x10009000u || channelBase == 0x10008000u;
                    const bool transferTagData = isVifChannel && (chcr & 0x40u) != 0u;

                    int tagsProcessed = 0;
                    uint32_t lastTagUpper = (chcr >> 16) & 0xFFFFu;

                    while (tagsProcessed < kMaxChainTags)
                    {
                        const uint32_t currentTagAddr = tagAddr;
                        const bool tagInSPR = isScratchpad(tagAddr);
                        uint32_t physTag = 0;
                        try
                        {
                            physTag = translateAddress(tagAddr);
                        }
                        catch (...)
                        {
                            break;
                        }
                        const uint8_t *tagBase;
                        uint32_t tagMax;
                        if (tagInSPR)
                        {
                            tagBase = m_scratchpad;
                            tagMax = PS2_SCRATCHPAD_SIZE;
                        }
                        else
                        {
                            tagBase = m_rdram;
                            tagMax = PS2_RAM_SIZE;
                        }
                        if (physTag + 16 > tagMax)
                            break;

                        const uint8_t *tp = tagBase + physTag;
                        uint64_t tag = loadScalar<uint64_t>(tp, 0, 16, "dma chain tag", tagAddr);
                        uint16_t tagQwc = static_cast<uint16_t>(tag & 0xFFFF);
                        uint32_t id = static_cast<uint32_t>((tag >> 28) & 0x7);
                        const bool irq = ((tag >> 31) & 0x1ull) != 0ull;
                        uint32_t addr = static_cast<uint32_t>((tag >> 32) & 0x7FFFFFFF);
                        lastTagUpper = static_cast<uint32_t>((tag >> 16) & 0xFFFFu);
                        ++tagsProcessed;

                        if (channelBase == 0x10009000u && tagsProcessed <= 12)
                        {
                            std::cerr << "[probe:vif1-tag] n=" << tagsProcessed
                                      << " addr=0x" << std::hex << currentTagAddr
                                      << " id=" << id << " qwc=0x" << tagQwc
                                      << " upper=0x" << (tag >> 32) << std::dec << '\n';
                        }

                        uint32_t dataAddr = 0;
                        bool hasPayload = (tagQwc > 0);
                        bool endChain = false;

                        switch (id)
                        {
                        case 0:
                            dataAddr = addr;
                            tagAddr = tagAddr + 16;
                            endChain = true;
                            break;
                        case 1:
                            dataAddr = tagAddr + 16;
                            tagAddr = dataAddr + static_cast<uint32_t>(tagQwc) * 16u;
                            break;
                        case 2:
                            dataAddr = tagAddr + 16;
                            tagAddr = addr;
                            break;
                        case 3:
                        case 4:
                            dataAddr = addr;
                            tagAddr = tagAddr + 16;
                            break;
                        case 5:
                            dataAddr = tagAddr + 16;
                            {
                                const uint32_t retAddr = dataAddr + static_cast<uint32_t>(tagQwc) * 16u;
                                if (asp == 0u)
                                {
                                    asr0 = retAddr;
                                    asp = 1u;
                                }
                                else if (asp == 1u)
                                {
                                    asr1 = retAddr;
                                    asp = 2u;
                                }
                            }
                            tagAddr = addr;
                            break;
                        case 6:
                            dataAddr = tagAddr + 16;
                            if (asp == 2u)
                            {
                                tagAddr = asr1;
                                asp = 1u;
                            }
                            else if (asp == 1u)
                            {
                                tagAddr = asr0;
                                asp = 0u;
                            }
                            else
                            {
                                endChain = true;
                            }
                            break;
                        case 7:
                            dataAddr = tagAddr + 16;
                            endChain = true;
                            break;
                        default:
                            hasPayload = false;
                            endChain = true;
                            break;
                        }

                        if (transferTagData)
                            appendVifTagData(currentTagAddr);

                        if (hasPayload)
                            appendData(dataAddr, tagQwc);
                        if (irq && tieEnabled)
                            endChain = true;

                        if (endChain)
                            break;
                    }

                    static std::atomic<uint32_t> chainProbes{0u};
                    const uint32_t chainProbe = chainProbes.fetch_add(1u, std::memory_order_relaxed);
                    if (chainProbe < 64u)
                    {
                        std::cerr << "[probe:dma-chain] ch=0x" << std::hex << channelBase
                                  << " tags=" << std::dec << tagsProcessed
                                  << " bytes=0x" << std::hex << chainBuf.size()
                                  << " start=0x" << tadr << " end=0x" << tagAddr << std::dec << '\n';
                    }

                    m_ioRegisters[channelBase + 0x30] = tagAddr;
                    m_ioRegisters[channelBase + 0x40] = asr0;
                    m_ioRegisters[channelBase + 0x50] = asr1;
                    chcr = (chcr & ~(0x3u << 4)) | ((asp & 0x3u) << 4);
                    chcr = (chcr & 0x0000FFFFu) | (lastTagUpper << 16);
                    m_ioRegisters[channelBase + 0x00] = chcr;

                    if (!chainBuf.empty())
                    {
                        PendingTransfer pt;
                        pt.fromScratchpad = false;
                        pt.srcAddr = 0;
                        pt.qwc = 0;
                        pt.chainData = std::move(chainBuf);
                        if (channelBase == 0x1000A000)
                        {
                            m_pendingGifTransfers.push_back(std::move(pt));
                        }
                        else if (channelBase == 0x10009000u)
                        {
                            m_pendingVif1Transfers.push_back(std::move(pt));
                        }
                        else if (channelBase == 0x10008000u)
                        {
                            m_pendingVif0Transfers.push_back(std::move(pt));
                        }
                    }
                    // else if (channelBase == 0x10009000u)
                    // {

                    // }
                }
                else if (qwc > 0)
                {
                    enqueueTransfer(madr, qwc);
                }

                const bool autoProcessTransfers =
                    (channelBase == 0x1000A000u) ? (m_gifPacketCallback || m_gifArbiter != nullptr) : true;
                if (autoProcessTransfers)
                {
                    processPendingTransfers();
                }
            }
        }
        return true;
    }

    if (address >= 0x10000000 && address < 0x10010000)
    {
        if (address >= 0x10000200 && address < 0x10000300)
        {
            return true;
        }
        if (address >= 0x10000000 && address < 0x10000100)
        {
            return true;
        }
    }

    return false;
}

bool PS2Memory::tryProcessScratchpadDma(uint32_t channelBase, uint32_t chcr)
{
    static constexpr uint32_t kSprFromChannel = 0x1000D000u;
    static constexpr uint32_t kSprToChannel = 0x1000D400u;
    if (channelBase != kSprFromChannel && channelBase != kSprToChannel)
        return false;

    const uint32_t mode = (chcr >> 2u) & 0x3u;
    if (mode != 0u)
        return false;

    const uint32_t qwc = m_ioRegisters[channelBase + 0x20u] & 0xFFFFu;
    const uint32_t byteCount = qwc * 16u;
    const uint32_t originalMadr = m_ioRegisters[channelBase + 0x10u] & 0x7FFFFFF0u;
    const uint32_t originalSadr = m_ioRegisters[channelBase + 0x80u] & 0x3FF0u;

    uint32_t mainOffset = 0u;
    try
    {
        mainOffset = translateAddress(originalMadr);
    }
    catch (const std::exception &)
    {
        return false;
    }

    if (mainOffset > PS2_RAM_SIZE || byteCount > PS2_RAM_SIZE - mainOffset)
        return false;

    const bool fromScratchpad = channelBase == kSprFromChannel;
    uint32_t scratchOffset = originalSadr;
    uint32_t bytesLeft = byteCount;
    uint32_t copied = 0u;
    while (bytesLeft != 0u)
    {
        const uint32_t scratchChunk = PS2_SCRATCHPAD_SIZE - scratchOffset;
        const uint32_t chunk = std::min(bytesLeft, scratchChunk);
        if (fromScratchpad)
        {
            std::memcpy(m_rdram + mainOffset + copied, m_scratchpad + scratchOffset, chunk);
            markModified(mainOffset + copied, chunk);
        }
        else
        {
            std::memcpy(m_scratchpad + scratchOffset, m_rdram + mainOffset + copied, chunk);
        }

        copied += chunk;
        bytesLeft -= chunk;
        scratchOffset = (scratchOffset + chunk) & (PS2_SCRATCHPAD_SIZE - 1u);
    }

    m_ioRegisters[channelBase + 0x10u] = (originalMadr + byteCount) & 0x7FFFFFF0u;
    m_ioRegisters[channelBase + 0x20u] = 0u;
    m_ioRegisters[channelBase + 0x80u] = (originalSadr + byteCount) & 0x3FF0u;
    completeDmacChannel(channelBase, fromScratchpad ? 8u : 9u);
    return true;
}

void PS2Memory::completeDmacChannel(uint32_t channelBase, uint32_t cause)
{
    static constexpr uint32_t kDStat = 0x1000E010u;
    m_ioRegisters[channelBase] &= ~0x100u;

    uint32_t dstat = m_ioRegisters.count(kDStat) ? m_ioRegisters[kDStat] : 0u;
    dstat |= 1u << cause;
    const uint32_t status = dstat & 0x3FFu;
    const uint32_t mask = (dstat >> 16u) & 0x3FFu;
    if ((status & mask) != 0u)
        dstat |= 1u << 31u;
    else
        dstat &= ~(1u << 31u);
    m_ioRegisters[kDStat] = dstat;
    queueCompletedDmacCause(cause);
}

void PS2Memory::processPendingTransfers()
{
    const bool hadGif = !m_pendingGifTransfers.empty();
    uint32_t observedGifQwc = 0u;
    for (const auto &transfer : m_pendingGifTransfers)
    {
        const uint64_t transferQwc = !transfer.chainData.empty()
                                         ? (transfer.chainData.size() / 16u)
                                         : transfer.qwc;
        observedGifQwc = static_cast<uint32_t>(std::min<uint64_t>(16u, static_cast<uint64_t>(observedGifQwc) + transferQwc));
    }
    if (observedGifQwc != 0u)
    {
        constexpr uint32_t kGifStat = 0x10003020u;
        constexpr uint32_t kGifFqcMask = 0x1F000000u;
        uint32_t &gifStat = m_ioRegisters[kGifStat];
        gifStat = (gifStat & ~kGifFqcMask) | (observedGifQwc << 24u);
    }

    for (size_t idx = 0; idx < m_pendingGifTransfers.size(); ++idx)
    {
        auto &p = m_pendingGifTransfers[idx];
        if (!p.chainData.empty())
        {
            static std::atomic<uint32_t> gifSubmits{0u};
            const uint32_t gifProbe = gifSubmits.fetch_add(1u, std::memory_order_relaxed);
            if (gifProbe < 64u)
                std::cerr << "[probe:gif-submit] kind=chain bytes=0x" << std::hex
                          << p.chainData.size() << std::dec << '\n';
            m_seenGifCopy = true;
            m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);
            submitGifPacket(GifPathId::Path3, p.chainData.data(), static_cast<uint32_t>(p.chainData.size()), false);
        }
        else if (p.qwc > 0)
        {
            const uint64_t bytes64 = static_cast<uint64_t>(p.qwc) * 16ull;
            uint32_t sizeBytes = (bytes64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes64);
            uint32_t srcPhys = 0;
            try
            {
                srcPhys = translateAddress(p.srcAddr);
            }
            catch (const std::exception &)
            {
                continue;
            }
            if (p.fromScratchpad)
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft >= 16)
                {
                    if (srcPhys >= PS2_SCRATCHPAD_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_SCRATCHPAD_SIZE)
                        chunk = PS2_SCRATCHPAD_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    m_seenGifCopy = true;
                    m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);
                    submitGifPacket(GifPathId::Path3, m_scratchpad + srcPhys, chunk, false);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
            else
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft >= 16)
                {
                    if (srcPhys >= PS2_RAM_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_RAM_SIZE)
                        chunk = PS2_RAM_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    m_seenGifCopy = true;
                    static std::atomic<uint32_t> gifSubmits{0u};
                    const uint32_t gifProbe = gifSubmits.fetch_add(1u, std::memory_order_relaxed);
                    if (gifProbe < 64u)
                        std::cerr << "[probe:gif-submit] kind=linear bytes=0x" << std::hex
                                  << chunk << " src=0x" << srcPhys << std::dec << '\n';
                    m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);
                    submitGifPacket(GifPathId::Path3, m_rdram + srcPhys, chunk, false);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
        }
    }
    m_pendingGifTransfers.clear();

    const bool hadVif0 = !m_pendingVif0Transfers.empty();
    for (auto &p : m_pendingVif0Transfers)
    {
        if (!p.chainData.empty())
        {
            processVIF0Data(p.chainData.data(), static_cast<uint32_t>(p.chainData.size()));
        }
        else if (p.qwc > 0)
        {
            uint32_t srcPhys = 0;
            const uint64_t bytes64 = static_cast<uint64_t>(p.qwc) * 16ull;
            uint32_t sizeBytes = (bytes64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes64);
            try
            {
                srcPhys = translateAddress(p.srcAddr);
            }
            catch (const std::exception &)
            {
                continue;
            }
            if (p.fromScratchpad)
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft > 0)
                {
                    if (srcPhys >= PS2_SCRATCHPAD_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_SCRATCHPAD_SIZE)
                        chunk = PS2_SCRATCHPAD_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    processVIF0Data(m_scratchpad + srcPhys, chunk);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
            else
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft > 0)
                {
                    if (srcPhys >= PS2_RAM_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_RAM_SIZE)
                        chunk = PS2_RAM_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    processVIF0Data(srcPhys, chunk);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
        }
    }
    m_pendingVif0Transfers.clear();

    const bool hadVif1 = !m_pendingVif1Transfers.empty();
    for (auto &p : m_pendingVif1Transfers)
    {
        if (!p.chainData.empty())
        {
            processVIF1Data(p.chainData.data(), static_cast<uint32_t>(p.chainData.size()));
        }
        else if (p.qwc > 0)
        {
            uint32_t srcPhys = 0;
            const uint64_t bytes64 = static_cast<uint64_t>(p.qwc) * 16ull;
            uint32_t sizeBytes = (bytes64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes64);
            try
            {
                srcPhys = translateAddress(p.srcAddr);
            }
            catch (const std::exception &)
            {
                continue;
            }
            if (p.fromScratchpad)
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft > 0)
                {
                    if (srcPhys >= PS2_SCRATCHPAD_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_SCRATCHPAD_SIZE)
                        chunk = PS2_SCRATCHPAD_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    processVIF1Data(m_scratchpad + srcPhys, chunk);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
            else
            {
                uint32_t bytesLeft = sizeBytes;
                while (bytesLeft > 0)
                {
                    if (srcPhys >= PS2_RAM_SIZE)
                        srcPhys = 0;
                    uint32_t chunk = bytesLeft;
                    if (srcPhys + chunk > PS2_RAM_SIZE)
                        chunk = PS2_RAM_SIZE - srcPhys;
                    if (chunk == 0)
                        break;
                    processVIF1Data(srcPhys, chunk);
                    bytesLeft -= chunk;
                    srcPhys += chunk;
                }
            }
        }
    }
    m_pendingVif1Transfers.clear();

    if (m_gifArbiter)
        m_gifArbiter->drain();

    static constexpr uint32_t GIF_CHANNEL = 0x1000A000;
    static constexpr uint32_t VIF0_CHANNEL = 0x10008000;
    static constexpr uint32_t VIF1_CHANNEL = 0x10009000;
    static constexpr uint32_t D_STAT = 0x1000E010u;

    auto raiseDStatChannel = [&](uint32_t channelBit)
    {
        uint32_t dstat = m_ioRegisters.count(D_STAT) ? m_ioRegisters[D_STAT] : 0u;
        dstat |= (1u << channelBit);

        const uint32_t status = dstat & 0x3FFu;
        const uint32_t mask = (dstat >> 16) & 0x3FFu;
        if ((status & mask) != 0u)
            dstat |= (1u << 31);
        else
            dstat &= ~(1u << 31);

        m_ioRegisters[D_STAT] = dstat;
    };

    if (hadGif)
    {
        raiseDStatChannel(2u); // GIF channel
        queueCompletedDmacCause(2u);
        m_ioRegisters[GIF_CHANNEL + 0x00] &= ~0x100u;
        m_ioRegisters[GIF_CHANNEL + 0x20] = 0;
    }
    if (hadVif0)
    {
        raiseDStatChannel(0u); // VIF0 channel
        queueCompletedDmacCause(0u);
        m_ioRegisters[VIF0_CHANNEL + 0x00] &= ~0x100u;
        m_ioRegisters[VIF0_CHANNEL + 0x20] = 0;
    }
    if (hadVif1)
    {
        raiseDStatChannel(1u); // VIF1 channel
        queueCompletedDmacCause(1u);
        m_ioRegisters[VIF1_CHANNEL + 0x00] &= ~0x100u;
        m_ioRegisters[VIF1_CHANNEL + 0x20] = 0;
    }
}

void PS2Memory::queueCompletedDmacCause(uint32_t cause)
{
    std::lock_guard<std::mutex> lock(m_completedDmacMutex);
    m_completedDmacCauses.push_back(cause);
}

std::vector<uint32_t> PS2Memory::consumeCompletedDmacCauses()
{
    std::lock_guard<std::mutex> lock(m_completedDmacMutex);
    std::vector<uint32_t> causes;
    causes.swap(m_completedDmacCauses);
    return causes;
}

void PS2Memory::flushMaskedPath3Packets(bool drainImmediately)
{
    if (m_path3Masked || m_path3MaskedFifo.empty())
        return;

    auto emit = [&](const uint8_t *packetData, uint32_t packetSize)
    {
        if (m_gifArbiter)
            m_gifArbiter->submit(GifPathId::Path3, packetData, packetSize, false);
        else if (m_gifPacketCallback)
            m_gifPacketCallback(packetData, packetSize);
    };

    for (const auto &packet : m_path3MaskedFifo)
    {
        if (packet.size() >= 16u)
            emit(packet.data(), static_cast<uint32_t>(packet.size()));
    }
    m_path3MaskedFifo.clear();

    if (m_gifArbiter && drainImmediately)
        m_gifArbiter->drain();
}

void PS2Memory::submitGifPacket(GifPathId pathId, const uint8_t *data, uint32_t sizeBytes, bool drainImmediately, bool path2DirectHl)
{
    if (!data || sizeBytes < 16)
        return;

    static std::atomic<uint32_t> gifPathSubmits{0u};
    const uint32_t gifPathProbe = gifPathSubmits.fetch_add(1u, std::memory_order_relaxed);
    uint64_t probeTag = 0u;
    std::memcpy(&probeTag, data, sizeof(probeTag));
    const bool imageTag = ((probeTag >> 58u) & 0x3u) == 2u;
    if (gifPathProbe < 128u)
    {
        std::cerr << "[probe:gif-path] path=" << ((pathId == GifPathId::Path2) ? 2 : 3)
                  << " bytes=0x" << std::hex << sizeBytes
                  << " tag=0x" << probeTag
                  << " directhl=" << (path2DirectHl ? 1 : 0) << std::dec << '\n';
    }
    if (imageTag)
    {
        static std::atomic<uint32_t> imagePathProbes{0u};
        const uint32_t imageProbe = imagePathProbes.fetch_add(1u, std::memory_order_relaxed);
        if (imageProbe < 32u)
        {
            uint32_t first = 0u;
            if (sizeBytes >= 20u)
                std::memcpy(&first, data + 16u, sizeof(first));
            std::cerr << "[probe:gif-image-path] path=" << ((pathId == GifPathId::Path2) ? 2 : 3)
                      << " nloop=0x" << std::hex << (probeTag & 0x7FFFu)
                      << " bytes=0x" << sizeBytes << " first=0x" << first << std::dec << '\n';
        }
    }

    if (pathId == GifPathId::Path3)
    {
        if (m_path3Masked)
        {
            m_path3MaskedFifo.emplace_back(data, data + sizeBytes);
            return;
        }
        flushMaskedPath3Packets(false);
    }

    if (m_gifArbiter)
        m_gifArbiter->submit(pathId, data, sizeBytes, path2DirectHl);
    else if (m_gifPacketCallback)
        m_gifPacketCallback(data, sizeBytes);

    if (m_gifArbiter && drainImmediately)
        m_gifArbiter->drain();
}

void PS2Memory::processGIFPacket(uint32_t srcPhysAddr, uint32_t qwCount)
{
    if (!m_rdram || qwCount == 0)
        return;
    const uint64_t bytes64 = static_cast<uint64_t>(qwCount) * 16ull;
    uint32_t sizeBytes = (bytes64 > 0xFFFFFFFFull) ? 0xFFFFFFFFu : static_cast<uint32_t>(bytes64);
    uint32_t bytesLeft = sizeBytes;
    while (bytesLeft >= 16)
    {
        if (srcPhysAddr >= PS2_RAM_SIZE)
            srcPhysAddr = 0;
        uint32_t chunk = bytesLeft;
        if (srcPhysAddr + chunk > PS2_RAM_SIZE)
            chunk = PS2_RAM_SIZE - srcPhysAddr;
        if (chunk == 0)
            break;

        m_seenGifCopy = true;
        m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);
        submitGifPacket(GifPathId::Path3, m_rdram + srcPhysAddr, chunk);

        bytesLeft -= chunk;
        srcPhysAddr += chunk;
    }
}

void PS2Memory::processGIFPacket(const uint8_t *data, uint32_t sizeBytes)
{
    if (m_gifArbiter)
        submitGifPacket(GifPathId::Path3, data, sizeBytes);
    else if (m_gifPacketCallback && data && sizeBytes >= 16)
        m_gifPacketCallback(data, sizeBytes);
}

bool PS2Memory::tryProcessNativeGifImageUploadChain(GS &gs, uint32_t tadr, uint32_t chcr)
{
    static constexpr uint32_t GIF_CHANNEL = 0x1000A000u;
    static constexpr uint32_t D_STAT = 0x1000E010u;
    static constexpr uint32_t D_CTRL = 0x1000E000u;

    if (!m_rdram || !m_gsVRAM || m_path3Masked)
        return false;
    if (m_gifArbiter && !m_gifArbiter->empty())
        return false;
    if ((chcr & 0x100u) == 0u || ((chcr >> 2u) & 0x3u) != 1u)
        return false;
    if ((chcr & (1u << 7u)) != 0u || ((chcr >> 4u) & 0x3u) != 0u)
        return false;

    const auto dctrlIt = m_ioRegisters.find(D_CTRL);
    if (dctrlIt != m_ioRegisters.end() && ((dctrlIt->second & 0x1u) == 0u))
        return false;

    auto resolveContiguous = [&](uint32_t guestAddr, uint32_t bytes, const uint8_t *&out) -> bool
    {
        try
        {
            const bool scratch = isScratchpad(guestAddr);
            const uint32_t phys = translateAddress(guestAddr);
            const uint8_t *base = scratch ? m_scratchpad : m_rdram;
            const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
            if (!base || phys > limit || bytes > limit - phys)
                return false;
            out = base + phys;
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    };

    auto loadDmaTagAt = [&](uint32_t guestAddr, DmaTagView &out) -> bool
    {
        const uint8_t *ptr = nullptr;
        if (!resolveContiguous(guestAddr, 16u, ptr))
            return false;
        out = decodeDmaTag(loadScalar<uint64_t>(ptr, 0u, 16u, "native gif dma tag", guestAddr));
        return true;
    };

    auto decodeSetupPayload = [&](const uint8_t *payload, uint64_t (&regs)[4]) -> bool
    {
        const uint64_t tagLo = loadScalar<uint64_t>(payload, 0u, 80u, "native gif setup tag", 0u);
        const uint64_t tagHi = loadScalar<uint64_t>(payload, 8u, 80u, "native gif setup regs", 0u);
        if (gifTagNloop(tagLo) != 4u ||
            gifTagFlg(tagLo) != GIF_FMT_PACKED ||
            gifTagNreg(tagLo) != 1u ||
            (tagHi & 0xFull) != 0x0Eull)
        {
            return false;
        }

        static constexpr uint8_t kExpectedRegs[4] = {
            GS_REG_BITBLTBUF,
            GS_REG_TRXPOS,
            GS_REG_TRXREG,
            GS_REG_TRXDIR,
        };

        uint32_t offset = 16u;
        for (uint32_t i = 0; i < 4u; ++i)
        {
            regs[i] = loadScalar<uint64_t>(payload, offset, 80u, "native gif setup value", 0u);
            const uint64_t reg = loadScalar<uint64_t>(payload, offset + 8u, 80u, "native gif setup register", 0u);
            if ((reg & 0xFFu) != kExpectedRegs[i])
                return false;
            offset += 16u;
        }

        const uint32_t trxdirMode = static_cast<uint32_t>(regs[3] & 0x3ull);
        const uint32_t rrw = static_cast<uint32_t>(regs[2] & 0xFFFull);
        const uint32_t rrh = static_cast<uint32_t>((regs[2] >> 32u) & 0xFFFull);
        return trxdirMode == 0u && rrw != 0u && rrh != 0u;
    };

    DmaTagView setupTag{};
    if (!loadDmaTagAt(tadr, setupTag) ||
        setupTag.id != 1u ||
        setupTag.qwc != 5u ||
        setupTag.irq)
    {
        return false;
    }

    const uint8_t *setupPayload = nullptr;
    const uint32_t setupPayloadAddr = tadr + 16u;
    if (!resolveContiguous(setupPayloadAddr, 5u * 16u, setupPayload))
        return false;

    uint64_t setupRegs[4] = {};
    if (!decodeSetupPayload(setupPayload, setupRegs))
        return false;

    uint32_t imageTagDmaAddr = setupPayloadAddr + 5u * 16u;
    DmaTagView imageTagDma{};
    if (!loadDmaTagAt(imageTagDmaAddr, imageTagDma) ||
        imageTagDma.id != 1u ||
        imageTagDma.qwc != 1u ||
        imageTagDma.irq)
    {
        return false;
    }

    const uint8_t *imageGifTag = nullptr;
    if (!resolveContiguous(imageTagDmaAddr + 16u, 16u, imageGifTag))
        return false;

    const uint64_t imageTagLo = loadScalar<uint64_t>(imageGifTag, 0u, 16u, "native gif image tag", imageTagDmaAddr + 16u);
    if (gifTagFlg(imageTagLo) != GIF_FMT_IMAGE)
        return false;

    const uint32_t imageQwc = gifTagNloop(imageTagLo);
    if (imageQwc == 0u)
        return false;

    const uint64_t imageBytes64 = static_cast<uint64_t>(imageQwc) * 16ull;
    if (imageBytes64 > 0xFFFFFFFFull)
        return false;
    const uint32_t imageBytes = static_cast<uint32_t>(imageBytes64);

    const uint32_t payloadTagAddr = imageTagDmaAddr + 32u;
    DmaTagView payloadTag{};
    if (!loadDmaTagAt(payloadTagAddr, payloadTag) ||
        payloadTag.qwc != imageQwc ||
        payloadTag.irq)
    {
        return false;
    }

    uint32_t imageDataAddr = 0u;
    uint32_t finalTadr = payloadTagAddr;
    uint32_t lastTagUpper = payloadTag.upper;
    if (payloadTag.id == 3u || payloadTag.id == 4u)
    {
        imageDataAddr = payloadTag.addr;
        const uint32_t terminalTagAddr = payloadTagAddr + 16u;
        DmaTagView terminalTag{};
        if (!loadDmaTagAt(terminalTagAddr, terminalTag) ||
            terminalTag.qwc != 0u ||
            terminalTag.irq ||
            (terminalTag.id != 0u && terminalTag.id != 7u))
        {
            return false;
        }
        finalTadr = (terminalTag.id == 0u) ? (terminalTagAddr + 16u) : terminalTagAddr;
        lastTagUpper = terminalTag.upper;
    }
    else if (payloadTag.id == 7u)
    {
        imageDataAddr = payloadTagAddr + 16u;
        finalTadr = payloadTagAddr;
    }
    else
    {
        return false;
    }

    const uint8_t *imageData = nullptr;
    if (!resolveContiguous(imageDataAddr, imageBytes, imageData))
        return false;

    m_dmaStartCount.fetch_add(1, std::memory_order_relaxed);
    m_seenGifCopy = true;
    m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);
    gs.uploadImageNative(setupRegs[0], setupRegs[1], setupRegs[2], setupRegs[3], imageData, imageBytes);

    m_ioRegisters[GIF_CHANNEL + 0x30u] = finalTadr;
    m_ioRegisters[GIF_CHANNEL + 0x40u] = 0u;
    m_ioRegisters[GIF_CHANNEL + 0x50u] = 0u;
    m_ioRegisters[GIF_CHANNEL + 0x00u] = ((chcr & 0x0000FFFFu) | (lastTagUpper << 16u)) & ~0x100u;
    m_ioRegisters[GIF_CHANNEL + 0x20u] = 0u;

    uint32_t dstat = m_ioRegisters.count(D_STAT) ? m_ioRegisters[D_STAT] : 0u;
    dstat |= (1u << 2u);
    const uint32_t status = dstat & 0x3FFu;
    const uint32_t mask = (dstat >> 16u) & 0x3FFu;
    if ((status & mask) != 0u)
        dstat |= (1u << 31u);
    else
        dstat &= ~(1u << 31u);
    m_ioRegisters[D_STAT] = dstat;
    queueCompletedDmacCause(2u);
    return true;
}

bool PS2Memory::tryProcessNativeGifPackedChain(GS &gs, uint32_t tadr, uint32_t chcr)
{
    static constexpr uint32_t GIF_CHANNEL = 0x1000A000u;
    static constexpr uint32_t D_STAT = 0x1000E010u;
    static constexpr uint32_t D_CTRL = 0x1000E000u;

    if (!m_rdram || !m_gsVRAM || m_path3Masked)
        return false;
    if (m_gifArbiter && !m_gifArbiter->empty())
        return false;
    if ((chcr & 0x100u) == 0u || ((chcr >> 2u) & 0x3u) != 1u)
        return false;
    if ((chcr & (1u << 7u)) != 0u || ((chcr >> 4u) & 0x3u) != 0u)
        return false;

    const auto dctrlIt = m_ioRegisters.find(D_CTRL);
    if (dctrlIt != m_ioRegisters.end() && ((dctrlIt->second & 0x1u) == 0u))
        return false;

    auto resolveContiguous = [&](uint32_t guestAddr, uint32_t bytes, const uint8_t *&out) -> bool
    {
        try
        {
            const bool scratch = isScratchpad(guestAddr);
            const uint32_t phys = translateAddress(guestAddr);
            const uint8_t *base = scratch ? m_scratchpad : m_rdram;
            const uint32_t limit = scratch ? PS2_SCRATCHPAD_SIZE : PS2_RAM_SIZE;
            if (!base || phys > limit || bytes > limit - phys)
                return false;
            out = base + phys;
            return true;
        }
        catch (const std::exception &)
        {
            return false;
        }
    };

    const uint8_t *tagPtr = nullptr;
    if (!resolveContiguous(tadr, 16u, tagPtr))
        return false;

    const DmaTagView tag = decodeDmaTag(loadScalar<uint64_t>(tagPtr, 0u, 16u, "native packed gif dma tag", tadr));
    if (tag.id != 7u || tag.qwc == 0u || tag.irq)
        return false;

    const uint64_t payloadBytes64 = static_cast<uint64_t>(tag.qwc) * 16ull;
    if (payloadBytes64 > 0xFFFFFFFFull)
        return false;
    const uint32_t payloadBytes = static_cast<uint32_t>(payloadBytes64);

    const uint8_t *payload = nullptr;
    if (!resolveContiguous(tadr + 16u, payloadBytes, payload))
        return false;
    if (!gs.processNativePackedGIFPacket(payload, payloadBytes))
        return false;

    m_dmaStartCount.fetch_add(1, std::memory_order_relaxed);
    m_seenGifCopy = true;
    m_gifCopyCount.fetch_add(1, std::memory_order_relaxed);

    m_ioRegisters[GIF_CHANNEL + 0x30u] = tadr;
    m_ioRegisters[GIF_CHANNEL + 0x40u] = 0u;
    m_ioRegisters[GIF_CHANNEL + 0x50u] = 0u;
    m_ioRegisters[GIF_CHANNEL + 0x00u] = ((chcr & 0x0000FFFFu) | (tag.upper << 16u)) & ~0x100u;
    m_ioRegisters[GIF_CHANNEL + 0x20u] = 0u;

    uint32_t dstat = m_ioRegisters.count(D_STAT) ? m_ioRegisters[D_STAT] : 0u;
    dstat |= (1u << 2u);
    const uint32_t status = dstat & 0x3FFu;
    const uint32_t mask = (dstat >> 16u) & 0x3FFu;
    if ((status & mask) != 0u)
        dstat |= (1u << 31u);
    else
        dstat &= ~(1u << 31u);
    m_ioRegisters[D_STAT] = dstat;
    queueCompletedDmacCause(2u);
    return true;
}

int PS2Memory::pollDmaRegisters()
{
    return 0;
}

uint32_t PS2Memory::readIORegister(uint32_t address)
{
    size_t timerIndex = 0u;
    uint32_t timerOffset = 0u;
    if (decodeEeTimerRegister(address, timerIndex, timerOffset))
    {
        const EeTimer &timer = m_eeTimers[timerIndex];
        switch (timerOffset)
        {
        case kEeTimerCountOffset:
            return timer.count & 0xFFFFu;
        case kEeTimerModeOffset:
            return timer.mode & (kEeTimerModeConfigMask | kEeTimerModeStatusMask);
        case kEeTimerCompareOffset:
            return timer.compare & 0xFFFFu;
        case kEeTimerHoldOffset:
            return timer.hold & 0xFFFFu;
        default:
            return 0u;
        }
    }

    if (isGsPrivReg(address))
    {
        // NB: unreachable from read8/16/32/64 today, same reasoning as the write
        // path above; kept correct for direct callers.
        const uint32_t off = address & 7u;
        const uint32_t regOff = (address - PS2_GS_PRIV_REG_BASE) & ~0x7u;
        if (regOff == kGsCsrRegOffset)
        {
            return static_cast<uint32_t>((gs_regs.csr.load() >> (off * 8u)) & 0xFFFFFFFFull);
        }
        if (uint64_t *reg = gsRegPtr(gs_regs, address))
        {
            return static_cast<uint32_t>((*reg >> (off * 8u)) & 0xFFFFFFFFull);
        }
        return 0u;
    }

    if (address >= 0x10002000 && address <= 0x10002030)
    {
        uint32_t val = 0;
        observeIpuRead();
        switch (address)
        {
        case 0x10002000:
            val = m_ipu.cmdData;
            break;
        case 0x10002010:
            val = readIpuCtrl();
            break;
        case 0x10002020:
            val = readIpuBp();
            break;
        case 0x10002030:
            // FDEC/VDEC publish the same look-ahead word through TOP.  The
            // Katamari stream parser reads TOP after CMD completion.
            val = m_ipu.topData;
            break;
        default:
            val = 0;
            break;
        }
        return val;
    }

    if (address == 0x10003020u) // GIF_STAT
    {
        uint32_t stat = m_ioRegisters.count(address) ? m_ioRegisters[address] : 0u;
        const uint32_t mode = m_ioRegisters.count(0x10003010u) ? m_ioRegisters[0x10003010u] : 0u;
        const uint32_t ctrl = m_ioRegisters.count(0x10003000u) ? m_ioRegisters[0x10003000u] : 0u;

        // M3R and IMT mirror GIF_MODE, PSE mirrors GIF_CTRL, and M3P is the
        // effective PATH3 mask controlled by the VIF1 MSKPATH3 command.
        stat = (stat & ~0xFu) |
               (mode & 0x1u) |
               (m_path3Masked ? 0x2u : 0u) |
               (mode & 0x4u) |
               (ctrl & 0x8u);
        return stat;
    }

    if (address >= 0x10000000 && address < 0x10010000)
    {
        if (address >= 0x10008000 && address < 0x1000F000)
        {
            if ((address & 0xFF) == 0x00)
            {
                // IPU DMA remains active while a FIFO temporarily blocks the
                // transfer.  Merely polling CHCR must not acknowledge/finish
                // it; runIpuInDma/runIpuOutDma clear STR at the real end.
                if (address == 0x1000B000u || address == 0x1000B400u)
                    return m_ioRegisters[address];
                uint32_t channelStatus = m_ioRegisters[address] & ~0x100u;
                m_ioRegisters[address] = channelStatus;
                return channelStatus;
            }
        }

        if (address >= 0x10000200 && address < 0x10000300)
        {
            return 0;
        }

        if (address >= 0x1000F200 && address <= 0x1000F260)
        {
            if (address == 0x1000F230)
            {
                return 0x60000;
            }
            if (address == 0x1000F240)
            {
                return 0xF0000002;
            }
            return 0;
        }
    }

    auto it = m_ioRegisters.find(address);
    if (it != m_ioRegisters.end())
    {
        return it->second;
    }

    return 0;
}

void PS2Memory::registerCodeRegion(uint32_t start, uint32_t end)
{
    if (end <= start)
    {
        std::cerr << "Ignoring invalid code region: start=0x" << std::hex << start
                  << " end=0x" << end << std::dec << std::endl;
        return;
    }

    if ((end - start) > PS2_RAM_SIZE)
    {
        std::cerr << "Ignoring oversized code region: start=0x" << std::hex << start
                  << " end=0x" << end << std::dec << std::endl;
        return;
    }

    for (const auto &existing : m_codeRegions)
    {
        if (existing.start == start && existing.end == end)
        {
            return;
        }
    }

    CodeRegion region;
    region.start = start;
    region.end = end;

    size_t sizeInWords = (end - start + 3u) / 4u;
    region.modified.resize(sizeInWords, false);

    m_codeRegions.push_back(region);
    RUNTIME_LOG("Registered code region: " << std::hex << start << " - " << end << std::dec);
}

bool PS2Memory::isAddressInRegion(uint32_t address, const CodeRegion &region)
{
    return (address >= region.start && address < region.end);
}

bool PS2Memory::isCodeAddress(uint32_t address) const
{
    for (const auto &region : m_codeRegions)
    {
        if (address >= region.start && address < region.end)
        {
            return true;
        }
    }
    return false;
}

void PS2Memory::markModified(uint32_t address, uint32_t size)
{
    if (size == 0)
    {
        return;
    }

    const uint64_t writeEnd = static_cast<uint64_t>(address) + static_cast<uint64_t>(size);
    for (auto &region : m_codeRegions)
    {
        const uint64_t regionStart = region.start;
        const uint64_t regionEnd = region.end;
        if (writeEnd <= regionStart || static_cast<uint64_t>(address) >= regionEnd)
        {
            continue;
        }

        uint32_t overlapStart = static_cast<uint32_t>(std::max<uint64_t>(address, regionStart));
        uint32_t overlapEnd = static_cast<uint32_t>(std::min<uint64_t>(writeEnd, regionEnd));

        for (uint32_t addr = overlapStart; addr < overlapEnd; addr += 4)
        {
            size_t bitIndex = (addr - region.start) / 4;
            if (bitIndex < region.modified.size())
            {
                region.modified[bitIndex] = true;
                RUNTIME_LOG("Marked code at " << std::hex << addr << std::dec << " as modified");
            }
        }
    }
}

bool PS2Memory::isCodeModified(uint32_t address, uint32_t size)
{
    if (size == 0)
    {
        return false;
    }

    const uint64_t writeEnd = static_cast<uint64_t>(address) + static_cast<uint64_t>(size);
    for (const auto &region : m_codeRegions)
    {
        const uint64_t regionStart = region.start;
        const uint64_t regionEnd = region.end;
        if (writeEnd <= regionStart || static_cast<uint64_t>(address) >= regionEnd)
        {
            continue;
        }

        uint32_t overlapStart = static_cast<uint32_t>(std::max<uint64_t>(address, regionStart));
        uint32_t overlapEnd = static_cast<uint32_t>(std::min<uint64_t>(writeEnd, regionEnd));

        for (uint32_t addr = overlapStart; addr < overlapEnd; addr += 4)
        {
            size_t bitIndex = (addr - region.start) / 4;
            if (bitIndex < region.modified.size() && region.modified[bitIndex])
            {
                return true; // Found modified code
            }
        }
    }

    return false; // No modifications found
}

void PS2Memory::clearModifiedFlag(uint32_t address, uint32_t size)
{
    if (size == 0)
    {
        return;
    }

    const uint64_t writeEnd = static_cast<uint64_t>(address) + static_cast<uint64_t>(size);
    for (auto &region : m_codeRegions)
    {
        const uint64_t regionStart = region.start;
        const uint64_t regionEnd = region.end;
        if (writeEnd <= regionStart || static_cast<uint64_t>(address) >= regionEnd)
        {
            continue;
        }

        uint32_t overlapStart = static_cast<uint32_t>(std::max<uint64_t>(address, regionStart));
        uint32_t overlapEnd = static_cast<uint32_t>(std::min<uint64_t>(writeEnd, regionEnd));

        for (uint32_t addr = overlapStart; addr < overlapEnd; addr += 4)
        {
            size_t bitIndex = (addr - region.start) / 4;
            if (bitIndex < region.modified.size())
            {
                region.modified[bitIndex] = false;
            }
        }
    }
}
