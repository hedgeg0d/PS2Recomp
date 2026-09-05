#include "runtime/ee_scheduler.h"

#include "ps2_log.h"
#include "ps2_runtime_macros.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
    uint64_t katamariStreamHash(const uint8_t *rdram, uint32_t base)
    {
        uint64_t hash = 1469598103934665603ull;
        for (uint32_t i = 0u; i < 0x20000u; ++i)
        {
            hash ^= rdram[base + i];
            hash *= 1099511628211ull;
        }
        return hash;
    }

    bool g_katamariFullMonitorArmed = false;
    uint64_t g_katamariPreviousFull6 = 0u;
    uint64_t g_katamariPreviousFullC = 0u;
    std::array<uint8_t, 0x20000u> g_katamariBaseline6{};
    std::array<uint8_t, 0x20000u> g_katamariBaselineC{};

    constexpr int KE_OK = 0;
    constexpr int KE_ERROR = -1;
    constexpr int KE_ILLEGAL_PRIORITY = -403;
    constexpr int KE_ILLEGAL_THID = -406;
    constexpr int KE_UNKNOWN_THID = -407;
    constexpr int KE_UNKNOWN_SEMID = -408;
    constexpr int KE_UNKNOWN_EVFID = -409;
    constexpr int KE_DORMANT = -413;
    constexpr int KE_NOT_DORMANT = -414;
    constexpr int KE_NOT_SUSPEND = -415;
    constexpr int KE_NOT_WAIT = -416;
    constexpr int KE_RELEASE_WAIT = -418;
    constexpr int KE_SEMA_ZERO = -419;
    constexpr int KE_SEMA_OVF = -420;
    constexpr int KE_EVF_COND = -421;
    constexpr int KE_WAIT_DELETE = -425;

    constexpr uint32_t WEF_OR = 0x01u;
    constexpr uint32_t WEF_CLEAR = 0x10u;
    constexpr uint32_t WEF_CLEAR_ALL = 0x20u;
    constexpr auto kVBlankPeriod = std::chrono::microseconds(16667);
    constexpr auto kVBlankDuration = std::chrono::microseconds(500);
    constexpr uint64_t kAlarmTickMicroseconds = 64u;
    constexpr uint32_t kDebugPublishDispatchInterval = 4096u;

    constexpr uint64_t microsecondsToEeCycles(uint64_t microseconds)
    {
        return (microseconds * EeScheduler::kEeClockHz + 999999ull) / 1000000ull;
    }

    std::chrono::nanoseconds eeCyclesToHostDuration(uint64_t cycles)
    {
        constexpr uint64_t kNanosecondsPerSecond = 1000000000ull;
        const uint64_t wholeSeconds = cycles / EeScheduler::kEeClockHz;
        const uint64_t remainingCycles = cycles % EeScheduler::kEeClockHz;
        const uint64_t remainingNanoseconds = (remainingCycles * kNanosecondsPerSecond + EeScheduler::kEeClockHz - 1u) / EeScheduler::kEeClockHz;
        return std::chrono::seconds(wholeSeconds) + std::chrono::nanoseconds(remainingNanoseconds);
    }

    constexpr uint64_t kVBlankPeriodCycles = microsecondsToEeCycles(16667u);
    constexpr uint64_t kVBlankDurationCycles = microsecondsToEeCycles(500u);
    constexpr uint64_t kAlarmTickCycles = microsecondsToEeCycles(kAlarmTickMicroseconds);

    template <typename Map>
    int allocatePositiveId(int &nextId, const Map &objects)
    {
        const int first = std::max(1, nextId);
        int candidate = first;
        do
        {
            if (!objects.contains(candidate))
            {
                nextId = (candidate == std::numeric_limits<int>::max()) ? 1 : candidate + 1;
                return candidate;
            }
            candidate = (candidate == std::numeric_limits<int>::max()) ? 1 : candidate + 1;
        } while (candidate != first);
        return 0;
    }
}

EeScheduler::EeScheduler(PS2Runtime &runtime)
    : m_runtime(runtime)
{
}

EeScheduler::~EeScheduler()
{
    requestStop();
}

void EeScheduler::reset(uint8_t *rdram, const R5900Context &mainContext)
{
    m_executorThread = std::this_thread::get_id();
    m_rdram = rdram;
    m_readyQueues = {};
    m_threads.clear();
    m_semaphores.clear();
    m_eventFlags.clear();
    m_alarms.clear();
    m_intcHandlers.clear();
    m_dmacHandlers.clear();
    m_nextThreadId = kFirstThreadId;
    m_nextInvocationThreadId = -1;
    m_nextSemaphoreId = 1;
    m_nextEventFlagId = 1;
    m_nextAlarmId = 1;
    m_nextIntcHandlerId = 1;
    m_nextDmacHandlerId = 1;
    m_intcHeadOrder = 0;
    m_intcTailOrder = 1000;
    m_dmacHeadOrder = 0;
    m_dmacTailOrder = 1000;
    m_enabledIntcMask = 0xFFFFFFFFu;
    m_enabledDmacMask = 0xFFFFFFFFu;
    m_currentThreadId = 0;
    m_rescheduleRequested = false;
    m_timeSliceExpired = false;
    m_insideInterrupt = false;
    m_pendingEeTimerInterrupts = 0u;
    m_eeCycle = 0u;
    m_sliceEndCycle = kDefaultTimeSliceCycles;
    m_stopRequested.store(false, std::memory_order_release);
    m_checkpointPending.store(false, std::memory_order_release);
    m_debugPublishCountdown = 0u;
    {
        std::lock_guard lock(m_eventMutex);
        m_events.clear();
        m_deadlines.clear();
        m_pendingInvocations.clear();
    }
    m_eventSequence = 0;
    m_invocationSequence = 0;
    m_invocationStackTops.clear();
    m_vsyncTick = 0;
    m_vsyncFlagAddress = 0;
    m_vsyncTickAddress = 0;
    m_gsVSyncCallback = 0;
    m_gsVSyncCallbackGp = 0;
    m_gsVSyncCallbackSp = 0;
    m_runtime.memory().gs().vsyncTick.store(0u, std::memory_order_release);
    m_runtime.memory().resetEeTimers();

    GuestThread main{};
    main.id = kMainThreadId;
    main.context = mainContext;
    main.entry = mainContext.pc;
    // $sp is live execution state, not the stable initial stack descriptor
    // returned by ReferThreadStatus. SetupThread records that metadata.
    main.stack = 0u;
    main.gp = getRegU32(&mainContext, 28);
    // The EE bootstrap thread is created at kernel priority 0 on hardware.
    // Keeping that priority also lets it time-slice fairly with the SIF/RPC
    // service thread, instead of a priority-0 service loop starving boot.
    main.initialPriority = 0;
    main.currentPriority = 0;
    main.status = EeThreadStatus::Ready;
    m_threads.emplace(main.id, std::move(main));
    m_readyQueues[0].push_back(kMainThreadId);
    scheduleEvent(m_eeCycle + kVBlankPeriodCycles,
                  std::chrono::steady_clock::now() + kVBlankPeriod,
                  EeEvent{EeEventType::VBlankStart, 0, 0});
    publishSnapshot();
}

void EeScheduler::run()
{
    assertExecutor();
    m_running.store(true, std::memory_order_release);

    while (!m_stopRequested.load(std::memory_order_acquire))
    {
        processPendingEvents();
        if (m_stopRequested.load(std::memory_order_acquire))
        {
            break;
        }

        if (m_currentThreadId == 0)
        {
            GuestThread *next = selectReady();
            if (!next && m_pendingInvocations.empty())
            {
                publishSnapshot();
                waitForEvent();
                continue;
            }
            if (next)
            {
                makeRunning(*next);
            }
            else
            {
                GuestThread *owner = &acquireInvocationThread();
                GuestInvocation invocation = std::move(m_pendingInvocations.front());
                m_pendingInvocations.pop_front();
                owner->status = EeThreadStatus::Running;
                m_currentThreadId = owner->id;
                renewTimeSlice();
                if (invocation.kind == GuestInvocationKind::Interrupt ||
                    getRegU32(&invocation.context, 29) == 0u)
                {
                    SET_GPR_U32(&invocation.context, 29, invocationStackTop());
                }
                owner->invocations.push_back(std::move(invocation));
            }
        }

        GuestThread *running = currentThread();
        assert(running != nullptr);
        if (running->resumeCompletion)
        {
            auto completion = std::move(running->resumeCompletion);
            running->resumeCompletion = {};
            try
            {
                completion(running->activeContext());
            }
            catch (const EeDispatcherTransfer &)
            {
            }
            if (m_currentThreadId == 0)
            {
                continue;
            }
        }
        R5900Context &context = running->activeContext();
        if (m_debugPublishCountdown == 0u)
        {
            copyMainContextToRuntime();
            publishSnapshot();
            m_debugPublishCountdown = kDebugPublishDispatchInterval - 1u;
        }
        else
        {
            --m_debugPublishCountdown;
        }

        m_runtime.m_debugPc.store(context.pc, std::memory_order_relaxed);
        m_runtime.m_debugRa.store(getRegU32(&context, 31), std::memory_order_relaxed);
        m_runtime.m_debugSp.store(getRegU32(&context, 29), std::memory_order_relaxed);
        m_runtime.m_debugGp.store(getRegU32(&context, 28), std::memory_order_relaxed);

        if (context.pc == 0u)
        {
            if (!running->invocations.empty())
            {
                GuestInvocation completed = std::move(running->invocations.back());
                running->invocations.pop_back();
                if (completed.onComplete)
                {
                    try
                    {
                        completed.onComplete(completed.context, running->activeContext());
                    }
                    catch (const EeDispatcherTransfer &)
                    {
                    }
                }
                // A burst of IRQ completions must not monopolize the owner
                // thread.  Once one interrupt frame returns, yield the owner
                // when more callbacks are queued so normal ready threads can
                // make progress before the next asynchronous delivery.
                if (completed.kind == GuestInvocationKind::Interrupt &&
                    !m_pendingInvocations.empty())
                {
                    // Dispatcher-only owners are not runnable guest threads;
                    // leave the pseudo-thread dormant so a pending IRQ does
                    // not continually win the ready queue over real EE work.
                    if (running->id < 0)
                    {
                        makeDormant(*running);
                    }
                    else
                    {
                        enqueueReady(*running, false);
                    }
                    m_currentThreadId = 0;
                }
                continue;
            }
            makeDormant(*running);
            m_currentThreadId = 0;
            continue;
        }

        // Do not preempt an active interrupt callback with another pending
        // invocation.  IRQ dispatch queues completions raised by a callback;
        // pushing them here would recursively grow the guest callback stack
        // before the current handler reaches its JR $ra.  Retain the queue
        // until the interrupt frame completes, then service it normally.
        const bool activeInterrupt = !running->invocations.empty() &&
                                     running->invocations.back().kind == GuestInvocationKind::Interrupt;
        if (!m_pendingInvocations.empty() && !activeInterrupt)
        {
            GuestInvocation invocation = std::move(m_pendingInvocations.front());
            m_pendingInvocations.pop_front();
            if (invocation.kind == GuestInvocationKind::Interrupt ||
                getRegU32(&invocation.context, 29) == 0u)
            {
                SET_GPR_U32(&invocation.context, 29, invocationStackTop());
            }
            running->invocations.push_back(std::move(invocation));
            continue;
        }

        if (!m_runtime.hasFunction(context.pc))
        {
            if (!running->invocations.empty())
            {
                context.pc = 0u;
            }
            else
            {
                m_runtime.reportMissingFunction(m_rdram,
                                                &context,
                                                context.pc,
                                                context.pc,
                                                PS2Runtime::GuestBranchKind::DirectJump,
                                                "EE scheduler");
                if (context.pc == 0x54DAC0u)
                {
                    std::cerr << "[probe:stack54] sp=0x" << std::hex << getRegU32(&context, 29)
                              << " words=";
                    const uint32_t sp = getRegU32(&context, 29);
                    for (int32_t off = -0x40; off < 0x80; off += 8)
                    {
                        const uint32_t addr = (sp + static_cast<uint32_t>(off)) & PS2_RAM_MASK;
                        std::cerr << " " << (off >= 0 ? "+" : "") << off << ":" << Ps2FastRead32(m_rdram, addr)
                                  << "," << Ps2FastRead32(m_rdram, addr + 4);
                    }
                    std::cerr << std::dec << std::endl;
                }
                makeDormant(*running);
                m_currentThreadId = 0;
            }
            continue;
        }
        PS2Runtime::RecompiledFunction function = m_runtime.lookupFunction(context.pc);

        if (checkpointDue(kGuestDispatchCycles))
        {
            continue;
        }

        const uint32_t executingPc = context.pc;
        // Temporary Katamari boot trace: keep the first few scheduler slices
        // visible so a task-registration loop can be localized without
        // instrumenting generated game code.  This is bounded and side-effect
        // free; remove once the generic scheduler issue is resolved.
        {
            static uint32_t bootPcTraceCount = 0u;
            if (bootPcTraceCount < 320u)
            {
                std::cerr << "[probe:sched-pc] tid=" << running->id
                          << " pc=0x" << std::hex << executingPc
                          << " ra=0x" << getRegU32(&context, 31)
                          << " sp=0x" << getRegU32(&context, 29)
                          << std::dec << '\n';
                ++bootPcTraceCount;
            }
        }
        if (executingPc == 0x1038140u || executingPc == 0x10381A0u ||
            executingPc == 0x10382B0u || executingPc == 0x10382DCu ||
            executingPc == 0x103836Cu || executingPc == 0x1673D8u ||
            executingPc == 0x1673F0u || executingPc == 0x1673F8u ||
            executingPc == 0x167400u || executingPc == 0x167408u)
        {
            static uint32_t initPathProbeCount = 0u;
            if (initPathProbeCount < 160u)
            {
                std::cerr << "[probe:init-path] tid=" << running->id
                          << " pc=0x" << std::hex << executingPc
                          << " next=0x" << context.pc
                          << " ra=0x" << getRegU32(&context, 31)
                          << " sp=0x" << getRegU32(&context, 29)
                          << " ready=" << static_cast<unsigned>(m_rdram[0x101D394u])
                          << " busy=" << static_cast<unsigned>(m_rdram[0x101DB77u])
                          << std::dec << '\n';
                ++initPathProbeCount;
            }
        }
        try
        {
            // Keep a bounded view of the MPEG/resource worker after the
            // initial refill.  The worker is split across several generated
            // entry points (0x188010..0x1881c8), so logging only one entry
            // misses the cross-slice continuation at EOF.
            if (running->id == 3)
            {
                static uint32_t decoderBoundaryProbeCount = 0u;
                if (decoderBoundaryProbeCount < 160u)
                {
                    std::cerr << "[probe:decoder-boundary] pc=0x" << std::hex << executingPc
                              << " next=0x" << context.pc
                              << " state08=0x" << Ps2FastRead32(m_rdram, 0x101D448u)
                              << " state0c=0x" << Ps2FastRead32(m_rdram, 0x101D44Cu)
                              << " state10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " state14=0x" << Ps2FastRead32(m_rdram, 0x101D454u)
                              << " state18=0x" << Ps2FastRead32(m_rdram, 0x101D458u)
                              << " state1c=0x" << Ps2FastRead16(m_rdram, 0x101D45Cu)
                              << " status=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << std::dec;
                    const uint32_t out = Ps2FastRead32(m_rdram, 0x101D44Cu) & PS2_RAM_MASK;
                    uint64_t hash = 1469598103934665603ull;
                    uint32_t nonzero = 0u;
                    if (out < PS2_RAM_SIZE)
                    {
                        const uint32_t bytes = std::min<uint32_t>(0x2000u, PS2_RAM_SIZE - out);
                        for (uint32_t i = 0u; i < bytes; ++i)
                        {
                            const uint8_t b = m_rdram[out + i];
                            hash ^= b;
                            hash *= 1099511628211ull;
                            nonzero += (b != 0u) ? 1u : 0u;
                        }
                    }
                    std::cerr << " outHash=0x" << std::hex << hash
                              << " outNonzero=0x" << nonzero << std::dec << '\n';
                    ++decoderBoundaryProbeCount;
                }
            }
            // Temporary decoder-state probe: the guest MPEG worker is a
            // normal EE thread, so its entry bypasses dispatchGuestBranch.
            // Keep this diagnostic in the scheduler while tracing the
            // generic async-I/O contract; it does not alter guest state.
            if (executingPc == 0x188010u || executingPc == 0x188028u || executingPc == 0x188034u)
            {
                static uint32_t decoderProbeCount = 0u;
                if (decoderProbeCount < 64u)
                {
                    const uint32_t state = 0x101D440u;
                    std::cerr << "[probe:decoder-thread] pc=0x" << std::hex << executingPc
                              << " status=0x" << running->context.cop0_status
                              << " state+08=0x" << std::hex
                              << Ps2FastRead32(m_rdram, state + 0x08u)
                              << " +0c=0x" << Ps2FastRead32(m_rdram, state + 0x0Cu)
                              << " +10=0x" << Ps2FastRead32(m_rdram, state + 0x10u)
                              << " +14=0x" << Ps2FastRead32(m_rdram, state + 0x14u)
                              << " +18=0x" << Ps2FastRead32(m_rdram, state + 0x18u)
                              << " +1c=0x" << Ps2FastRead32(m_rdram, state + 0x1Cu)
                              << " +1e=0x" << static_cast<unsigned>(m_rdram[state + 0x1Eu])
                              << " input=";
                    for (uint32_t i = 0; i < 16u; ++i)
                    {
                        std::cerr << (i == 0u ? "" : " ")
                                  << static_cast<unsigned>(m_rdram[0x1E4D080u + i]);
                    }
                    std::cerr << " buf=";
                    for (uint32_t i = 0; i < 8u; ++i)
                    {
                        std::cerr << static_cast<unsigned>(m_rdram[0x1038140u + i]) << ' ';
                    }
                    std::cerr << std::dec << '\n';
                    ++decoderProbeCount;
                }
            }
            if (executingPc == 0x1A23B0u)
            {
                static uint32_t streamWriterCallProbe = 0u;
                if (streamWriterCallProbe < 16u)
                {
                    std::cerr << "[probe:stream-writer-call] pc=0x" << std::hex << executingPc
                              << " sp=0x" << getRegU32(&context, 29)
                              << " ra=0x" << getRegU32(&context, 31)
                              << " a0=0x" << getRegU32(&context, 4)
                              << " a1=0x" << getRegU32(&context, 5)
                              << " a2=0x" << getRegU32(&context, 6)
                              << " a3=0x" << getRegU32(&context, 7)
                              << " s0=0x" << getRegU32(&context, 16)
                              << " s1=0x" << getRegU32(&context, 17)
                              << " s2=0x" << getRegU32(&context, 18)
                              << " s3=0x" << getRegU32(&context, 19)
                              << " global1021198=0x" << Ps2FastRead32(m_rdram, 0x1021198u)
                              << std::dec << '\n';
                    ++streamWriterCallProbe;
                }
            }
            m_insideInterrupt = !running->invocations.empty() && running->invocations.back().kind == GuestInvocationKind::Interrupt;
            m_guestExecuting.store(true, std::memory_order_release);

            // Temporary stream-integrity probe.  IOP transfers are exact at
            // the adapter boundary, so sample the two decoder segments around
            // every EE entry to identify a later guest writer.  The probe is
            // deliberately local to this .cpp and uses no generated headers.
            {
                constexpr uint32_t kStreamBase = 0x1E4D080u;
                constexpr uint32_t kSegmentSize = 0x20000u;
                auto sampleSegment = [&](uint32_t base) {
                    uint64_t hash = 1469598103934665603ull;
                    for (uint32_t off = 0u; off < kSegmentSize; off += 0x1000u)
                    {
                        for (uint32_t i = 0u; i < 16u; ++i)
                        {
                            hash ^= m_rdram[base + off + i];
                            hash *= 1099511628211ull;
                        }
                    }
                    return hash;
                };
                auto hasPrefix = [&](uint32_t base, std::initializer_list<uint8_t> bytes) {
                    uint32_t i = 0u;
                    for (uint8_t expected : bytes)
                    {
                        if (m_rdram[base + i++] != expected)
                            return false;
                    }
                    return true;
                };
                static bool monitorSeg6 = false;
                static bool monitorSegC = false;
                static uint64_t previousSeg6 = 0u;
                static uint64_t previousSegC = 0u;
                const uint32_t seg6 = kStreamBase + 6u * kSegmentSize;
                const uint32_t segC = kStreamBase + 0xCu * kSegmentSize;
                if (!monitorSeg6 && hasPrefix(seg6, {0x04, 0x11, 0xB0, 0xF0, 0x41, 0xA2, 0xFE, 0x02,
                                                     0x2F, 0x28, 0x24, 0x31, 0x03, 0xAF, 0x43, 0xE8}))
                {
                    monitorSeg6 = true;
                    previousSeg6 = sampleSegment(seg6);
                    std::cerr << "[probe:stream-monitor] armed seg=6 hash=0x" << std::hex
                              << previousSeg6 << std::dec << '\n';
                }
                if (!monitorSegC && hasPrefix(segC, {0x44, 0x3D, 0x40, 0x01, 0x08, 0xF9, 0x96, 0xD0,
                                                     0xBF, 0x84, 0xE3, 0x16, 0xD4, 0x22, 0x18, 0xF0}))
                {
                    monitorSegC = true;
                    previousSegC = sampleSegment(segC);
                    std::cerr << "[probe:stream-monitor] armed seg=c hash=0x" << std::hex
                              << previousSegC << std::dec << '\n';
                }
                if (monitorSeg6)
                {
                    const uint64_t current = sampleSegment(seg6);
                    if (current != previousSeg6)
                    {
                        std::cerr << "[probe:stream-modify] seg=6 entry=0x" << std::hex
                                  << executingPc << " old=0x" << previousSeg6 << " new=0x" << current
                                  << std::dec << '\n';
                        previousSeg6 = current;
                    }
                }
                if (monitorSegC)
                {
                    const uint64_t current = sampleSegment(segC);
                    if (current != previousSegC)
                    {
                        std::cerr << "[probe:stream-modify] seg=c entry=0x" << std::hex
                                  << executingPc << " old=0x" << previousSegC << " new=0x" << current
                                  << std::dec << '\n';
                        previousSegC = current;
                    }
                }
            }
            {
                // Once segment C is present all IOP chunks have arrived.
                // Hash both complete segments at the function boundary; this
                // catches writes that sparse sampling can miss and attributes
                // the change to the next guest entry without touching headers.
                constexpr uint32_t streamBase = 0x1E4D080u;
                const uint32_t seg6 = streamBase + 6u * 0x20000u;
                const uint32_t segC = streamBase + 0xCu * 0x20000u;
                if (!g_katamariFullMonitorArmed && m_rdram[segC] == 0x44u && m_rdram[segC + 1u] == 0x3Du &&
                    m_rdram[segC + 2u] == 0x40u && m_rdram[segC + 3u] == 0x01u)
                {
                    g_katamariFullMonitorArmed = true;
                    g_katamariPreviousFull6 = katamariStreamHash(m_rdram, seg6);
                    g_katamariPreviousFullC = katamariStreamHash(m_rdram, segC);
                    std::memcpy(g_katamariBaseline6.data(), m_rdram + seg6, g_katamariBaseline6.size());
                    std::memcpy(g_katamariBaselineC.data(), m_rdram + segC, g_katamariBaselineC.size());
                    std::cerr << "[probe:stream-full] armed h6=0x" << std::hex << g_katamariPreviousFull6
                              << " hc=0x" << g_katamariPreviousFullC << std::dec << '\n';
                }
            }
            function(m_rdram, &context, &m_runtime);
            // Temporary Katamari init/task trace.  This is intentionally
            // scheduler-side (no generated headers or game runtime hooks):
            // it records every EE entry in the startup/task address bands,
            // including threads other than the MPEG worker (thread 3).
            // Keeping it bounded makes the diagnostic cheap enough for the
            // short probe runs while still exposing missed indirect entries.
            if ((executingPc >= 0x100000u && executingPc < 0x117000u) ||
                (executingPc >= 0x149000u && executingPc < 0x154000u) ||
                (executingPc >= 0x164000u && executingPc < 0x17c000u))
            {
                static uint32_t initTraceCount = 0u;
                if (initTraceCount < 320u)
                {
                    std::cerr << "[probe:init-entry] tid=" << running->id
                              << " entry=0x" << std::hex << executingPc
                              << " next=0x" << context.pc
                              << " ra=0x" << getRegU32(&context, 31)
                              << " a0=0x" << getRegU32(&context, 4)
                              << " a1=0x" << getRegU32(&context, 5)
                              << " status=" << static_cast<unsigned>(running->status)
                              << " tasks=0x" << Ps2FastRead32(m_rdram, 0x101BDD0u)
                              << std::dec << '\n';
                    ++initTraceCount;
                }
            }
            if (executingPc == 0x1675f8u || (executingPc >= 0x167800u && executingPc < 0x167910u))
            {
                static uint32_t bootStateSnapshotCount = 0u;
                if (bootStateSnapshotCount++ < 24u)
                {
                    const uint32_t task = Ps2FastRead32(m_rdram, 0x101BDD0u) & PS2_RAM_MASK;
                    std::cerr << "[probe:boot-snapshot] pc=0x" << std::hex << context.pc
                              << " task14=0x" << Ps2FastRead32(m_rdram, task + 0x14u)
                              << " busy=0x" << static_cast<unsigned>(m_rdram[0x101DB77u])
                              << " g5611c0=0x" << Ps2FastRead32(m_rdram, 0x5611C0u)
                              << " g5611d0=0x" << Ps2FastRead32(m_rdram, 0x5611D0u)
                              << " g5611e0=0x" << Ps2FastRead32(m_rdram, 0x5611E0u)
                              << " g5611e4=0x" << Ps2FastRead32(m_rdram, 0x5611E4u)
                              << " g5611e8=0x" << Ps2FastRead32(m_rdram, 0x5611E8u)
                              << " g5611ec=0x" << Ps2FastRead32(m_rdram, 0x5611ECu)
                              << " decAC=0x" << static_cast<unsigned>(m_rdram[0x101D42Cu])
                              << " decAD=0x" << static_cast<unsigned>(m_rdram[0x101D42Du])
                              << " decA8=0x" << static_cast<unsigned>(m_rdram[0x101D428u])
                              << " decA9=0x" << static_cast<unsigned>(m_rdram[0x101D429u])
                              << " decAA=0x" << Ps2FastRead16(m_rdram, 0x101D42Au)
                              << " decRemain=0x" << Ps2FastRead32(m_rdram, 0x101D448u)
                              << " decOut=0x" << Ps2FastRead32(m_rdram, 0x101D44Cu)
                              << " decSrc=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " decEnd=0x" << Ps2FastRead32(m_rdram, 0x101D454u)
                              << " stream=0x" << Ps2FastRead32(m_rdram, 0x4A9E4Cu)
                              << std::dec << '\n';
                }
            }
            if (executingPc == 0x167620u || executingPc == 0x167800u || executingPc == 0x167908u)
            {
                static uint32_t taskListProbeCount = 0u;
                if (taskListProbeCount < 48u)
                {
                    const uint32_t head = Ps2FastRead32(m_rdram, 0x101BDD0u) & PS2_RAM_MASK;
                    const uint32_t first = head < PS2_RAM_SIZE ? Ps2FastRead32(m_rdram, head + 0x14u) : 0u;
                    const uint32_t second = first < PS2_RAM_SIZE ? Ps2FastRead32(m_rdram, first + 0x04u) : 0u;
                    const uint32_t third = first < PS2_RAM_SIZE ? Ps2FastRead32(m_rdram, first + 0x08u) : 0u;
                    std::cerr << "[probe:task-list] pc=0x" << std::hex << executingPc
                              << " head=0x" << head << " head14=0x" << Ps2FastRead32(m_rdram, head + 0x14u)
                              << " item=0x" << first << " cb=0x" << second << " next=0x" << third
                              << " g5611c0=0x" << Ps2FastRead32(m_rdram, 0x5611C0u)
                              << " g5611cc=0x" << static_cast<unsigned>(m_rdram[0x5611CCu])
                              << std::dec << '\n';
                    ++taskListProbeCount;
                }
            }
            if (running->id == 3 && executingPc >= 0x188078u && executingPc <= 0x1880B4u)
            {
                static uint32_t decoderNormalProgressProbeCount = 0u;
                if ((decoderNormalProgressProbeCount++ % 1000u) == 0u)
                {
                    std::cerr << "[probe:decoder-progress] pc=0x" << std::hex << executingPc
                              << " next=0x" << context.pc
                              << " remaining=0x" << getRegU32(&context, 19)
                              << " out=0x" << getRegU32(&context, 20)
                              << " src=0x" << getRegU32(&context, 21)
                              << " words=0x" << getRegU32(&context, 17)
                              << " bitpos=0x" << getRegU32(&context, 22)
                              << " bits=0x" << getRegU32(&context, 18)
                              << " state10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " state1e=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << std::dec << '\n';
                }
            }
            if (running->id == 3)
            {
                static uint32_t decoderAfterBoundaryProbeCount = 0u;
                if (decoderAfterBoundaryProbeCount < 160u)
                {
                    std::cerr << "[probe:decoder-after-boundary] pc=0x" << std::hex << executingPc
                              << " next=0x" << context.pc
                              << " state10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " state14=0x" << Ps2FastRead32(m_rdram, 0x101D454u)
                              << " state1c=0x" << Ps2FastRead16(m_rdram, 0x101D45Cu)
                              << " status=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << std::dec << '\n';
                    ++decoderAfterBoundaryProbeCount;
                }
            }
            {
                constexpr uint32_t streamBase = 0x1E4D080u;
                const uint32_t seg6 = streamBase + 6u * 0x20000u;
                const uint32_t segC = streamBase + 0xCu * 0x20000u;
                // Full-segment hashing is diagnostic only; sample sparsely so
                // the probe does not dominate guest execution time.
                static uint32_t fullMonitorSampleCounter = 0u;
                const bool sampleFullMonitor = ((fullMonitorSampleCounter++ & 0x3FFu) == 0u);
                if (g_katamariFullMonitorArmed && sampleFullMonitor)
                {
                    const uint64_t current6 = katamariStreamHash(m_rdram, seg6);
                    const uint64_t currentC = katamariStreamHash(m_rdram, segC);
                    if (current6 != g_katamariPreviousFull6 || currentC != g_katamariPreviousFullC)
                    {
                        std::cerr << "[probe:stream-full-change-after] entry=0x" << std::hex
                                  << executingPc << " h6=0x" << current6 << " hc=0x" << currentC
                                  << " prev6=0x" << g_katamariPreviousFull6 << " prevc=0x" << g_katamariPreviousFullC
                                  << std::dec;
                        uint32_t differences = 0u;
                        for (uint32_t i = 0u; i < 0x20000u && differences < 16u; ++i)
                        {
                            if (m_rdram[seg6 + i] != g_katamariBaseline6[i])
                            {
                                std::cerr << " seg6+0x" << std::hex << i << ":"
                                          << static_cast<unsigned>(g_katamariBaseline6[i]) << ">"
                                          << static_cast<unsigned>(m_rdram[seg6 + i]);
                                ++differences;
                            }
                        }
                        differences = 0u;
                        for (uint32_t i = 0u; i < 0x20000u && differences < 16u; ++i)
                        {
                            if (m_rdram[segC + i] != g_katamariBaselineC[i])
                            {
                                std::cerr << " segc+0x" << std::hex << i << ":"
                                          << static_cast<unsigned>(g_katamariBaselineC[i]) << ">"
                                          << static_cast<unsigned>(m_rdram[segC + i]);
                                ++differences;
                            }
                        }
                        std::cerr << std::dec << '\n';
                        std::memcpy(g_katamariBaseline6.data(), m_rdram + seg6, g_katamariBaseline6.size());
                        std::memcpy(g_katamariBaselineC.data(), m_rdram + segC, g_katamariBaselineC.size());
                        g_katamariPreviousFull6 = current6;
                        g_katamariPreviousFullC = currentC;
                    }
                }
            }
            m_guestExecuting.store(false, std::memory_order_release);
            m_insideInterrupt = false;
            if (executingPc == 0x188010u || executingPc == 0x188028u || executingPc == 0x188034u)
            {
                static uint32_t decoderReturnProbeCount = 0u;
                if (decoderReturnProbeCount < 96u)
                {
                    std::cerr << "[probe:decoder-return] entry=0x" << std::hex << executingPc
                              << " pc=0x" << context.pc
                              << " state+08=0x" << Ps2FastRead32(m_rdram, 0x101D448u)
                              << " +0c=0x" << Ps2FastRead32(m_rdram, 0x101D44Cu)
                              << " +10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " +14=0x" << Ps2FastRead32(m_rdram, 0x101D454u)
                              << " +1e=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << std::dec << '\n';
                    ++decoderReturnProbeCount;
                }
            }
            if (context.pc == 0x54DAC0u)
            {
                std::cerr << "[probe:function54] fn=0x" << std::hex << executingPc
                          << " ra=0x" << getRegU32(&context, 31)
                          << " sp=0x" << getRegU32(&context, 29) << std::dec << std::endl;
            }
        }
        catch (const EeDispatcherTransfer &)
        {
            m_guestExecuting.store(false, std::memory_order_release);
            m_insideInterrupt = false;
            if (running->id == 3 && executingPc >= 0x188078u && executingPc <= 0x1880B4u)
            {
                static uint32_t decoderProgressProbeCount = 0u;
                if ((decoderProgressProbeCount++ % 256u) == 0u)
                {
                    std::cerr << "[probe:decoder-progress] pc=0x" << std::hex << executingPc
                              << " next=0x" << context.pc
                              << " remaining=0x" << getRegU32(&context, 19)
                              << " out=0x" << getRegU32(&context, 20)
                              << " src=0x" << getRegU32(&context, 21)
                              << " words=0x" << getRegU32(&context, 17)
                              << " bitpos=0x" << getRegU32(&context, 22)
                              << " bits=0x" << getRegU32(&context, 18)
                              << " state10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " state1e=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << std::dec << '\n';
                }
            }
            if (executingPc == 0x188010u || executingPc == 0x188028u || executingPc == 0x188034u)
            {
                static uint32_t decoderTransferProbeCount = 0u;
                if (decoderTransferProbeCount < 96u)
                {
                        std::cerr << "[probe:decoder-transfer] entry=0x" << std::hex << executingPc
                              << " pc=0x" << context.pc << " tid=" << running->id
                              << " status=" << static_cast<int>(running->status)
                              << " ra=0x" << getRegU32(&context, 31)
                              << " invocations=" << running->invocations.size()
                              << " state+10=0x" << Ps2FastRead32(m_rdram, 0x101D450u)
                              << " state+1e=0x" << static_cast<unsigned>(m_rdram[0x101D45Eu])
                              << " out=";
                    for (uint32_t i = 0; i < 16u; ++i)
                    {
                        std::cerr << (i == 0u ? "" : " ")
                                  << static_cast<unsigned>(m_rdram[0x1038140u + i]);
                    }
                    std::cerr << " out64=0x" << Ps2FastRead64(m_rdram, 0x1038140u)
                              << std::dec << '\n';
                    ++decoderTransferProbeCount;
                }
            }
        }
        catch (...)
        {
            m_guestExecuting.store(false, std::memory_order_release);
            m_running.store(false, std::memory_order_release);
            publishSnapshot();
            throw;
        }

        processPendingEvents();
        if (m_rescheduleRequested && m_currentThreadId != 0)
        {
            GuestThread *preempted = currentThread();
            assert(preempted != nullptr);
            enqueueReady(*preempted, !m_timeSliceExpired);
            m_currentThreadId = 0;
            m_rescheduleRequested = false;
            m_timeSliceExpired = false;
        }
    }

    m_guestExecuting.store(false, std::memory_order_release);
    m_running.store(false, std::memory_order_release);
    copyMainContextToRuntime();
    publishSnapshot();
}

void EeScheduler::requestStop()
{
    m_stopRequested.store(true, std::memory_order_release);
    m_checkpointPending.store(true, std::memory_order_release);
    m_eventCv.notify_all();
}

void EeScheduler::postEvent(EeEvent event)
{
    if (event.type == EeEventType::Stop)
    {
        requestStop();
        return;
    }

    {
        std::lock_guard lock(m_eventMutex);
        m_events.push_back(event);
        m_checkpointPending.store(true, std::memory_order_release);
    }
    m_eventCv.notify_one();
}

bool EeScheduler::checkpointDue(uint32_t cycles) noexcept
{
    accountCycles(cycles);

    if (m_checkpointPending.load(std::memory_order_acquire) ||
        m_stopRequested.load(std::memory_order_acquire))
    {
        return true;
    }

    const uint64_t nextEventCycle = m_nextDeadlineCycle.load(std::memory_order_acquire);
    if (nextEventCycle != 0u && m_eeCycle >= nextEventCycle)
    {
        m_checkpointPending.store(true, std::memory_order_release);
        return true;
    }

    if (m_eeCycle < m_sliceEndCycle)
    {
        return false;
    }

    const GuestThread *running = currentThread();
    if (running != nullptr && hasReadyAtOrAbovePriority(running->currentPriority))
    {
        m_rescheduleRequested = true;
        m_timeSliceExpired = true;
        return true;
    }

    renewTimeSlice();
    return false;
}

void EeScheduler::accountCycles(uint32_t cycles) noexcept
{
    const uint64_t elapsed = std::max<uint64_t>(1u, cycles);
    m_eeCycle += elapsed;
    m_pendingEeTimerInterrupts |= m_runtime.memory().advanceEeTimers(elapsed);
    if (m_pendingEeTimerInterrupts != 0u)
    {
        m_checkpointPending.store(true, std::memory_order_release);
    }
}

bool EeScheduler::isExecutingGuest() const noexcept
{
    return m_guestExecuting.load(std::memory_order_acquire);
}

void EeScheduler::setupCurrentThread(uint32_t stack, uint32_t stackSize, uint32_t gp)
{
    assertExecutor();
    GuestThread *target = currentThread();
    if (!target)
    {
        return;
    }

    target->stack = stack;
    target->stackSize = stackSize;
    target->gp = gp;
    if (stack != 0u && stack < PS2_RAM_SIZE)
    {
        std::lock_guard<std::mutex> lock(m_runtime.m_asyncCallbackStackMutex);
        m_runtime.m_asyncCallbackStackTop = std::min(m_runtime.m_asyncCallbackStackTop,
                                                      stack & ~(16u - 1u));
    }
    publishSnapshot();
}

int EeScheduler::createThread(const EeThreadCreateParams &params)
{
    assertExecutor();
    // Priority zero is reserved for the kernel's internal top thread
    // (PS2SDK's InitThread creates it with initial_priority = 0).
    if (params.priority < 0 || params.priority >= kPriorityCount)
    {
        return KE_ILLEGAL_PRIORITY;
    }

    const int id = allocateThreadId();
    if (id == 0)
    {
        return KE_ERROR;
    }

    GuestThread thread{};
    thread.id = id;
    thread.entry = params.entry;
    thread.stack = params.stack;
    thread.stackSize = params.stackSize;
    thread.gp = params.gp;
    thread.attr = params.attr;
    thread.option = params.option;
    thread.initialPriority = params.priority;
    thread.currentPriority = params.priority;
    thread.status = EeThreadStatus::Dormant;
    m_threads.emplace(id, std::move(thread));
    publishSnapshot();
    return id;
}

int EeScheduler::deleteThread(int id, uint32_t &ownedStack)
{
    assertExecutor();
    ownedStack = 0;
    if (id <= kMainThreadId)
    {
        return KE_ILLEGAL_THID;
    }
    auto it = m_threads.find(id);
    if (it == m_threads.end())
    {
        return KE_UNKNOWN_THID;
    }
    if (it->second.status != EeThreadStatus::Dormant)
    {
        return KE_NOT_DORMANT;
    }
    if (it->second.ownsStack)
    {
        ownedStack = it->second.stack;
    }
    m_threads.erase(it);
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::startThread(int id, uint32_t arg, const R5900Context &caller, bool interruptSafe)
{
    assertExecutor();
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (target->status != EeThreadStatus::Dormant)
    {
        return KE_NOT_DORMANT;
    }

    target->context = R5900Context{};
    target->context.pc = target->entry;
    target->arg = arg;
    target->suspendCount = 0;
    target->wakeupCount = 0;
    target->wait = {};
    SET_GPR_U32(&target->context, 4, arg);
    SET_GPR_U32(&target->context, 28, target->gp != 0u ? target->gp : getRegU32(&caller, 28));
    const uint32_t stackTop = target->stack != 0u
                                  ? (target->stack + target->stackSize) & ~0xFu
                                  : getRegU32(&caller, 29);
    SET_GPR_U32(&target->context, 29, stackTop);
    SET_GPR_U32(&target->context, 31, 0u);
    enqueueReady(*target);
    requestPreemptionIfHigher(*target, interruptSafe);
    publishSnapshot();
    return KE_OK;
}

[[noreturn]] void EeScheduler::exitCurrent(bool deleteThreadRecord)
{
    assertExecutor();
    GuestThread *exiting = currentThread();
    assert(exiting != nullptr);
    const int id = exiting->id;
    const uint32_t ownedStack = deleteThreadRecord && exiting->ownsStack ? exiting->stack : 0u;
    makeDormant(*exiting);
    m_currentThreadId = 0;
    if (deleteThreadRecord && id != kMainThreadId)
    {
        m_threads.erase(id);
    }
    if (ownedStack != 0u)
    {
        m_runtime.guestFree(ownedStack);
    }
    publishSnapshot();
    throw EeDispatcherTransfer{};
}

int EeScheduler::terminateThread(int id, uint32_t &ownedStack, bool interruptSafe)
{
    assertExecutor();
    ownedStack = 0;
    if (id == 0 || id == m_currentThreadId)
    {
        return KE_ILLEGAL_THID;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (target->status == EeThreadStatus::Dormant)
    {
        return KE_DORMANT;
    }
    if (target->ownsStack)
    {
        ownedStack = target->stack;
        target->ownsStack = false;
    }
    makeDormant(*target);
    (void)interruptSafe;
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::suspendThread(int id, bool interruptSafe)
{
    assertExecutor();
    if (id == 0)
    {
        id = m_currentThreadId;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (target->status == EeThreadStatus::Dormant)
    {
        return KE_DORMANT;
    }

    ++target->suspendCount;
    switch (target->status)
    {
    case EeThreadStatus::Running:
        target->status = EeThreadStatus::Suspended;
        m_currentThreadId = 0;
        m_rescheduleRequested = true;
        break;
    case EeThreadStatus::Ready:
        removeReady(*target);
        target->status = EeThreadStatus::Suspended;
        break;
    case EeThreadStatus::Waiting:
        target->status = EeThreadStatus::WaitingSuspended;
        break;
    case EeThreadStatus::WaitingSuspended:
    case EeThreadStatus::Suspended:
        break;
    case EeThreadStatus::Dormant:
        break;
    }
    if (interruptSafe && m_insideInterrupt)
    {
        m_rescheduleRequested = true;
    }
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::resumeThread(int id, bool interruptSafe)
{
    assertExecutor();
    if (id == 0)
    {
        return KE_ILLEGAL_THID;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (target->suspendCount == 0)
    {
        return KE_NOT_SUSPEND;
    }
    --target->suspendCount;
    if (target->suspendCount != 0)
    {
        return KE_OK;
    }
    if (target->status == EeThreadStatus::WaitingSuspended)
    {
        target->status = EeThreadStatus::Waiting;
    }
    else if (target->status == EeThreadStatus::Suspended)
    {
        enqueueReady(*target);
        requestPreemptionIfHigher(*target, interruptSafe);
    }
    publishSnapshot();
    return KE_OK;
}

void EeScheduler::sleepCurrent()
{
    assertExecutor();
    GuestThread *self = currentThread();
    assert(self != nullptr);
    if (self->wakeupCount != 0u)
    {
        --self->wakeupCount;
        setReturnS32(&self->activeContext(), KE_OK);
        return;
    }
    blockCurrent(EeWaitState{EeWaitReason::Sleep, std::monostate{}});
}

int EeScheduler::wakeupThread(int id, bool interruptSafe)
{
    assertExecutor();
    if (id == 0 || id == m_currentThreadId)
    {
        return KE_ILLEGAL_THID;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (id == 3)
    {
        static uint32_t wakeProbeCount = 0u;
        if (wakeProbeCount < 32u)
        {
            std::cerr << "[probe:decoder-wakeup] id=3 current=" << m_currentThreadId
                      << " status=" << static_cast<int>(target->status)
                      << " wait=" << static_cast<int>(target->wait.reason)
                      << " wakeCount=" << target->wakeupCount
                      << " pc=0x" << std::hex << target->context.pc
                      << std::dec << '\n';
            ++wakeProbeCount;
        }
    }
    if (target->status == EeThreadStatus::Dormant)
    {
        return KE_DORMANT;
    }
    if ((target->status == EeThreadStatus::Waiting || target->status == EeThreadStatus::WaitingSuspended) &&
        target->wait.reason == EeWaitReason::Sleep)
    {
        makeReady(*target, KE_OK, interruptSafe);
    }
    else
    {
        ++target->wakeupCount;
    }
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::cancelWakeup(int id)
{
    assertExecutor();
    if (id == 0)
    {
        id = m_currentThreadId;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    const int old = static_cast<int>(target->wakeupCount);
    target->wakeupCount = 0;
    publishSnapshot();
    return old;
}

int EeScheduler::changePriority(int id, int priority, bool interruptSafe, int &oldPriority)
{
    assertExecutor();
    if (priority < 1 || priority >= kPriorityCount)
    {
        return KE_ILLEGAL_PRIORITY;
    }
    if (id == 0)
    {
        id = m_currentThreadId;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    oldPriority = target->currentPriority;
    if (target->status == EeThreadStatus::Ready)
    {
        removeReady(*target);
        target->currentPriority = priority;
        enqueueReady(*target);
        requestPreemptionIfHigher(*target, interruptSafe);
    }
    else
    {
        target->currentPriority = priority;
        if (target->status == EeThreadStatus::Running)
        {
            for (int p = 0; p < target->currentPriority; ++p)
            {
                if (!m_readyQueues[p].empty())
                {
                    m_rescheduleRequested = true;
                    break;
                }
            }
        }
    }
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::rotateReadyQueue(int priority, bool interruptSafe)
{
    assertExecutor();
    if (priority == 0)
    {
        const GuestThread *self = currentThread();
        priority = self ? self->currentPriority : 0;
    }
    if (priority < 0 || priority >= kPriorityCount)
    {
        return KE_ILLEGAL_PRIORITY;
    }

    GuestThread *self = currentThread();
    if (self && self->currentPriority == priority)
    {
        enqueueReady(*self);
        m_currentThreadId = 0;
        m_rescheduleRequested = true;
    }
    else
    {
        auto &queue = m_readyQueues[priority];
        if (queue.size() > 1u)
        {
            const int head = queue.front();
            queue.pop_front();
            queue.push_back(head);
        }
    }
    (void)interruptSafe;
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::releaseWait(int id, bool interruptSafe)
{
    assertExecutor();
    if (id == 0)
    {
        return KE_ILLEGAL_THID;
    }
    GuestThread *target = thread(id);
    if (!target)
    {
        return KE_UNKNOWN_THID;
    }
    if (target->status != EeThreadStatus::Waiting && target->status != EeThreadStatus::WaitingSuspended)
    {
        return KE_NOT_WAIT;
    }
    removeFromWaitObject(*target);
    makeReady(*target, KE_RELEASE_WAIT, interruptSafe);
    publishSnapshot();
    return KE_OK;
}

void EeScheduler::transferIfRequested(bool interruptSafe)
{
    assertExecutor();
    if (interruptSafe || m_insideInterrupt || !m_rescheduleRequested)
    {
        return;
    }
    if (m_currentThreadId != 0)
    {
        GuestThread *self = currentThread();
        assert(self != nullptr);
        enqueueReady(*self, true);
        m_currentThreadId = 0;
    }
    m_rescheduleRequested = false;
    m_timeSliceExpired = false;
    publishSnapshot();
    throw EeDispatcherTransfer{};
}

int EeScheduler::createSemaphore(int initCount, int maxCount, uint32_t attr, uint32_t option)
{
    assertExecutor();
    if (maxCount <= 0 || initCount < 0 || initCount > maxCount)
    {
        return KE_ERROR;
    }
    const int id = allocatePositiveId(m_nextSemaphoreId, m_semaphores);
    if (id == 0)
    {
        return KE_ERROR;
    }
    EeSemaphore semaphore{};
    semaphore.id = id;
    semaphore.count = initCount;
    semaphore.maxCount = maxCount;
    semaphore.initCount = initCount;
    semaphore.attr = attr;
    semaphore.option = option;
    m_semaphores.emplace(id, std::move(semaphore));
    publishSnapshot();
    return id;
}

int EeScheduler::deleteSemaphore(int id, bool interruptSafe)
{
    assertExecutor();
    auto it = m_semaphores.find(id);
    if (it == m_semaphores.end())
    {
        return KE_UNKNOWN_SEMID;
    }
    std::deque<int> waiters = std::move(it->second.waiters);
    m_semaphores.erase(it);
    for (const int threadId : waiters)
    {
        if (GuestThread *waiter = thread(threadId))
        {
            makeReady(*waiter, KE_WAIT_DELETE, interruptSafe);
        }
    }
    publishSnapshot();
    return id;
}

int EeScheduler::signalSemaphore(int id, bool interruptSafe)
{
    assertExecutor();
    EeSemaphore *object = semaphore(id);
    if (!object)
    {
        return KE_UNKNOWN_SEMID;
    }
    // THROWAWAY-DIAGNOSTIC: who signals the boot semaphore.
    {
        static uint32_t semaSignalProbeCount = 0u;
        if (semaSignalProbeCount < 400u)
        {
            std::cerr << "[probe:sema-signal] id=" << id
                      << " count=" << object->count
                      << " waiters=" << object->waiters.size()
                      << " current=" << m_currentThreadId << '\n';
            ++semaSignalProbeCount;
        }
    }
    if (!object->waiters.empty())
    {
        const int waiterId = object->waiters.front();
        object->waiters.pop_front();
        GuestThread *waiter = thread(waiterId);
        assert(waiter != nullptr);
        makeReady(*waiter, id, interruptSafe);
        publishSnapshot();
        return id;
    }
    if (object->count == object->maxCount)
    {
        return KE_SEMA_OVF;
    }
    ++object->count;
    publishSnapshot();
    return id;
}

int EeScheduler::pollSemaphore(int id)
{
    assertExecutor();
    EeSemaphore *object = semaphore(id);
    if (!object)
    {
        return KE_UNKNOWN_SEMID;
    }
    if (object->count == 0)
    {
        return KE_SEMA_ZERO;
    }
    --object->count;
    publishSnapshot();
    return id;
}

void EeScheduler::waitSemaphore(int id)
{
    assertExecutor();
    EeSemaphore *object = semaphore(id);
    if (!object)
    {
        GuestThread *self = currentThread();
        assert(self != nullptr);
        setReturnS32(&self->activeContext(), KE_UNKNOWN_SEMID);
        return;
    }
    if (object->count != 0)
    {
        --object->count;
        GuestThread *self = currentThread();
        assert(self != nullptr);
        setReturnS32(&self->activeContext(), id);
        publishSnapshot();
        return;
    }
    GuestThread *self = currentThread();
    assert(self != nullptr);
    object->waiters.push_back(self->id);
    blockCurrent(EeWaitState{EeWaitReason::Semaphore, EeSemaphoreWait{id}});
}

int EeScheduler::createEventFlag(uint32_t initialBits, uint32_t attr, uint32_t option)
{
    assertExecutor();
    const int id = allocatePositiveId(m_nextEventFlagId, m_eventFlags);
    if (id == 0)
    {
        return KE_ERROR;
    }
    EeEventFlag flag{};
    flag.id = id;
    flag.attr = attr;
    flag.option = option;
    flag.initBits = initialBits;
    flag.bits = initialBits;
    m_eventFlags.emplace(id, std::move(flag));
    publishSnapshot();
    return id;
}

int EeScheduler::deleteEventFlag(int id, bool interruptSafe)
{
    assertExecutor();
    auto it = m_eventFlags.find(id);
    if (it == m_eventFlags.end())
    {
        return KE_UNKNOWN_EVFID;
    }
    std::deque<int> waiters = std::move(it->second.waiters);
    m_eventFlags.erase(it);
    for (const int threadId : waiters)
    {
        if (GuestThread *waiter = thread(threadId))
        {
            makeReady(*waiter, KE_WAIT_DELETE, interruptSafe);
        }
    }
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::setEventFlag(int id, uint32_t bits, bool interruptSafe)
{
    assertExecutor();
    EeEventFlag *flag = eventFlag(id);
    if (!flag)
    {
        return KE_UNKNOWN_EVFID;
    }
    flag->bits |= bits;
    finishEventWaiters(*flag, interruptSafe);
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::clearEventFlag(int id, uint32_t mask)
{
    assertExecutor();
    EeEventFlag *flag = eventFlag(id);
    if (!flag)
    {
        return KE_UNKNOWN_EVFID;
    }
    flag->bits &= mask;
    publishSnapshot();
    return KE_OK;
}

int EeScheduler::pollEventFlag(int id, uint32_t bits, uint32_t mode, uint32_t &observedBits)
{
    assertExecutor();
    EeEventFlag *flag = eventFlag(id);
    if (!flag)
    {
        return KE_UNKNOWN_EVFID;
    }
    if (!eventCondition(flag->bits, bits, mode))
    {
        return KE_EVF_COND;
    }
    observedBits = flag->bits;
    if ((mode & WEF_CLEAR_ALL) != 0u)
    {
        flag->bits = 0;
    }
    else if ((mode & WEF_CLEAR) != 0u)
    {
        flag->bits &= ~bits;
    }
    publishSnapshot();
    return KE_OK;
}

void EeScheduler::waitEventFlag(int id, uint32_t bits, uint32_t mode, uint32_t resultAddress)
{
    assertExecutor();
    EeEventFlag *flag = eventFlag(id);
    GuestThread *self = currentThread();
    assert(self != nullptr);
    if (!flag)
    {
        setReturnS32(&self->activeContext(), KE_UNKNOWN_EVFID);
        return;
    }
    if (eventCondition(flag->bits, bits, mode))
    {
        const uint32_t observed = flag->bits;
        writeGuestU32(resultAddress, observed);
        if ((mode & WEF_CLEAR_ALL) != 0u)
        {
            flag->bits = 0;
        }
        else if ((mode & WEF_CLEAR) != 0u)
        {
            flag->bits &= ~bits;
        }
        setReturnS32(&self->activeContext(), KE_OK);
        publishSnapshot();
        return;
    }
    flag->waiters.push_back(self->id);
    blockCurrent(EeWaitState{EeWaitReason::EventFlag,
                             EeEventFlagWait{id, bits, mode, resultAddress}});
}

int EeScheduler::setAlarm(uint16_t ticks,
                          uint32_t handler,
                          uint32_t argument,
                          uint32_t gp,
                          uint32_t sp)
{
    assertExecutor();
    if (handler == 0u || !m_runtime.hasFunction(handler))
    {
        return KE_ERROR;
    }
    const int id = allocatePositiveId(m_nextAlarmId, m_alarms);
    if (id == 0)
    {
        return KE_ERROR;
    }
    m_alarms.emplace(id, EeAlarm{id, ticks, handler, argument, gp, sp});
    const uint64_t tickCount = ticks == 0u ? 1u : static_cast<uint64_t>(ticks);
    scheduleEvent(m_eeCycle + tickCount * kAlarmTickCycles,
                  std::chrono::steady_clock::now() + std::chrono::microseconds(tickCount * kAlarmTickMicroseconds),
                  EeEvent{EeEventType::Alarm, static_cast<uint32_t>(id), 0});
    return id;
}

int EeScheduler::cancelAlarm(int id)
{
    assertExecutor();
    if (m_alarms.erase(id) == 0u)
    {
        return KE_ERROR;
    }
    {
        std::lock_guard lock(m_eventMutex);
        std::erase_if(m_deadlines, [id](const ScheduledEvent &scheduled)
                      { return scheduled.event.type == EeEventType::Alarm &&
                               scheduled.event.id == static_cast<uint32_t>(id); });
        updateNextDeadline();
    }
    return KE_OK;
}

void EeScheduler::queueInvocation(GuestInvocation invocation)
{
    assertExecutor();
    // Interrupt sources are level-triggered from the emulated hardware.  If
    // an identical handler is already waiting for delivery, coalesce the
    // edge instead of building an unbounded backlog that can starve ready EE
    // threads (and, for callback-owned stacks, force needless re-entry).
    if (invocation.kind == GuestInvocationKind::Interrupt)
    {
        const bool alreadyPending = std::any_of(
            m_pendingInvocations.begin(), m_pendingInvocations.end(),
            [&invocation](const GuestInvocation &pending)
            {
                return pending.kind == GuestInvocationKind::Interrupt &&
                       pending.context.pc == invocation.context.pc &&
                       pending.tag == invocation.tag;
            });
        if (alreadyPending)
        {
            return;
        }
    }
    invocation.sequence = ++m_invocationSequence;
    m_pendingInvocations.push_back(std::move(invocation));
    m_checkpointPending.store(true, std::memory_order_release);
}

[[noreturn]] void EeScheduler::invokeCurrent(GuestInvocation invocation)
{
    assertExecutor();
    GuestThread *owner = currentThread();
    assert(owner != nullptr);
    {
        static uint32_t invokeProbeCount = 0u;
        if (invokeProbeCount < 128u)
        {
            std::cerr << "[probe:invoke-current] owner=" << owner->id
                      << " depth=" << owner->invocations.size()
                      << " kind=" << static_cast<unsigned>(invocation.kind)
                      << " pc=0x" << std::hex << invocation.context.pc
                      << " tag=0x" << invocation.tag << std::dec << '\n';
            ++invokeProbeCount;
        }
    }
    if (invocation.kind == GuestInvocationKind::Interrupt ||
        getRegU32(&invocation.context, 29) == 0u)
    {
        SET_GPR_U32(&invocation.context, 29, invocationStackTop());
    }
    invocation.sequence = ++m_invocationSequence;
    owner->invocations.push_back(std::move(invocation));
    publishSnapshot();
    throw EeDispatcherTransfer{};
}

[[noreturn]] void EeScheduler::invokeCurrentSequence(std::vector<GuestInvocation> invocations)
{
    assertExecutor();
    GuestThread *owner = currentThread();
    assert(owner != nullptr);
    {
        static uint32_t invokeSequenceProbeCount = 0u;
        if (invokeSequenceProbeCount < 96u)
        {
            std::cerr << "[probe:invoke-sequence] owner=" << owner->id
                      << " depth=" << owner->invocations.size()
                      << " count=" << invocations.size() << std::dec << '\n';
            ++invokeSequenceProbeCount;
        }
    }
    assert(!invocations.empty());
    for (auto it = invocations.rbegin(); it != invocations.rend(); ++it)
    {
        if (it->kind == GuestInvocationKind::Interrupt ||
            getRegU32(&it->context, 29) == 0u)
        {
            SET_GPR_U32(&it->context, 29, invocationStackTop());
        }
        it->sequence = ++m_invocationSequence;
        owner->invocations.push_back(std::move(*it));
    }
    publishSnapshot();
    throw EeDispatcherTransfer{};
}

bool EeScheduler::hasInvocation(GuestInvocationKind kind, uint64_t tag) const
{
    const GuestThread *owner = currentThread();
    if (!owner)
    {
        return false;
    }
    return std::any_of(owner->invocations.begin(), owner->invocations.end(),
                       [kind, tag](const GuestInvocation &invocation)
                       {
                           return invocation.kind == kind && invocation.tag == tag;
                       });
}

uint32_t EeScheduler::invocationStackTop()
{
    assertExecutor();
    const GuestThread *owner = currentThread();
    if (!owner)
    {
        throw std::logic_error("EE invocation stack requested without a current guest context");
    }
    const size_t depth = owner ? owner->invocations.size() : 0u;
    {
        static uint32_t invocationStackProbeCount = 0u;
        if (invocationStackProbeCount < 96u)
        {
            std::cerr << "[probe:invocation-stack] owner=" << owner->id
                      << " depth=" << depth
                      << " activePc=0x" << std::hex << owner->activeContext().pc
                      << std::dec << '\n';
            ++invocationStackProbeCount;
        }
    }
    const uint64_t key = (static_cast<uint64_t>(static_cast<uint32_t>(owner->id)) << 32u) |
                         static_cast<uint32_t>(depth);
    const auto existing = m_invocationStackTops.find(key);
    if (existing != m_invocationStackTops.end())
    {
        return existing->second;
    }
    constexpr uint32_t kInvocationStackSize = 0x4000u;
    const uint32_t top = m_runtime.reserveAsyncCallbackStack(kInvocationStackSize, 16u);
    if (top == 0u)
    {
        throw std::runtime_error("EE invocation stack space exhausted");
    }
    m_invocationStackTops.emplace(key, top);
    return top;
}

int EeScheduler::addIrqHandler(bool dmac,
                               uint32_t cause,
                               uint32_t handler,
                               bool append,
                               uint32_t argument,
                               uint32_t gp,
                               uint32_t sp)
{
    assertExecutor();
    auto &handlers = dmac ? m_dmacHandlers : m_intcHandlers;
    int &nextId = dmac ? m_nextDmacHandlerId : m_nextIntcHandlerId;
    const int id = allocatePositiveId(nextId, handlers);
    if (id == 0)
    {
        return KE_ERROR;
    }
    int &head = dmac ? m_dmacHeadOrder : m_intcHeadOrder;
    int &tail = dmac ? m_dmacTailOrder : m_intcTailOrder;
    handlers.emplace(id,
                     EeIrqHandler{id,
                                  cause,
                                  handler,
                                  argument,
                                  gp,
                                  sp,
                                  true,
                                  append ? ++tail : --head});
    return id;
}

int EeScheduler::removeIrqHandler(bool dmac, uint32_t cause, int id)
{
    assertExecutor();
    auto &handlers = dmac ? m_dmacHandlers : m_intcHandlers;
    auto it = handlers.find(id);
    if (it != handlers.end() && it->second.cause == cause)
    {
        handlers.erase(it);
    }
    return KE_OK;
}

int EeScheduler::setIrqHandlerEnabled(bool dmac, int id, bool enabled)
{
    assertExecutor();
    auto &handlers = dmac ? m_dmacHandlers : m_intcHandlers;
    auto it = handlers.find(id);
    if (it != handlers.end())
    {
        it->second.enabled = enabled;
    }
    return KE_OK;
}

int EeScheduler::setIrqCauseEnabled(bool dmac, uint32_t cause, bool enabled)
{
    assertExecutor();
    if (cause < 32u)
    {
        uint32_t &mask = dmac ? m_enabledDmacMask : m_enabledIntcMask;
        if (enabled)
        {
            mask |= 1u << cause;
        }
        else
        {
            mask &= ~(1u << cause);
        }
    }
    return KE_OK;
}

void EeScheduler::dispatchIrq(bool dmac, uint32_t cause)
{
    if (m_executorThread == std::thread::id{})
    {
        m_executorThread = std::this_thread::get_id();
    }
    assertExecutor();
    const uint32_t mask = dmac ? m_enabledDmacMask : m_enabledIntcMask;
    if (cause < 32u && (mask & (1u << cause)) == 0u)
    {
        return;
    }
    const auto &handlers = dmac ? m_dmacHandlers : m_intcHandlers;
    std::vector<EeIrqHandler> matching;
    for (const auto &[id, handler] : handlers)
    {
        (void)id;
        if (handler.enabled && handler.cause == cause && handler.handler != 0u &&
            m_runtime.hasFunction(handler.handler))
        {
            matching.push_back(handler);
        }
    }
    std::sort(matching.begin(), matching.end(), [](const EeIrqHandler &left, const EeIrqHandler &right)
              { return left.order < right.order; });
    if (dmac && cause == 5u)
    {
        static uint32_t sif0DispatchProbeCount = 0u;
        if (sif0DispatchProbeCount++ < 256u)
        {
            std::cerr << "[probe:sif0-dispatch] handlers=" << matching.size()
                      << " enabled=1 running=" << m_running.load(std::memory_order_relaxed)
                      << " current=" << (currentThread() ? currentThread()->id : -1)
                      << " inside=" << m_insideInterrupt
                      << " depth=" << (currentThread() ? currentThread()->invocations.size() : 0u)
                      << " first=0x" << std::hex
                      << (matching.empty() ? 0u : matching.front().handler)
                      << std::dec
                      << '\n';
        }
    }
    for (const EeIrqHandler &handler : matching)
    {
        GuestInvocation invocation{};
        invocation.kind = GuestInvocationKind::Interrupt;
        invocation.context.pc = handler.handler;
        SET_GPR_U32(&invocation.context, 4, cause);
        SET_GPR_U32(&invocation.context, 5, handler.argument);
        SET_GPR_U32(&invocation.context, 28, handler.gp);
        SET_GPR_U32(&invocation.context, 29, 0u);
        SET_GPR_U32(&invocation.context, 31, 0u);
        queueInvocation(std::move(invocation));
    }
}

void EeScheduler::dispatchIrqNow(bool dmac, uint32_t cause)
{
    if (m_executorThread == std::thread::id{})
    {
        m_executorThread = std::this_thread::get_id();
    }
    assertExecutor();
    const uint32_t mask = dmac ? m_enabledDmacMask : m_enabledIntcMask;
    if (cause < 32u && (mask & (1u << cause)) == 0u)
    {
        return;
    }
    const auto &handlers = dmac ? m_dmacHandlers : m_intcHandlers;
    std::vector<EeIrqHandler> matching;
    for (const auto &[id, handler] : handlers)
    {
        (void)id;
        if (handler.enabled && handler.cause == cause && handler.handler != 0u &&
            m_runtime.hasFunction(handler.handler))
        {
            matching.push_back(handler);
        }
    }
    std::sort(matching.begin(), matching.end(), [](const EeIrqHandler &left, const EeIrqHandler &right)
              { return left.order < right.order; });
    std::vector<GuestInvocation> invocations;
    invocations.reserve(matching.size());
    for (const EeIrqHandler &handler : matching)
    {
        GuestInvocation invocation{};
        invocation.kind = GuestInvocationKind::Interrupt;
        invocation.context.pc = handler.handler;
        SET_GPR_U32(&invocation.context, 4, cause);
        SET_GPR_U32(&invocation.context, 5, handler.argument);
        SET_GPR_U32(&invocation.context, 28, handler.gp);
        SET_GPR_U32(&invocation.context, 29, 0u);
        SET_GPR_U32(&invocation.context, 31, 0u);
        invocations.push_back(std::move(invocation));
    }
    if (invocations.empty())
    {
        return;
    }

    // A completion raised while an invocation is active must remain pending
    // until that frame returns.  Checking the stack itself is more robust than
    // relying only on the cached interrupt flag: callbacks can raise another
    // IRQ from inside generated code before the next scheduler boundary.
    if (m_running.load(std::memory_order_acquire) && currentThread() &&
        !m_insideInterrupt && currentThread()->invocations.empty())
    {
        invokeCurrentSequence(std::move(invocations));
    }
    else
    {
        for (GuestInvocation &invocation : invocations)
        {
            queueInvocation(std::move(invocation));
        }
    }
}

void EeScheduler::setVSyncFlag(uint32_t flagAddress, uint32_t tickAddress)
{
    assertExecutor();
    m_vsyncFlagAddress = flagAddress;
    m_vsyncTickAddress = tickAddress;
    writeGuestU32(flagAddress, 0u);
    if (tickAddress != 0u)
    {
        const uint32_t physical = tickAddress & 0x1FFFFFFFu;
        if (m_rdram && physical <= PS2_RAM_SIZE - sizeof(uint64_t))
        {
            const uint64_t zero = 0u;
            std::memcpy(m_rdram + physical, &zero, sizeof(zero));
        }
    }
}

uint64_t EeScheduler::currentVSyncTick() const noexcept
{
    return m_vsyncTick;
}

uint32_t EeScheduler::setGsVSyncCallback(uint32_t callback, uint32_t gp, uint32_t sp)
{
    assertExecutor();
    (void)sp;
    const uint32_t previous = m_gsVSyncCallback;
    m_gsVSyncCallback = callback;
    m_gsVSyncCallbackGp = gp;
    m_gsVSyncCallbackSp = 0u;
    return previous;
}

[[noreturn]] void EeScheduler::waitVSync(uint64_t afterTick, int fixedResult, std::function<void(R5900Context &)> completion)
{
    blockCurrent(EeWaitState{
        EeWaitReason::VSync,
        EeVSyncWait{afterTick, fixedResult},
        std::move(completion)});
}

void EeScheduler::completeVSync(uint64_t tick)
{
    assertExecutor();
    std::vector<int> completed;
    for (const auto &[id, candidate] : m_threads)
    {
        if ((candidate.status == EeThreadStatus::Waiting || candidate.status == EeThreadStatus::WaitingSuspended) &&
            candidate.wait.reason == EeWaitReason::VSync &&
            std::get<EeVSyncWait>(candidate.wait.payload).afterTick < tick)
        {
            completed.push_back(id);
        }
    }
    std::sort(completed.begin(), completed.end());
    for (const int id : completed)
    {
        GuestThread *waiter = thread(id);
        assert(waiter != nullptr);
        const EeVSyncWait wait = std::get<EeVSyncWait>(waiter->wait.payload);
        const int result = wait.fixedResult >= 0
                               ? wait.fixedResult
                               : static_cast<int>((tick - 1u) & 1u);
        makeReady(*waiter, result, false);
    }
    publishSnapshot();
}

void EeScheduler::completeExternalWait(uint32_t type, uint64_t token, int result)
{
    assertExecutor();
    std::vector<int> completed;
    for (const auto &[id, candidate] : m_threads)
    {
        if ((candidate.status != EeThreadStatus::Waiting && candidate.status != EeThreadStatus::WaitingSuspended) ||
            (candidate.wait.reason != EeWaitReason::External &&
             candidate.wait.reason != EeWaitReason::Mpeg))
        {
            continue;
        }
        const auto &external = std::get<EeExternalWait>(candidate.wait.payload);
        if (external.type == type && external.token == token)
        {
            completed.push_back(id);
        }
    }
    std::sort(completed.begin(), completed.end());
    for (const int id : completed)
    {
        GuestThread *waiter = thread(id);
        assert(waiter != nullptr);
        makeReady(*waiter, result, false);
    }
    publishSnapshot();
}

[[noreturn]] void EeScheduler::waitExternal(EeWaitReason reason,
                                            uint32_t type,
                                            uint64_t token,
                                            std::function<void(R5900Context &)> completion)
{
    EeWaitState wait{reason, EeExternalWait{type, token}, std::move(completion)};
    blockCurrent(std::move(wait));
}

GuestThread *EeScheduler::thread(int id)
{
    auto it = m_threads.find(id);
    return it == m_threads.end() ? nullptr : &it->second;
}

const GuestThread *EeScheduler::thread(int id) const
{
    auto it = m_threads.find(id);
    return it == m_threads.end() ? nullptr : &it->second;
}

EeSemaphore *EeScheduler::semaphore(int id)
{
    auto it = m_semaphores.find(id);
    return it == m_semaphores.end() ? nullptr : &it->second;
}

const EeSemaphore *EeScheduler::semaphore(int id) const
{
    auto it = m_semaphores.find(id);
    return it == m_semaphores.end() ? nullptr : &it->second;
}

EeEventFlag *EeScheduler::eventFlag(int id)
{
    auto it = m_eventFlags.find(id);
    return it == m_eventFlags.end() ? nullptr : &it->second;
}

const EeEventFlag *EeScheduler::eventFlag(int id) const
{
    auto it = m_eventFlags.find(id);
    return it == m_eventFlags.end() ? nullptr : &it->second;
}

GuestThread *EeScheduler::currentThread()
{
    return thread(m_currentThreadId);
}

const GuestThread *EeScheduler::currentThread() const
{
    return thread(m_currentThreadId);
}

int EeScheduler::currentThreadId() const noexcept
{
    return m_currentThreadId;
}

R5900Context *EeScheduler::currentContext()
{
    GuestThread *self = currentThread();
    return self ? &self->activeContext() : nullptr;
}

uint8_t *EeScheduler::rdram() const noexcept
{
    return m_rdram;
}

void EeScheduler::bindMainContextForSyscall(R5900Context &ctx, uint8_t *rdram)
{
    if (m_executorThread == std::thread::id{} || m_threads.empty())
    {
        reset(rdram, ctx);
        GuestThread *main = selectReady();
        assert(main != nullptr);
        makeRunning(*main);
        return;
    }
    assertExecutor();
    m_rdram = rdram;
    if (m_currentThreadId == 0)
    {
        GuestThread *main = thread(kMainThreadId);
        assert(main != nullptr);
        assert(main->status == EeThreadStatus::Ready);
        removeReady(*main);
        makeRunning(*main);
    }
}

EeKernelSnapshot EeScheduler::snapshot() const
{
    std::lock_guard lock(m_snapshotMutex);
    return m_snapshot;
}

void EeScheduler::publishSnapshot()
{
    EeKernelSnapshot next{};
    next.sequence = ++m_snapshotSequence;
    next.eeCycle = m_eeCycle;
    next.sliceEndCycle = m_sliceEndCycle;
    next.nextEventCycle = m_nextDeadlineCycle.load(std::memory_order_acquire);
    next.runningThreadId = m_currentThreadId;
    next.threads.reserve(m_threads.size());
    for (const auto &[id, item] : m_threads)
    {
        if (id < 0)
        {
            continue;
        }
        EeThreadSnapshot snapshot{};
        snapshot.id = id;
        snapshot.pc = item.activeContext().pc;
        snapshot.ra = getRegU32(&item.activeContext(), 31);
        snapshot.sp = getRegU32(&item.activeContext(), 29);
        snapshot.entry = item.entry;
        snapshot.stack = item.stack;
        snapshot.stackSize = item.stackSize;
        snapshot.gp = item.gp;
        snapshot.initialPriority = item.initialPriority;
        snapshot.currentPriority = item.currentPriority;
        snapshot.status = item.status;
        snapshot.waitReason = item.wait.reason;
        snapshot.waitId = waitObjectId(item.wait);
        snapshot.suspendCount = item.suspendCount;
        snapshot.wakeupCount = item.wakeupCount;
        next.threads.push_back(snapshot);
    }
    std::sort(next.threads.begin(), next.threads.end(), [](const auto &left, const auto &right)
              { return left.id < right.id; });
    next.semaphores.reserve(m_semaphores.size());
    for (const auto &[id, item] : m_semaphores)
    {
        next.semaphores.push_back(EeSemaphoreSnapshot{id,
                                                      item.count,
                                                      item.maxCount,
                                                      static_cast<uint32_t>(item.waiters.size())});
    }
    std::sort(next.semaphores.begin(), next.semaphores.end(), [](const auto &left, const auto &right)
              { return left.id < right.id; });
    next.eventFlags.reserve(m_eventFlags.size());
    for (const auto &[id, item] : m_eventFlags)
    {
        next.eventFlags.push_back(EeEventFlagSnapshot{id,
                                                      item.bits,
                                                      item.initBits,
                                                      item.attr,
                                                      static_cast<uint32_t>(item.waiters.size())});
    }
    std::sort(next.eventFlags.begin(), next.eventFlags.end(), [](const auto &left, const auto &right)
              { return left.id < right.id; });
    {
        std::lock_guard lock(m_snapshotMutex);
        m_snapshot = std::move(next);
    }
}

void EeScheduler::assertExecutor() const
{
    assert(m_executorThread == std::this_thread::get_id());
}

int EeScheduler::allocateThreadId()
{
    for (int attempts = 0; attempts <= kLastThreadId - kFirstThreadId; ++attempts)
    {
        const int candidate = m_nextThreadId;
        m_nextThreadId = candidate == kLastThreadId ? kFirstThreadId : candidate + 1;
        if (!m_threads.contains(candidate))
        {
            return candidate;
        }
    }
    return 0;
}

GuestThread &EeScheduler::acquireInvocationThread()
{
    for (auto &[id, candidate] : m_threads)
    {
        if (id < 0 && candidate.status == EeThreadStatus::Dormant && candidate.invocations.empty())
        {
            return candidate;
        }
    }

    GuestThread dispatcher{};
    dispatcher.id = m_nextInvocationThreadId--;
    dispatcher.initialPriority = 0;
    dispatcher.currentPriority = 0;
    dispatcher.status = EeThreadStatus::Dormant;
    return m_threads.emplace(dispatcher.id, std::move(dispatcher)).first->second;
}

void EeScheduler::enqueueReady(GuestThread &item, bool front)
{
    assert(item.currentPriority >= 0 && item.currentPriority < kPriorityCount);
    item.status = EeThreadStatus::Ready;
    auto &queue = m_readyQueues[item.currentPriority];
    if (front)
    {
        queue.push_front(item.id);
    }
    else
    {
        queue.push_back(item.id);
    }
}

void EeScheduler::removeReady(GuestThread &item)
{
    if (item.status != EeThreadStatus::Ready)
    {
        return;
    }
    auto &queue = m_readyQueues[item.currentPriority];
    auto it = std::find(queue.begin(), queue.end(), item.id);
    assert(it != queue.end());
    queue.erase(it);
}

GuestThread *EeScheduler::selectReady()
{
    for (auto &queue : m_readyQueues)
    {
        if (queue.empty())
        {
            continue;
        }
        const int id = queue.front();
        queue.pop_front();
        GuestThread *selected = thread(id);
        assert(selected != nullptr);
        assert(selected->status == EeThreadStatus::Ready);
        return selected;
    }
    return nullptr;
}

void EeScheduler::makeRunning(GuestThread &item)
{
    assert(m_currentThreadId == 0);
    assert(item.status == EeThreadStatus::Ready);
    item.status = EeThreadStatus::Running;
    m_currentThreadId = item.id;
    renewTimeSlice();
    // TEMP-EXPERIMENT: thread run census. Revert.
    {
        static std::array<uint64_t, 16> runCounts{};
        static uint64_t totalRuns = 0;
        if (item.id >= 0 && item.id < 16)
            runCounts[static_cast<size_t>(item.id)]++;
        ++totalRuns;
        if (totalRuns <= 40u)
        {
            std::cerr << "[thsched] run#" << totalRuns << " t=" << item.id
                      << " pc=0x" << std::hex << item.activeContext().pc << std::dec << std::endl;
        }
        if (totalRuns % 20000u == 0u)
        {
            std::cerr << "[thsched] total=" << totalRuns << " runs:";
            for (size_t i = 0; i < 8; ++i)
                std::cerr << " t" << i << "=" << runCounts[i];
            std::cerr << " pc=0x" << std::hex << item.activeContext().pc << std::dec << std::endl;
        }
    }
}

void EeScheduler::makeDormant(GuestThread &item)
{
    removeReady(item);
    removeFromWaitObject(item);
    if (item.id == 3)
    {
        static uint32_t dormantProbeCount = 0u;
        if (dormantProbeCount < 24u)
        {
            std::cerr << "[probe:decoder-dormant] pc=0x" << std::hex
                      << item.context.pc << " status=" << static_cast<int>(item.status)
                      << " ra=0x" << getRegU32(&item.context, 31)
                      << " invocations=" << item.invocations.size()
                      << std::dec << '\n';
            ++dormantProbeCount;
        }
    }
    item.status = EeThreadStatus::Dormant;
    item.wait = {};
    item.resumeCompletion = {};
    item.suspendCount = 0;
    item.wakeupCount = 0;
    item.invocations.clear();
}

void EeScheduler::removeFromWaitObject(GuestThread &item)
{
    const int id = item.id;
    if (item.wait.reason == EeWaitReason::Semaphore)
    {
        const int objectId = std::get<EeSemaphoreWait>(item.wait.payload).id;
        if (EeSemaphore *object = semaphore(objectId))
        {
            auto it = std::find(object->waiters.begin(), object->waiters.end(), id);
            if (it != object->waiters.end())
            {
                object->waiters.erase(it);
            }
        }
    }
    else if (item.wait.reason == EeWaitReason::EventFlag)
    {
        const int objectId = std::get<EeEventFlagWait>(item.wait.payload).id;
        if (EeEventFlag *object = eventFlag(objectId))
        {
            auto it = std::find(object->waiters.begin(), object->waiters.end(), id);
            if (it != object->waiters.end())
            {
                object->waiters.erase(it);
            }
        }
    }
    item.wait = {};
}

void EeScheduler::blockCurrent(EeWaitState wait)
{
    GuestThread *self = currentThread();
    assert(self != nullptr);
    // TEMP-EXPERIMENT: trace thread1 waits. Revert.
    if (self->id == 1)
    {
        static uint32_t t1block = 0u;
        if (t1block < 24u)
        {
            ++t1block;
            std::cerr << "[probe:t1-block] n=" << t1block << " pc=0x" << std::hex
                      << self->context.pc << " ra=0x"
                      << (uint32_t)_mm_extract_epi32(self->context.r[31], 0)
                      << " reason=" << std::dec << static_cast<int>(wait.reason)
                      << std::endl;
        }
    }
    if (self->id == 3 || self->id == 4)
    {
        static uint32_t blockProbeCount = 0u;
        if (blockProbeCount < 48u)
        {
            std::cerr << "[probe:decoder-block] id=" << self->id << " pc=0x" << std::hex
                      << self->context.pc << " reason=" << static_cast<int>(wait.reason)
                      << " s1=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[17], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[17], 0))
                      << " s3=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[19], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[19], 0))
                      << " s4=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[20], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[20], 0))
                      << " s5=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[21], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[21], 0))
                      << " s6=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[22], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[22], 0))
                      << " s7=0x" << ((uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[23], 1) << 32 | (uint64_t)(uint32_t)_mm_extract_epi32(self->context.r[23], 0))
                      << std::dec << '\n';
            ++blockProbeCount;
        }
    }
    else
    {
        static uint32_t blockOtherProbeCount = 0u;
        if (blockOtherProbeCount < 400u)
        {
            std::cerr << "[probe:thread-block] id=" << self->id << " pc=0x" << std::hex
                      << self->context.pc << " reason=" << static_cast<int>(wait.reason)
                      << " ra=0x" << getRegU32(&self->context, 31)
                      << " a0=0x" << getRegU32(&self->context, 4)
                      << std::dec << '\n';
            ++blockOtherProbeCount;
        }
    }
    self->wait = std::move(wait);
    self->status = self->suspendCount == 0 ? EeThreadStatus::Waiting : EeThreadStatus::WaitingSuspended;
    m_currentThreadId = 0;
    publishSnapshot();
    throw EeDispatcherTransfer{};
}

void EeScheduler::makeReady(GuestThread &item, int result, bool interruptSafe)
{
    auto completion = std::move(item.wait.completion);
    item.wait = {};
    setReturnS32(&item.activeContext(), result);
    item.resumeCompletion = std::move(completion);
    if (item.suspendCount != 0)
    {
        item.status = EeThreadStatus::Suspended;
        return;
    }
    enqueueReady(item);
    requestPreemptionIfHigher(item, interruptSafe);
}

void EeScheduler::requestPreemptionIfHigher(const GuestThread &readyThread, bool interruptSafe)
{
    const GuestThread *running = currentThread();
    if (!running || readyThread.currentPriority >= running->currentPriority)
    {
        return;
    }
    m_rescheduleRequested = true;
    if (interruptSafe || m_insideInterrupt)
    {
        m_checkpointPending.store(true, std::memory_order_release);
    }
}

void EeScheduler::applyPendingPreemption()
{
    if (!m_rescheduleRequested)
    {
        return;
    }
    if (m_currentThreadId == 0)
    {
        m_rescheduleRequested = false;
        m_timeSliceExpired = false;
        return;
    }
    GuestThread *self = currentThread();
    assert(self != nullptr);
    enqueueReady(*self, !m_timeSliceExpired);
    m_currentThreadId = 0;
    m_rescheduleRequested = false;
    m_timeSliceExpired = false;
}

void EeScheduler::processPendingEvents()
{
    assertExecutor();
    processDueDeadlines();
    const uint32_t timerInterrupts = m_pendingEeTimerInterrupts;
    m_pendingEeTimerInterrupts = 0u;
    for (uint32_t timer = 0u; timer < 4u; ++timer)
    {
        if ((timerInterrupts & (1u << timer)) != 0u)
        {
            dispatchIrq(false, 9u + timer);
        }
    }
    std::deque<EeEvent> pending;
    {
        std::lock_guard lock(m_eventMutex);
        pending.swap(m_events);
    }
    for (const EeEvent &event : pending)
    {
        processEvent(event);
    }

    {
        std::lock_guard lock(m_eventMutex);
        const uint64_t nextEventCycle = m_nextDeadlineCycle.load(std::memory_order_acquire);
        const bool cycleEventDue = nextEventCycle != 0u && m_eeCycle >= nextEventCycle;
        const bool pendingWork = !m_events.empty() || cycleEventDue || m_stopRequested.load(std::memory_order_acquire);
        m_checkpointPending.store(pendingWork, std::memory_order_release);
    }
    applyPendingPreemption();
}

void EeScheduler::processDueDeadlines()
{
    for (;;)
    {
        std::vector<ScheduledEvent> due;
        std::chrono::steady_clock::time_point pacingDeadline{};
        {
            std::unique_lock lock(m_eventMutex);
            const auto now = std::chrono::steady_clock::now();
            for (const ScheduledEvent &item : m_deadlines)
            {
                if (item.deadlineCycle <= m_eeCycle &&
                    (pacingDeadline == std::chrono::steady_clock::time_point{} ||
                     item.hostDeadline < pacingDeadline))
                {
                    pacingDeadline = item.hostDeadline;
                }
            }

            if (pacingDeadline == std::chrono::steady_clock::time_point{})
            {
                updateNextDeadline();
                return;
            }

            if (now < pacingDeadline)
            {
                m_eventCv.wait_until(lock, pacingDeadline, [this]()
                                     { return !m_events.empty() ||
                                              m_stopRequested.load(std::memory_order_acquire); });
                if (!m_events.empty() || m_stopRequested.load(std::memory_order_acquire))
                {
                    updateNextDeadline();
                    return;
                }
            }

            const auto pacedNow = std::chrono::steady_clock::now();
            auto firstFuture = std::partition(m_deadlines.begin(), m_deadlines.end(),
                                              [this, pacedNow](const ScheduledEvent &item)
                                              { return item.deadlineCycle <= m_eeCycle &&
                                                       item.hostDeadline <= pacedNow; });
            due.insert(due.end(),
                       std::make_move_iterator(m_deadlines.begin()),
                       std::make_move_iterator(firstFuture));
            m_deadlines.erase(m_deadlines.begin(), firstFuture);
            updateNextDeadline();
        }

        std::sort(due.begin(), due.end(), [](const ScheduledEvent &left, const ScheduledEvent &right)
                  {
                      if (left.deadlineCycle != right.deadlineCycle)
                      {
                          return left.deadlineCycle < right.deadlineCycle;
                      }
                      if (left.event.type != right.event.type)
                      {
                          return left.event.type < right.event.type;
                      }
                      if (left.event.id != right.event.id)
                      {
                          return left.event.id < right.event.id;
                      }
                      return left.sequence < right.sequence; });

        if (due.empty())
        {
            return;
        }

        for (ScheduledEvent &scheduled : due)
        {
            if (scheduled.event.type == EeEventType::VBlankStart)
            {
                scheduleEvent(scheduled.deadlineCycle + kVBlankDurationCycles,
                              scheduled.hostDeadline + kVBlankDuration,
                              EeEvent{EeEventType::VBlankEnd, 0, m_vsyncTick + 1u});
                scheduleEvent(scheduled.deadlineCycle + kVBlankPeriodCycles,
                              scheduled.hostDeadline + kVBlankPeriod,
                              EeEvent{EeEventType::VBlankStart, 0, 0});
            }
            processEvent(scheduled.event);
        }
    }
}

void EeScheduler::processEvent(const EeEvent &event)
{
    switch (event.type)
    {
    case EeEventType::Stop:
        requestStop();
        break;
    case EeEventType::VBlankStart:
        ++m_vsyncTick;
        // TEMP-EXPERIMENT: prove scheduler VBlank fires. Revert.
        {
            static int vblankSched = 0;
            if (vblankSched < 3)
            {
                ++vblankSched;
                std::cerr << "[vblank-sched] n=" << vblankSched << " tick=" << m_vsyncTick
                          << std::endl;
            }
        }
        m_runtime.memory().gs().vsyncTick.store(m_vsyncTick, std::memory_order_release);
        if ((m_vsyncTick & 1u) != 0u)
        {
            m_runtime.memory().gs().csr.fetch_or(0x2000ull, std::memory_order_acq_rel);
        }
        else
        {
            m_runtime.memory().gs().csr.fetch_and(~0x2000ull, std::memory_order_acq_rel);
        }
        writeGuestU32(m_vsyncFlagAddress, 1u);
        if (m_vsyncTickAddress != 0u)
        {
            const uint32_t physical = m_vsyncTickAddress & 0x1FFFFFFFu;
            if (m_rdram && physical <= PS2_RAM_SIZE - sizeof(uint64_t))
            {
                std::memcpy(m_rdram + physical, &m_vsyncTick, sizeof(m_vsyncTick));
            }
        }
        m_vsyncFlagAddress = 0u;
        m_vsyncTickAddress = 0u;
        completeVSync(m_vsyncTick);
        m_runtime.notifyIopVBlank();
        if (m_gsVSyncCallback != 0u && m_runtime.hasFunction(m_gsVSyncCallback))
        {
            GuestInvocation invocation{};
            invocation.kind = GuestInvocationKind::GsCallback;
            invocation.context.pc = m_gsVSyncCallback;
            SET_GPR_U32(&invocation.context, 4, static_cast<uint32_t>(m_vsyncTick));
            SET_GPR_U32(&invocation.context, 28, m_gsVSyncCallbackGp);
            SET_GPR_U32(&invocation.context, 29, m_gsVSyncCallbackSp);
            SET_GPR_U32(&invocation.context, 31, 0u);
            queueInvocation(std::move(invocation));
        }
        dispatchIrq(false, 2u);
        break;
    case EeEventType::ExternalWake:
        completeExternalWait(event.id, event.value, KE_OK);
        break;
    case EeEventType::VBlankEnd:
        dispatchIrq(false, 3u);
        break;
    case EeEventType::Dmac:
        break;
    case EeEventType::Alarm:
    {
        auto it = m_alarms.find(static_cast<int>(event.id));
        if (it == m_alarms.end())
        {
            break;
        }
        const EeAlarm alarm = it->second;
        m_alarms.erase(it);
        GuestInvocation invocation{};
        invocation.kind = GuestInvocationKind::Alarm;
        invocation.context.pc = alarm.handler;
        SET_GPR_U32(&invocation.context, 4, static_cast<uint32_t>(alarm.id));
        SET_GPR_U32(&invocation.context, 5, static_cast<uint32_t>(alarm.ticks));
        SET_GPR_U32(&invocation.context, 6, alarm.argument);
        SET_GPR_U32(&invocation.context, 28, alarm.gp);
        SET_GPR_U32(&invocation.context, 29, alarm.sp);
        SET_GPR_U32(&invocation.context, 31, 0u);
        queueInvocation(std::move(invocation));
        break;
    }
    }
}

void EeScheduler::finishEventWaiters(EeEventFlag &flag, bool interruptSafe)
{
    for (auto it = flag.waiters.begin(); it != flag.waiters.end();)
    {
        GuestThread *waiter = thread(*it);
        assert(waiter != nullptr);
        const EeEventFlagWait wait = std::get<EeEventFlagWait>(waiter->wait.payload);
        if (!eventCondition(flag.bits, wait.bits, wait.mode))
        {
            ++it;
            continue;
        }
        const uint32_t observed = flag.bits;
        writeGuestU32(wait.resultAddress, observed);
        if ((wait.mode & WEF_CLEAR_ALL) != 0u)
        {
            flag.bits = 0;
        }
        else if ((wait.mode & WEF_CLEAR) != 0u)
        {
            flag.bits &= ~wait.bits;
        }
        it = flag.waiters.erase(it);
        makeReady(*waiter, KE_OK, interruptSafe);
    }
}

bool EeScheduler::eventCondition(uint32_t current, uint32_t requested, uint32_t mode)
{
    return (mode & WEF_OR) != 0u ? (current & requested) != 0u
                                 : (current & requested) == requested;
}

int EeScheduler::waitObjectId(const EeWaitState &wait)
{
    switch (wait.reason)
    {
    case EeWaitReason::Semaphore:
        return std::get<EeSemaphoreWait>(wait.payload).id;
    case EeWaitReason::EventFlag:
        return std::get<EeEventFlagWait>(wait.payload).id;
    default:
        return 0;
    }
}

void EeScheduler::writeGuestU32(uint32_t address, uint32_t value)
{
    if (address == 0u)
    {
        return;
    }
    const uint32_t physical = address & 0x1FFFFFFFu;
    if (!m_rdram || physical > PS2_RAM_SIZE - sizeof(value))
    {
        return;
    }
    std::memcpy(m_rdram + physical, &value, sizeof(value));
}

void EeScheduler::waitForEvent()
{
    std::unique_lock lock(m_eventMutex);
    if (!m_events.empty() || m_stopRequested.load(std::memory_order_acquire))
    {
        return;
    }
    const uint64_t timerCycles = m_runtime.memory().cyclesUntilNextEeTimerInterrupt();
    const bool hasTimerDeadline = timerCycles != std::numeric_limits<uint64_t>::max();
    if (m_deadlines.empty() && !hasTimerDeadline)
    {
        m_eventCv.wait(lock, [this]()
                       { return !m_events.empty() || m_stopRequested.load(std::memory_order_acquire); });
        return;
    }

    uint64_t deadlineCycle = 0u;
    auto hostDeadline = std::chrono::steady_clock::time_point::max();
    if (!m_deadlines.empty())
    {
        const auto next = std::min_element(m_deadlines.begin(), m_deadlines.end(),
                                           [](const ScheduledEvent &left, const ScheduledEvent &right)
                                           {
                                               if (left.deadlineCycle != right.deadlineCycle)
                                               {
                                                   return left.deadlineCycle < right.deadlineCycle;
                                               }
                                               return left.sequence < right.sequence;
                                           });
        deadlineCycle = next->deadlineCycle;
        hostDeadline = next->hostDeadline;
    }
    if (hasTimerDeadline)
    {
        const auto timerHostDeadline = std::chrono::steady_clock::now() + eeCyclesToHostDuration(timerCycles);
        if (timerHostDeadline < hostDeadline)
        {
            deadlineCycle = m_eeCycle + timerCycles;
            hostDeadline = timerHostDeadline;
        }
    }

    const bool signaled = m_eventCv.wait_until(lock, hostDeadline, [this]()
                                               { return !m_events.empty() ||
                                                        m_stopRequested.load(std::memory_order_acquire); });
    if (!signaled)
    {
        const uint64_t elapsed = deadlineCycle > m_eeCycle ? deadlineCycle - m_eeCycle : 0u;
        lock.unlock();
        uint64_t remaining = elapsed;
        while (remaining > 0u)
        {
            const uint32_t step = static_cast<uint32_t>(std::min<uint64_t>(remaining, std::numeric_limits<uint32_t>::max()));
            accountCycles(step);
            remaining -= step;
        }
        m_checkpointPending.store(true, std::memory_order_release);
    }
}

void EeScheduler::scheduleEvent(uint64_t deadlineCycle,
                                std::chrono::steady_clock::time_point hostDeadline,
                                EeEvent event)
{
    {
        std::lock_guard lock(m_eventMutex);
        m_deadlines.push_back(ScheduledEvent{deadlineCycle, hostDeadline, event, ++m_eventSequence});
        updateNextDeadline();
    }
    m_eventCv.notify_one();
}

void EeScheduler::updateNextDeadline()
{
    if (m_deadlines.empty())
    {
        m_nextDeadlineCycle.store(0u, std::memory_order_release);
        return;
    }
    const auto it = std::min_element(m_deadlines.begin(), m_deadlines.end(),
                                     [](const ScheduledEvent &left, const ScheduledEvent &right)
                                     {
                                         if (left.deadlineCycle != right.deadlineCycle)
                                         {
                                             return left.deadlineCycle < right.deadlineCycle;
                                         }
                                         return left.sequence < right.sequence;
                                     });
    m_nextDeadlineCycle.store(it->deadlineCycle, std::memory_order_release);
}

bool EeScheduler::hasReadyAtOrAbovePriority(int priority) const
{
    const int last = std::clamp(priority, 0, kPriorityCount - 1);
    for (int p = 0; p <= last; ++p)
    {
        if (!m_readyQueues[static_cast<size_t>(p)].empty())
        {
            return true;
        }
    }
    return false;
}

void EeScheduler::renewTimeSlice()
{
    m_sliceEndCycle = m_eeCycle + kDefaultTimeSliceCycles;
    m_timeSliceExpired = false;
}

void EeScheduler::copyMainContextToRuntime()
{
    const GuestThread *main = thread(kMainThreadId);
    if (main)
    {
        m_runtime.m_cpuContext = main->context;
    }
}
