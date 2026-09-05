#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <iostream>
#include <string_view>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        // filectrl is the public RPC port of the PS2LIB file-control
        // middleware. Its protocol is shared by the IOP module and EE
        // client; it is not an address from a particular game's ELF.
        constexpr uint32_t kFilectrlSid = 0x00010000u;
        constexpr uint32_t kReportSize = 0x40u;
        constexpr uint32_t kSectorSize = 2048u;
        // The module allocates a 0x20000-byte system-memory ring buffer and
        // stages every guest read through it; transfers to the EE drain the
        // ring with wrap-around.
        constexpr uint32_t kRingBufferSize = 0x20000u;
        // The read thread caps a single CD transfer at 0x8000 bytes and keeps
        // reading until the descriptor is drained.
        constexpr uint32_t kMaxChunkBytes = 0x8000u;
        constexpr uint8_t kDiskTypeDvd = 0x14u;
        // sceCdStatus "drive is reading" state published by the status query.
        constexpr uint8_t kDriveStatusReading = 0x06u;
        constexpr uint8_t kDriveStatusSpin = 0x02u;
        // Generic failure marker; the module writes 0xFF into state fields on
        // failed completions.
        constexpr uint8_t kCdErrorGeneric = 0xFFu;

        // Report offsets inside the 0x40-byte block that the module transfers
        // to the caller-owned EE control block.
        constexpr uint32_t kReportTrayOffset = 0x0Bu;
        constexpr uint32_t kReportReadyOffset = 0x14u;
        constexpr uint32_t kReportErrorOffset = 0x15u;
        constexpr uint32_t kReportDriveStatusOffset = 0x16u;
        constexpr uint32_t kReportDiskTypeOffset = 0x17u;
        constexpr uint32_t kReportCompleteOffset = 0x28u;
        constexpr uint32_t kReportClockOffset = 0x30u;
        constexpr uint32_t kReportClockValidOffset = 0x38u;

        uint8_t toBcd(uint32_t value)
        {
            value %= 100u;
            return static_cast<uint8_t>(((value / 10u) << 4u) | (value % 10u));
        }

        void writeLe32(std::array<uint8_t, kReportSize> &report,
                       uint32_t offset,
                       uint32_t value)
        {
            if (offset + sizeof(value) <= report.size())
            {
                std::memcpy(report.data() + offset, &value, sizeof(value));
            }
        }

        class FilectrlService final : public IopService
        {
        public:
            explicit FilectrlService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "PS2LIB Filectrl";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
                m_controlBlockAddress = 0u;
                m_sectorBase = 0u;
                m_spinMode = 0u;
                m_ringReadOffset = 0u;
                m_ringWriteOffset = 0u;
                m_bufferedBytes = 0u;
                m_readRequestActive = false;
                m_readCancelPending = false;
                m_readNextLsn = 0u;
                m_readSectorOffset = 0u;
                m_readRemainBytes = 0u;
                m_readDiscardBytes = 0u;
                m_eofCompletionPending = false;
                m_eofGracePolls = 0u;
                m_lastReadLbn = 0u;
                m_lastReadSize = 0u;
                m_lastReadFlags = 0u;
                m_lastReadMode = 0u;
                m_lastResultWritten = false;
                m_report.fill(0u);
                m_ring.fill(0u);
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid != kFilectrlSid)
                {
                    return result;
                }

                result.handled = true;
                result.resultAddress = request.receive.address;
                m_lastResultWritten = false;

                // The real module services reads from a dedicated thread that
                // runs concurrently with the RPC dispatcher; servicing RPCs
                // advances the same read state.
                pumpRead();

                switch (request.function)
                {
                case 0u:
                    initialize(request);
                    break;
                case 1u:
                    setSpindleMotor(request);
                    break;
                case 2u:
                    addSector(request);
                    break;
                case 3u:
                    startRead(request);
                    break;
                case 4u:
                    transferBuffer(request);
                    break;
                case 5u:
                    cancelRead();
                    break;
                case 6u:
                    publishStatus();
                    break;
                case 7u:
                    standbyDrive();
                    break;
                case 8u:
                    stopDrive();
                    break;
                case 9u:
                    trayControl(request);
                    break;
                case 10u:
                    readClock();
                    break;
                case 11u:
                case 12u:
                    break;
                default:
                    break;
                }

                if (request.receive.address != 0u && request.receive.size != 0u &&
                    !m_lastResultWritten)
                {
                    std::array<uint8_t, 0x100u> response{};
                    (void)m_host.writeGuest(request.receive.address,
                                            response.data(),
                                            std::min<size_t>(request.receive.size, response.size()));
                }
                return result;
            }

        private:
            [[nodiscard]] bool readU32(const RpcRequest &request,
                                       uint32_t offset,
                                       uint32_t &value) const
            {
                value = 0u;
                return request.send.address != 0u &&
                       request.send.size >= offset + sizeof(value) &&
                       m_host.readGuest(request.send.address + offset, &value, sizeof(value));
            }

            [[nodiscard]] bool readU8(const RpcRequest &request,
                                      uint32_t offset,
                                      uint8_t &value) const
            {
                value = 0u;
                return request.send.address != 0u &&
                       request.send.size > offset &&
                       m_host.readGuest(request.send.address + offset, &value, sizeof(value));
            }

            void writeResult(const RpcRequest &request, uint32_t value)
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(value))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
                m_lastResultWritten = true;
            }

            // TransReport: copy the persistent 0x40-byte report block into the
            // caller-owned EE control block. The block keeps its state between
            // transfers; handlers mutate individual fields. Every transfer
            // first refreshes the buffering field from the ring counter so the
            // client can poll how many bytes are ready for transfer.
            void publishReport()
            {
                writeLe32(m_report, 0x10u, m_bufferedBytes);
                // Buffered transfers publish a non-zero checksum marker in
                // the report header once data is available.  The EE reader
                // uses the marker to distinguish an empty ring from a
                // completed compressed block.
                const uint16_t checksumMarker = m_bufferedBytes != 0u ? 1u : 0u;
                std::memcpy(m_report.data() + 0x0Au, &checksumMarker, sizeof(checksumMarker));
                if (m_controlBlockAddress != 0u)
                {
                    (void)m_host.writeGuest(m_controlBlockAddress, m_report.data(), m_report.size());
                }
                static uint32_t reportProbeCount = 0u;
                if (reportProbeCount < 32u)
                {
                    std::cerr << "[probe:filectrl-report] control=0x" << std::hex
                              << m_controlBlockAddress << " buffered=0x" << m_bufferedBytes
                              << " ready=0x" << static_cast<unsigned>(m_report[0x14u])
                              << " error=0x" << static_cast<unsigned>(m_report[0x15u])
                              << " status=0x" << static_cast<unsigned>(m_report[0x16u])
                              << " disk=0x" << static_cast<unsigned>(m_report[0x17u])
                              << " complete=0x" << static_cast<unsigned>(m_report[0x28u])
                              << std::dec << std::endl;
                    ++reportProbeCount;
                }
            }

            void initialize(const RpcRequest &request)
            {
                uint32_t controlBlockAddress = 0u;
                if (!readU32(request, 0u, controlBlockAddress))
                {
                    return;
                }

                m_controlBlockAddress = controlBlockAddress;
                m_sectorBase = 0u;
                m_spinMode = 0u;
                m_ringReadOffset = 0u;
                m_ringWriteOffset = 0u;
                m_bufferedBytes = 0u;
                m_readRequestActive = false;
                m_readCancelPending = false;
                m_readNextLsn = 0u;
                m_readSectorOffset = 0u;
                m_readRemainBytes = 0u;
                m_readDiscardBytes = 0u;
                m_lastReadLbn = 0u;
                m_lastReadSize = 0u;
                m_lastReadFlags = 0u;
                m_lastReadMode = 0u;

                m_report.fill(0u);
                // The module marks its report ready right after initialization.
                m_report[kReportReadyOffset] = 1u;
                m_report[kReportErrorOffset] = 0u;
                m_report[kReportDriveStatusOffset] = 0u;
                m_report[kReportDiskTypeOffset] = kDiskTypeDvd;
                publishReport();
            }

            void setSpindleMotor(const RpcRequest &request)
            {
                // The real handler only latches the requested spin mode; it
                // does not touch the report.
                (void)readU8(request, 0u, m_spinMode);
            }

            void addSector(const RpcRequest &request)
            {
                // Stored verbatim; read starts add it when the request asks
                // for a sector-relative LBN.
                (void)readU32(request, 0u, m_sectorBase);
            }

            void startRead(const RpcRequest &request)
            {
                uint32_t startLbn = 0u;
                uint32_t byteCount = 0u;
                uint32_t flags = 0u;
                uint8_t modeByte = 0u;
                if (!readU32(request, 0u, startLbn) ||
                    !readU32(request, 4u, byteCount) ||
                    !readU32(request, 8u, flags) ||
                    !readU8(request, 12u, modeByte))
                {
                    return;
                }

                const uint32_t bufferingMode = ((flags >> 3u) & 1u) != 0u ? 0x10u : 0u;
                const bool sameRequest = startLbn == m_lastReadLbn &&
                                         byteCount == m_lastReadSize &&
                                         flags == m_lastReadFlags &&
                                         modeByte == m_lastReadMode;
                // EE clients may poll/re-submit the same start packet while
                // the asynchronous read is still active.  Treat that as an
                // idempotent wake-up; restarting would rewind the stream and
                // feed duplicate bytes to the consumer.
                if (sameRequest && m_readRequestActive)
                {
                    pumpRead();
                    return;
                }
                // Once a read has completed, a repeated packet starts a new
                // logical stream at its explicitly requested LBN.
                m_readNextLsn = (modeByte & 1u) != 0u ? startLbn + m_sectorBase : startLbn;
                m_readSectorOffset = 0u;
                m_lastReadLbn = startLbn;
                m_lastReadSize = byteCount;
                m_lastReadFlags = flags;
                m_lastReadMode = modeByte;
                // Keep the logical request length exact.  The CD backend still
                // rounds each physical transfer to complete sectors below,
                // but sector padding is not part of the guest-visible stream.
                // This is important for buffered requests: exposing the
                // rounded tail makes the EE descriptor underflow at EOF.
                const uint32_t framedBytes = byteCount + bufferingMode;
                m_readRemainBytes = framedBytes;
                // Buffered reads include a 16-byte middleware header before
                // the payload exposed through BuffTrans.  Keep the extra
                // bytes in the asynchronous read accounting, but consume
                // them before publishing data into the client ring.
                m_readDiscardBytes = bufferingMode;
                m_readCancelPending = false;
                m_readRequestActive = true;
                m_report[kReportErrorOffset] = 0u;
                static uint32_t readStartProbeCount = 0u;
                if (readStartProbeCount++ < 32u)
                {
                    std::cerr << "[probe:filectrl-read] lbn=0x" << std::hex << startLbn
                              << " next=0x" << m_readNextLsn << " size=0x" << byteCount
                              << " flags=0x" << flags << " sectorBase=0x" << m_sectorBase
                              << " framed=0x" << framedBytes
                              << " discard=0x" << m_readDiscardBytes
                              << " buffered=0x" << m_bufferedBytes << std::dec << '\n';
                }
                // The module wakes its read thread here; servicing the request
                // starts filling the ring buffer.
                pumpRead();
            }

            // Read-thread pass: stage up to kMaxChunkBytes per pass from the
            // CD backend into the ring buffer, honouring its wrap-around.
            void pumpRead()
            {
                if (m_readCancelPending)
                {
                    m_readCancelPending = false;
                    m_readRequestActive = false;
                    m_readRemainBytes = 0u;
                    m_readDiscardBytes = 0u;
                    m_readSectorOffset = 0u;
                    return;
                }
                if (!m_readRequestActive)
                {
                    return;
                }

                while (m_readRemainBytes != 0u && m_bufferedBytes < kRingBufferSize)
                {
                    const uint32_t freeBytes = kRingBufferSize - m_bufferedBytes;
                    const uint32_t chunk = std::min<uint32_t>(
                        {m_readRemainBytes, kMaxChunkBytes, freeBytes});
                    if (chunk == 0u)
                    {
                        break;
                    }
                    // Keep the source position at byte granularity.  A
                    // buffered read may consume a partial sector (for
                    // example after discarding its protocol header); merely
                    // advancing the LSN by ceil(bytes/sector) would skip the
                    // remainder and corrupt the byte stream at every ring
                    // boundary.
                    const uint32_t sectors =
                        (m_readSectorOffset + chunk + kSectorSize - 1u) / kSectorSize;
                    const size_t readBytes = static_cast<size_t>(sectors) * kSectorSize;

                    std::vector<uint8_t> data(readBytes, 0u);
                    if (!m_host.readCdSectors(m_readNextLsn, sectors, data.data(), readBytes))
                    {
                        m_readRequestActive = false;
                        m_readRemainBytes = 0u;
                        m_readDiscardBytes = 0u;
                        m_readSectorOffset = 0u;
                        m_report[kReportErrorOffset] = kCdErrorGeneric;
                        publishReport();
                        return;
                    }

                    // Sector reads are rounded up for the CD backend, but the
                    // ring exposes exactly the requested byte count.  Do not
                    // publish the sector padding as an uninitialized gap.
                    const uint32_t discard = std::min<uint32_t>(m_readDiscardBytes, chunk);
                    if (discard != 0u)
                    {
                        m_readDiscardBytes -= discard;
                    }
                    const uint32_t payloadBytes = chunk - discard;
                    if (payloadBytes != 0u)
                    {
                        copyIntoRing(data.data() + m_readSectorOffset + discard, payloadBytes);
                    }
                    m_ringWriteOffset = (m_ringWriteOffset + payloadBytes) %
                                        kRingBufferSize;
                    m_bufferedBytes += payloadBytes;
                    const uint32_t consumed = m_readSectorOffset + chunk;
                    m_readNextLsn += consumed / kSectorSize;
                    m_readSectorOffset = consumed % kSectorSize;
                    m_readRemainBytes -= chunk;
                }

                if (m_readRemainBytes == 0u)
                {
                    m_readRequestActive = false;
                    // End-of-stream raises the completion flag; the drive
                    // state itself is derived by GetStatus from request
                    // activity (Reading while active, Spin when idle).
                    m_report[kReportCompleteOffset] = 1u;
                    publishReport();
                }
            }

            void copyIntoRing(const uint8_t *data, size_t size)
            {
                const uint32_t tail = kRingBufferSize - m_ringWriteOffset;
                const size_t headBytes = std::min<size_t>(size, tail);
                std::memcpy(m_ring.data() + m_ringWriteOffset, data, headBytes);
                if (size > headBytes)
                {
                    std::memcpy(m_ring.data(), data + headBytes, size - headBytes);
                }
            }

            void copyFromRing(uint32_t destination, uint32_t size)
            {
                std::array<uint8_t, 0x800u> staging{};
                uint32_t offset = m_ringReadOffset;
                uint32_t remaining = size;
                while (remaining != 0u)
                {
                    const uint32_t contiguous = std::min<uint32_t>(
                        {remaining, kRingBufferSize - offset, static_cast<uint32_t>(staging.size())});
                    const uint32_t sourceOffset = offset;
                    std::memcpy(staging.data(), m_ring.data() + sourceOffset, contiguous);
                    (void)m_host.writeGuest(destination, staging.data(), contiguous);
                    destination += contiguous;
                    offset = (offset + contiguous) % kRingBufferSize;
                    remaining -= contiguous;
                }
            }

            void transferBuffer(const RpcRequest &request)
            {
                uint32_t destination = 0u;
                uint32_t byteCount = 0u;
                if (!readU32(request, 0u, destination) ||
                    !readU32(request, 4u, byteCount) ||
                    destination == 0u)
                {
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }

                // The module rejects transfers larger than the staged data so
                // the client can poll for progress while a read is active.
                // At a completed read, however, the physical CD DMA is
                // sector-rounded while the logical request length is exact.
                // The EE consumer still submits its fixed-size final block;
                // expose the available tail and zero-pad the remainder rather
                // than turning a valid EOF into a permanent retry loop.
                pumpRead();
                if (m_bufferedBytes < byteCount)
                {
                    const bool completedTail = !m_readRequestActive &&
                                               m_readRemainBytes == 0u &&
                                               m_bufferedBytes != 0u;
                    if (completedTail)
                    {
                        const uint32_t availableBytes = m_bufferedBytes;
                        copyFromRing(destination, availableBytes);
                        if (byteCount > availableBytes)
                        {
                            std::array<uint8_t, 0x800u> zeros{};
                            uint32_t padDestination = destination + availableBytes;
                            uint32_t padding = byteCount - availableBytes;
                            while (padding != 0u)
                            {
                                const uint32_t chunk = std::min<uint32_t>(
                                    padding, static_cast<uint32_t>(zeros.size()));
                                (void)m_host.writeGuest(padDestination, zeros.data(), chunk);
                                padDestination += chunk;
                                padding -= chunk;
                            }
                        }
                        m_ringReadOffset = (m_ringReadOffset + availableBytes) %
                                           kRingBufferSize;
                        m_bufferedBytes = 0u;
                        // The producer reached logical EOF while this tail
                        // was still buffered.  Keep completion deferred for
                        // one status poll: asynchronous consumers use that
                        // edge to wake their reader and consume the final
                        // bytes already staged in the guest destination.
                        m_eofCompletionPending = true;
                        m_eofGracePolls = 1u;
                        m_report[kReportCompleteOffset] = 0u;
                        publishReport();
                        static uint32_t tailTransferProbeCount = 0u;
                        if (tailTransferProbeCount++ < 8u)
                        {
                            std::cerr << "[probe:filectrl-tail] dst=0x" << std::hex
                                      << destination << " requested=0x" << byteCount
                                      << " copied=0x" << availableBytes
                                      << " padded=0x" << (byteCount - availableBytes)
                                      << std::dec << '\n';
                        }
                        writeResult(request, 0u);
                        return;
                    }
                    static uint32_t transferResultProbeCount = 0u;
                    if (transferResultProbeCount++ < 96u)
                    {
                        std::cerr << "[probe:filectrl-result] dst=0x" << std::hex << destination
                                  << " size=0x" << byteCount << " result=-1 buffered=0x"
                                  << m_bufferedBytes << " active=" << std::dec << m_readRequestActive
                                  << " remain=0x" << std::hex << m_readRemainBytes << '\n';
                    }
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }

                copyFromRing(destination, byteCount);
                static uint32_t transferProbeCount = 0u;
                if (transferProbeCount < 8u)
                {
                    uint64_t transferHash = 1469598103934665603ull;
                    for (uint32_t i = 0u; i < byteCount; ++i)
                    {
                        transferHash ^= m_ring[(m_ringReadOffset + i) % kRingBufferSize];
                        transferHash *= 1099511628211ull;
                    }
                    std::array<uint8_t, 16u> sample{};
                    if (m_host.readGuest(destination, sample.data(), sample.size()))
                    {
                        std::cerr << "[probe:filectrl-transfer] dst=0x" << std::hex
                                  << destination << " size=0x" << byteCount << " fnv=0x" << transferHash << " bytes=";
                        for (uint8_t byte : sample)
                        {
                            std::cerr << static_cast<unsigned>(byte) << ' ';
                        }
                        std::cerr << std::dec << '\n';
                    }
                    ++transferProbeCount;
                }
                m_ringReadOffset = (m_ringReadOffset + byteCount) % kRingBufferSize;
                m_bufferedBytes -= byteCount;
                static uint32_t transferResultProbeCount = 0u;
                    if (transferResultProbeCount++ < 96u)
                    {
                        std::cerr << "[probe:filectrl-result] dst=0x" << std::hex << destination
                                  << " size=0x" << byteCount << " result=0 buffered=0x"
                                  << m_bufferedBytes << " active=" << std::dec << m_readRequestActive
                                  << " remain=0x" << std::hex << m_readRemainBytes
                                  << " next=0x" << m_readNextLsn << '\n';
                    }
                writeResult(request, 0u);
            }

            void cancelRead()
            {
                if (m_readRequestActive && !m_readCancelPending)
                {
                    // The real read thread is woken by Cancel and publishes
                    // its completion report asynchronously.  An RPC is the
                    // only scheduling boundary available to this HLE, so
                    // run that pending pass before returning the call.  This
                    // preserves the observable completion edge for clients
                    // that clear the ready byte and then wait for the report.
                    m_readCancelPending = true;
                    pumpRead();
                    publishReport();
                    return;
                }
                publishReport();
            }

            void publishStatus()
            {
                m_report[kReportErrorOffset] = 0u;
                // GetStatus publishes the drive state, not a constant. While
                // a read is staged or draining the drive is Reading (0x06);
                // with no active request it is back to Spin (0x02). Clients
                // use the Reading->Spin edge after the completion flag to
                // tear the stream down and run the next boot task; a constant
                // Reading leaves them waiting past end-of-stream.
                m_report[kReportDriveStatusOffset] =
                    m_readRequestActive ? kDriveStatusReading : kDriveStatusSpin;
                m_report[kReportDiskTypeOffset] = kDiskTypeDvd;
                if (m_eofCompletionPending)
                {
                    if (m_eofGracePolls != 0u)
                    {
                        --m_eofGracePolls;
                        m_report[kReportCompleteOffset] = 0u;
                    }
                    else
                    {
                        m_report[kReportCompleteOffset] = 1u;
                        m_eofCompletionPending = false;
                    }
                }
                publishReport();
            }

            void standbyDrive()
            {
                m_report[kReportCompleteOffset] = 0u;
                m_report[kReportCompleteOffset] = 1u;
                publishReport();
            }

            void stopDrive()
            {
                m_report[kReportCompleteOffset] = 0u;
                m_report[kReportCompleteOffset] = 1u;
                publishReport();
            }

            void trayControl(const RpcRequest &request)
            {
                uint8_t requestType = 0u;
                (void)readU8(request, 0u, requestType);
                (void)requestType;
                m_report[kReportCompleteOffset] = 0u;
                m_report[kReportTrayOffset] = 0u;
                m_report[kReportCompleteOffset] = 1u;
                publishReport();
            }

            void readClock()
            {
                std::time_t now = std::time(nullptr);
                std::tm localTime{};
#ifdef _WIN32
                localtime_s(&localTime, &now);
#else
                localtime_r(&now, &localTime);
#endif
                // sceCdCLOCK: status, second, minute, hour, pad, day, month,
                // year - all binary-coded decimal.
                m_report[kReportClockOffset + 0u] = 0u;
                m_report[kReportClockOffset + 1u] = toBcd(static_cast<uint32_t>(localTime.tm_sec));
                m_report[kReportClockOffset + 2u] = toBcd(static_cast<uint32_t>(localTime.tm_min));
                m_report[kReportClockOffset + 3u] = toBcd(static_cast<uint32_t>(localTime.tm_hour));
                m_report[kReportClockOffset + 4u] = 0u;
                m_report[kReportClockOffset + 5u] = toBcd(static_cast<uint32_t>(localTime.tm_mday));
                m_report[kReportClockOffset + 6u] = toBcd(static_cast<uint32_t>(localTime.tm_mon + 1));
                m_report[kReportClockOffset + 7u] = toBcd(static_cast<uint32_t>((localTime.tm_year + 1900) % 100));
                m_report[kReportErrorOffset] = 0u;
                m_report[kReportClockValidOffset] = 1u;
                publishReport();
            }

            IopHost &m_host;
            static constexpr std::array<uint32_t, 1u> kSids{kFilectrlSid};

            uint32_t m_controlBlockAddress = 0u;
            uint32_t m_sectorBase = 0u;
            uint8_t m_spinMode = 0u;
            std::array<uint8_t, kReportSize> m_report{};
            std::array<uint8_t, kRingBufferSize> m_ring{};
            uint32_t m_ringReadOffset = 0u;
            uint32_t m_ringWriteOffset = 0u;
            uint32_t m_bufferedBytes = 0u;
            bool m_readRequestActive = false;
            bool m_readCancelPending = false;
            uint32_t m_readNextLsn = 0u;
            uint32_t m_readSectorOffset = 0u;
            uint32_t m_readRemainBytes = 0u;
            uint32_t m_readDiscardBytes = 0u;
            bool m_eofCompletionPending = false;
            uint8_t m_eofGracePolls = 0u;
            uint32_t m_lastReadLbn = 0u;
            uint32_t m_lastReadSize = 0u;
            uint32_t m_lastReadFlags = 0u;
            uint8_t m_lastReadMode = 0u;
            bool m_lastResultWritten = false;
        };
    }

    std::unique_ptr<IopService> createFilectrlService(IopHost &host)
    {
        return std::make_unique<FilectrlService>(host);
    }
}
