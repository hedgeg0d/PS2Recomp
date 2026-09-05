#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

namespace ps2x::iop::detail
{
    namespace
    {
        // These are the public PADMAN RPC ports used by libpad. The old
        // ports are kept because some EE clients select them when the new
        // pair is unavailable.
        constexpr uint32_t kPadmanSid1 = 0x80000100u;
        constexpr uint32_t kPadmanSid2 = 0x80000101u;
        constexpr uint32_t kPadmanSid1Old = 0x8000010Fu;
        constexpr uint32_t kPadmanSid2Old = 0x8000011Fu;

        constexpr uint32_t kRpcFunction = 1u;
        constexpr uint32_t kCommandOpen = 0x01u;
        constexpr uint32_t kCommandSetMainMode = 0x06u;
        constexpr uint32_t kCommandGetButtonMask = 0x09u;
        constexpr uint32_t kCommandSetButtonInfo = 0x0Au;
        constexpr uint32_t kCommandGetPortMax = 0x0Cu;
        constexpr uint32_t kCommandGetSlotMax = 0x0Du;
        constexpr uint32_t kCommandClose = 0x0Eu;
        constexpr uint32_t kCommandEnd = 0x0Fu;
        constexpr uint32_t kCommandInit = 0x10u;
        constexpr uint32_t kCommandGetModuleVersion = 0x12u;

        constexpr uint32_t kCommandOpenOld = 0x80000100u;
        constexpr uint32_t kCommandInfoActOld = 0x80000102u;
        constexpr uint32_t kCommandInfoCombOld = 0x80000103u;
        constexpr uint32_t kCommandInfoModeOld = 0x80000104u;
        constexpr uint32_t kCommandSetMainModeOld = 0x80000105u;
        constexpr uint32_t kCommandGetButtonMaskOld = 0x80000108u;
        constexpr uint32_t kCommandSetButtonInfoOld = 0x80000109u;
        constexpr uint32_t kCommandGetPortMaxOld = 0x8000010Bu;
        constexpr uint32_t kCommandGetSlotMaxOld = 0x8000010Cu;
        constexpr uint32_t kCommandCloseOld = 0x8000010Du;
        constexpr uint32_t kCommandEndOld = 0x8000010Eu;

        constexpr uint32_t kMaximumResponseBytes = 0x100u;
        constexpr uint32_t kPadPortCount = 2u;
        constexpr uint32_t kPadSlotCount = 8u;
        constexpr uint32_t kPadAreaBytes = 0x100u;

        bool isPadmanSid(uint32_t sid)
        {
            return sid == kPadmanSid1 || sid == kPadmanSid2 ||
                   sid == kPadmanSid1Old || sid == kPadmanSid2Old;
        }

        uint32_t readU32(const std::array<uint8_t, kMaximumResponseBytes> &bytes,
                         uint32_t offset)
        {
            if (offset > bytes.size() - sizeof(uint32_t))
            {
                return 0u;
            }
            return static_cast<uint32_t>(bytes[offset]) |
                   (static_cast<uint32_t>(bytes[offset + 1u]) << 8u) |
                   (static_cast<uint32_t>(bytes[offset + 2u]) << 16u) |
                   (static_cast<uint32_t>(bytes[offset + 3u]) << 24u);
        }

        template <size_t Size>
        void writeU32(std::array<uint8_t, Size> &bytes,
                      uint32_t offset,
                      uint32_t value)
        {
            if (offset > bytes.size() - sizeof(uint32_t))
            {
                return;
            }
            bytes[offset] = static_cast<uint8_t>(value);
            bytes[offset + 1u] = static_cast<uint8_t>(value >> 8u);
            bytes[offset + 2u] = static_cast<uint8_t>(value >> 16u);
            bytes[offset + 3u] = static_cast<uint8_t>(value >> 24u);
        }

        void initializePadArea(std::array<uint8_t, kPadAreaBytes> &area)
        {
            area.fill(0u);

            // A connected digital pad reports the standard 0x41/0x5a
            // header, released buttons, and centered analog axes. The
            // double-buffered area is deterministic for a host without a
            // physical controller backend.
            area[0x00u] = 0x41u;
            area[0x01u] = 0x5Au;
            area[0x02u] = 0xFFu;
            area[0x03u] = 0xFFu;
            area[0x04u] = 0x80u;
            area[0x05u] = 0x80u;
            area[0x06u] = 0x80u;
            area[0x07u] = 0x80u;
            writeU32(area, 0x58u, 1u); // frame
            writeU32(area, 0x60u, 8u); // data length
            area[0x74u] = 6u;          // PAD_STATE_STABLE
            area[0x75u] = 0u;          // PAD_RSTAT_COMPLETE
            area[0x76u] = 1u;          // current task
        }

        class PadmanService final : public IopService
        {
        public:
            explicit PadmanService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "PADMAN";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
                for (auto &port : m_open)
                {
                    port.fill(false);
                }
                for (auto &port : m_padArea)
                {
                    port.fill(0u);
                }
                for (auto &port : m_frame)
                {
                    port.fill(0u);
                }
            }

            // Mirrors the SIO2 autopoll tick: refresh open ports' pad areas
            // with live host state so the guest never observes a frozen pad.
            void onVBlank() override
            {
                // TEMP-EXPERIMENT: prove padman gets VBlank. Revert.
                static int padVblank = 0;
                if (padVblank < 3)
                {
                    int open = 0;
                    for (const auto &port : m_open)
                    {
                        for (bool b : port)
                        {
                            open += b ? 1 : 0;
                        }
                    }
                    std::printf("[vblank-pad] n=%d open=%d\n", padVblank, open);
                    std::fflush(stdout);
                }
                ++padVblank;
                for (uint32_t port = 0u; port < kPadPortCount; ++port)
                {
                    for (uint32_t slot = 0u; slot < kPadSlotCount; ++slot)
                    {
                        if (!m_open[port][slot] || m_padArea[port][slot] == 0u)
                        {
                            continue;
                        }
                        refreshPadArea(port, slot);
                    }
                }
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                // TEMP-EXPERIMENT: log sio2man queries (card status?). Revert.
                {
                    static int sioCount = 0;
                    if (sioCount < 16)
                    {
                        ++sioCount;
                        std::array<uint8_t, 32> head{};
                        if (request.send.address != 0u && request.send.size != 0u)
                            (void)m_host.readGuest(request.send.address, head.data(), std::min<size_t>(request.send.size, head.size()));
                        std::fprintf(stdout, "[sioq] n=%d sid=0x%x fn=0x%x send=%u recv=%u cmd=0x%x bytes: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
                            sioCount, request.sid, request.function, request.send.size, request.receive.size,
                            static_cast<unsigned>(head[0] | (head[1]<<8) | (head[2]<<16) | (head[3]<<24)),
                            head[0], head[1], head[2], head[3], head[4], head[5], head[6], head[7],
                            head[8], head[9], head[10], head[11], head[12], head[13], head[14], head[15]);
                        std::fflush(stdout);
                    }
                }
                if (!isPadmanSid(request.sid))
                {
                    return result;
                }

                result.handled = true;
                result.resultAddress = request.receive.address;

                std::array<uint8_t, kMaximumResponseBytes> arguments{};
                if (request.send.address != 0u && request.send.size != 0u)
                {
                    (void)m_host.readGuest(request.send.address,
                                           arguments.data(),
                                           std::min<size_t>(request.send.size, arguments.size()));
                }

                std::array<uint8_t, kMaximumResponseBytes> response{};
                if (request.function == kRpcFunction)
                {
                    dispatchCommand(readU32(arguments, 0u), arguments, response);
                }
                else
                {
                    writeU32(response, 16u, 0u);
                }

                if (request.receive.address != 0u && request.receive.size != 0u)
                {
                    (void)m_host.writeGuest(request.receive.address,
                                            response.data(),
                                            std::min<size_t>(request.receive.size, response.size()));
                }
                return result;
            }

        private:
            void dispatchCommand(uint32_t command,
                                 const std::array<uint8_t, kMaximumResponseBytes> &arguments,
                                 std::array<uint8_t, kMaximumResponseBytes> &response)
            {
                switch (command)
                {
                case kCommandInit:
                    initializeConnectionState(arguments);
                    writeU32(response, 16u, 1u);
                    return;
                case kCommandOpen:
                case kCommandOpenOld:
                    handleOpen(arguments, response);
                    return;
                case kCommandClose:
                case kCommandCloseOld:
                    handleClose(arguments, response);
                    return;
                case kCommandGetPortMax:
                case kCommandGetPortMaxOld:
                    writeU32(response, 16u, kPadPortCount);
                    return;
                case kCommandGetSlotMax:
                case kCommandGetSlotMaxOld:
                    writeU32(response, 16u, 1u);
                    return;
                case kCommandGetModuleVersion:
                    // The ROM padman does not expose a meaningful module
                    // version through this command.
                    writeU32(response, 16u, 0u);
                    return;
                case kCommandEnd:
                case kCommandEndOld:
                    writeU32(response, 16u, 1u);
                    return;
                case kCommandSetMainMode:
                case kCommandSetMainModeOld:
                case kCommandGetButtonMask:
                case kCommandGetButtonMaskOld:
                case kCommandSetButtonInfo:
                case kCommandSetButtonInfoOld:
                case kCommandInfoActOld:
                case kCommandInfoCombOld:
                case kCommandInfoModeOld:
                    // These operations are asynchronous in the real
                    // module. A completed request is the useful portable
                    // behavior for a host without a physical pad.
                    writeU32(response, 16u, 1u);
                    return;
                default:
                    writeU32(response, 16u, 0u);
                    return;
                }
            }

            void initializeConnectionState(const std::array<uint8_t, kMaximumResponseBytes> &arguments)
            {
                const uint32_t statusAddress = readU32(arguments, 16u);
                if (statusAddress == 0u)
                {
                    return;
                }

                std::array<uint8_t, 0x80u> status{};
                writeU32(status, 4u, 1u);
                writeU32(status, 8u, 1u);
                (void)m_host.writeGuest(statusAddress, status.data(), status.size());
            }

            void handleOpen(const std::array<uint8_t, kMaximumResponseBytes> &arguments,
                            std::array<uint8_t, kMaximumResponseBytes> &response)
            {
                const uint32_t port = readU32(arguments, 4u);
                const uint32_t slot = readU32(arguments, 8u);
                const uint32_t padAreaAddress = readU32(arguments, 16u);
                if (port >= kPadPortCount || slot >= kPadSlotCount || padAreaAddress == 0u)
                {
                    return;
                }

                std::array<uint8_t, kPadAreaBytes> area{};
                initializePadArea(area);
                const bool firstBufferWritten = m_host.writeGuest(padAreaAddress, area.data(), area.size());
                const bool secondBufferWritten = m_host.writeGuest(padAreaAddress + kPadAreaBytes,
                                                                    area.data(),
                                                                    area.size());
                if (firstBufferWritten && secondBufferWritten)
                {
                    m_open[port][slot] = true;
                    m_padArea[port][slot] = padAreaAddress;
                    m_frame[port][slot] = 1u;
                    writeU32(response, 16u, 1u);
                }
            }

            void handleClose(const std::array<uint8_t, kMaximumResponseBytes> &arguments,
                             std::array<uint8_t, kMaximumResponseBytes> &response)
            {
                const uint32_t port = readU32(arguments, 4u);
                const uint32_t slot = readU32(arguments, 8u);
                if (port >= kPadPortCount || slot >= kPadSlotCount)
                {
                    return;
                }
                m_open[port][slot] = false;
                m_padArea[port][slot] = 0u;
                writeU32(response, 16u, 1u);
            }

            void refreshPadArea(uint32_t port, uint32_t slot)
            {
                std::array<uint8_t, 32u> state{};
                if (!m_host.readPadState(static_cast<int>(port),
                                         static_cast<int>(slot),
                                         state.data(),
                                         state.size()))
                {
                    return;
                }
                std::array<uint8_t, kPadAreaBytes> area{};
                initializePadArea(area);
                area[0x02u] = state[2];
                area[0x03u] = state[3];
                area[0x04u] = state[4];
                area[0x05u] = state[5];
                area[0x06u] = state[6];
                area[0x07u] = state[7];
                writeU32(area, 0x58u, ++m_frame[port][slot]);
                const uint32_t base = m_padArea[port][slot];
                (void)m_host.writeGuest(base, area.data(), area.size());
                (void)m_host.writeGuest(base + kPadAreaBytes, area.data(), area.size());
                // TEMP-EXPERIMENT: prove the live-pad chain works. Revert.
                const uint16_t buttons =
                    static_cast<uint16_t>(state[2] | (state[3] << 8));
                if (buttons != 0xFFFFu)
                {
                    std::printf("[pad-live] port=%u slot=%u buttons=0x%04x frame=%u area=0x%08x\n",
                                port,
                                slot,
                                buttons,
                                m_frame[port][slot],
                                base);
                    std::fflush(stdout);
                }
            }

            IopHost &m_host;
            static constexpr std::array<uint32_t, 4u> kSids{
                kPadmanSid1, kPadmanSid2, kPadmanSid1Old, kPadmanSid2Old};
            std::array<std::array<bool, kPadSlotCount>, kPadPortCount> m_open{};
            std::array<std::array<uint32_t, kPadSlotCount>, kPadPortCount> m_padArea{};
            std::array<std::array<uint32_t, kPadSlotCount>, kPadPortCount> m_frame{};
        };
    }

    std::unique_ptr<IopService> createPadmanService(IopHost &host)
    {
        return std::make_unique<PadmanService>(host);
    }
}
