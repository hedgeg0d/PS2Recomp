#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>
#include <string_view>

namespace ps2x::iop::detail
{
    namespace
    {
        // PS2LIB_Sound exposes one SIF RPC port for its command dispatcher.
        // The port is part of the middleware ABI and is shared by titles that
        // load this module; it is not a game/profile address binding.
        constexpr uint32_t kPs2libSoundSid = 0x00010001u;
        constexpr uint32_t kMaximumResponseBytes = 0x100u;

        // The Sound middleware declares its per-channel work areas inside the
        // SndInit (function 0) packet. Each entry is a 0x40-byte caller-owned
        // block; the IOP module writes completion state back into that memory
        // (the real module's gNp_TransToEE emits a SIF DMA from the block
        // recorded at SndInit). The EE client polls byte +0x37 as its
        // "command in flight" flag and expects the server to clear it after a
        // request has been consumed. These constants describe that transport
        // contract, not a particular game's addresses.
        constexpr uint32_t kWorkAreaCount = 2u;
        constexpr uint32_t kWorkAreaStride = 0x40u;
        constexpr uint32_t kWorkBusyOffset = 0x37u;
        constexpr uint32_t kGuestRamMask = 0x1FFFFFFFu;

        class Ps2libSoundService final : public IopService
        {
        public:
            explicit Ps2libSoundService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "PS2LIB Sound";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
                m_workAreas.fill(0u);
                m_workAreaLogCount = 0u;
                m_busyProbeLogCount = 0u;
                m_rpcProbeLogCount = 0u;
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid != kPs2libSoundSid)
                {
                    return result;
                }

                result.handled = true;
                result.resultAddress = request.receive.address;

                // Keep a bounded transport trace while validating middleware
                // progress.  This records the generic RPC contract (function
                // and buffer sizes), never a title-specific address.
                if (m_rpcProbeLogCount < 96u)
                {
                    std::cerr << "[ps2lib_sound] rpc fn=0x" << std::hex
                              << request.function << " send=0x" << request.send.size
                              << " recv=0x" << request.receive.size << std::dec
                              << std::endl;
                    ++m_rpcProbeLogCount;
                }

                // SndInit publishes the client-owned work areas. They live in
                // guest RAM at the addresses the client itself supplied, so
                // remembering them here is pure middleware transport.
                if (request.function == 0u)
                {
                    rememberWorkAreas(request);
                }

                // PS2LIB_Sound is a transport-facing wrapper around the
                // same host audio endpoint used by libsd. Forwarding the
                // request keeps the core service useful for every title
                // using this middleware and leaves command interpretation to
                // the selected audio backend.
                m_host.audioCommand(request.sid,
                                    request.function,
                                    request.send,
                                    request.receive);

                // The original dispatcher returns a pointer to a shared
                // 32-bit result slot. Keep the transport contract intact for
                // every command while the host audio backend handles actual
                // SPU work through its normal libsd path.
                if (request.receive.address != 0u && request.receive.size != 0u)
                {
                    std::array<uint8_t, kMaximumResponseBytes> response{};
                    (void)m_host.writeGuest(request.receive.address,
                                            response.data(),
                                            std::min<size_t>(request.receive.size, response.size()));
                }

                // The EE client keeps byte +0x37 of its work area set while a
                // request is in flight and blocks until the server consumes
                // it. The real module clears the flag as part of reporting the
                // result back into the client-owned block; mirror that here for
                // every served command.
                clearWorkBusyFlags();
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

            void rememberWorkAreas(const RpcRequest &request)
            {
                uint32_t first = 0u;
                uint32_t second = 0u;
                if (!readU32(request, 0u, first))
                {
                    return;
                }
                const uint32_t secondCandidate = readU32(request, 4u, second) ? second : 0u;

                m_workAreas[0] = first & kGuestRamMask;
                m_workAreas[1] = secondCandidate != 0u ? (secondCandidate & kGuestRamMask) : 0u;

                if (m_workAreaLogCount < 4u)
                {
                    std::cerr << "[ps2lib_sound] SndInit work areas: 0x" << std::hex
                              << m_workAreas[0] << " 0x" << m_workAreas[1] << std::dec << std::endl;
                    ++m_workAreaLogCount;
                }
            }

            void clearWorkBusyFlags()
            {
                const uint8_t zero = 0u;
                for (uint32_t area : m_workAreas)
                {
                    if (area == 0u)
                    {
                        continue;
                    }
                    const bool wrote = m_host.writeGuest(area + kWorkBusyOffset, &zero, 1u);
                    if (m_busyProbeLogCount < 8u)
                    {
                        uint8_t value = 0xFFu;
                        const bool read = m_host.readGuest(area + kWorkBusyOffset, &value, 1u);
                        std::cerr << "[ps2lib_sound] clear busy area=0x" << std::hex << area
                                  << " wrote=" << (wrote ? 1 : 0)
                                  << " read=" << (read ? 1 : 0)
                                  << " value=0x" << static_cast<unsigned>(value)
                                  << std::dec << std::endl;
                        ++m_busyProbeLogCount;
                    }
                }
            }

            IopHost &m_host;
            std::array<uint32_t, kWorkAreaCount> m_workAreas{0u, 0u};
            uint32_t m_workAreaLogCount = 0u;
            uint32_t m_busyProbeLogCount = 0u;
            uint32_t m_rpcProbeLogCount = 0u;
            inline static constexpr std::array<uint32_t, 1u> kSids{kPs2libSoundSid};
        };
    }

    std::unique_ptr<IopService> createPs2libSoundService(IopHost &host)
    {
        return std::make_unique<Ps2libSoundService>(host);
    }
}
