#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        constexpr uint32_t kIopHeapSid = 0x80000003u;
        constexpr uint32_t kFunctionAllocate = 1u;
        constexpr uint32_t kFunctionFree = 2u;
        constexpr uint32_t kFunctionLoad = 3u;
        constexpr uint32_t kPathOffset = sizeof(uint32_t);
        constexpr uint32_t kPathSize = 252u;
        constexpr size_t kCopyChunkSize = 64u * 1024u;

        class IopHeapService final : public IopService
        {
        public:
            explicit IopHeapService(IopHost &host)
                : m_host(host)
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "IOPHEAP";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return kSids;
            }

            void reset() override
            {
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                if (request.sid != kIopHeapSid)
                {
                    return result;
                }

                result.handled = true;
                result.resultAddress = request.receive.address;
                switch (request.function)
                {
                case kFunctionAllocate:
                    handleAllocate(request);
                    break;
                case kFunctionFree:
                    handleFree(request);
                    break;
                case kFunctionLoad:
                    handleLoad(request);
                    break;
                default:
                    // The stock IOPHEAP server has no operation 4: its
                    // shared output slot is left at its neutral value.  Keep
                    // unknown extensions harmless instead of manufacturing
                    // an invalid guest address (0xffffffff) that a caller
                    // may feed into a subsequent SIF DMA.
                    writeResult(request, 0u);
                    break;
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

            [[nodiscard]] std::string readPath(const RpcRequest &request) const
            {
                if (request.send.address == 0u || request.send.size <= kPathOffset)
                {
                    return {};
                }

                const uint32_t available = std::min<uint32_t>(
                    kPathSize, request.send.size - kPathOffset);
                std::string path(available, '\0');
                if (!m_host.readGuest(request.send.address + kPathOffset,
                                      path.data(),
                                      path.size()))
                {
                    return {};
                }
                const std::size_t terminator = path.find('\0');
                if (terminator != std::string::npos)
                {
                    path.resize(terminator);
                }
                return path;
            }

            void writeResult(const RpcRequest &request, uint32_t value) const
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(value))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
            }

            void handleAllocate(const RpcRequest &request) const
            {
                uint32_t size = 0u;
                const uint32_t address = readU32(request, 0u, size)
                                             ? m_host.allocateIopHeap(size)
                                             : 0u;
                writeResult(request, address);
            }

            void handleFree(const RpcRequest &request) const
            {
                uint32_t address = 0u;
                const bool valid = readU32(request, 0u, address) &&
                                   m_host.freeIopHeap(address);
                writeResult(request, valid ? 0u : static_cast<uint32_t>(-1));
            }

            void handleLoad(const RpcRequest &request) const
            {
                uint32_t destination = 0u;
                const std::string path = readPath(request);
                if (!readU32(request, 0u, destination) || destination == 0u || path.empty())
                {
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }

                const std::string translated = m_host.translateGuestPath(path);
                const uint64_t handle = translated.empty() ? 0u : m_host.openHostFile(translated);
                if (handle == 0u)
                {
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }

                uint64_t fileSize = 0u;
                bool success = m_host.hostFileSize(handle, fileSize) &&
                               fileSize <= std::numeric_limits<uint32_t>::max();
                std::vector<uint8_t> buffer(kCopyChunkSize);
                for (uint64_t offset = 0u; success && offset < fileSize;)
                {
                    const size_t chunkSize = static_cast<size_t>(std::min<uint64_t>(
                        buffer.size(), fileSize - offset));
                    size_t bytesRead = 0u;
                    success = m_host.readHostFile(handle,
                                                  offset,
                                                  buffer.data(),
                                                  chunkSize,
                                                  bytesRead) &&
                              bytesRead == chunkSize &&
                              offset <= std::numeric_limits<uint32_t>::max() - destination &&
                              m_host.writeGuest(destination + static_cast<uint32_t>(offset),
                                                buffer.data(),
                                                bytesRead);
                    offset += bytesRead;
                }
                m_host.closeHostFile(handle);
                writeResult(request, success ? 0u : static_cast<uint32_t>(-1));
            }

            IopHost &m_host;
            static constexpr std::array<uint32_t, 1u> kSids{kIopHeapSid};
        };
    }

    std::unique_ptr<IopService> createIopHeapService(IopHost &host)
    {
        return std::make_unique<IopHeapService>(host);
    }
}
