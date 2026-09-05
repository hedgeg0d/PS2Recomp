#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

namespace ps2x::iop::detail
{
    namespace
    {
        constexpr uint32_t kLoadFileSid = 0x80000006u;

        constexpr uint32_t kFunctionModuleLoad = 0u;
        constexpr uint32_t kFunctionElfLoad = 1u;
        constexpr uint32_t kFunctionSetAddress = 2u;
        constexpr uint32_t kFunctionGetAddress = 3u;
        constexpr uint32_t kFunctionMagicGateModuleLoad = 5u;
        constexpr uint32_t kFunctionMagicGateElfLoad = 6u;
        constexpr uint32_t kFunctionModuleBufferLoad = 7u;
        constexpr uint32_t kFunctionModuleStop = 8u;
        constexpr uint32_t kFunctionModuleUnload = 9u;
        constexpr uint32_t kFunctionSearchModuleByName = 10u;
        constexpr uint32_t kFunctionSearchModuleByAddress = 11u;
        constexpr uint32_t kFunctionGetVersion = 0xFFu;

        constexpr uint32_t kPathOffset = 8u;
        constexpr uint32_t kPathSize = 252u;
        constexpr uint32_t kVersion = 0x30303133u;

        class LoadFileService final : public IopService
        {
        public:
            explicit LoadFileService(IopHost &host)
                : m_host(host),
                  m_sids{kLoadFileSid}
            {
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "LOADFILE";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return m_sids;
            }

            void reset() override
            {
                m_modules.clear();
                m_nextModuleId = 1;
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                result.handled = true;
                result.resultAddress = request.receive.address;

                switch (request.function)
                {
                case kFunctionModuleLoad:
                    handleModuleLoad(request);
                    break;
                case kFunctionElfLoad:
                case kFunctionMagicGateModuleLoad:
                case kFunctionMagicGateElfLoad:
                    handleElfLoad(request);
                    break;
                case kFunctionSetAddress:
                case kFunctionGetAddress:
                    writeResult(request, 0u);
                    break;
                case kFunctionModuleBufferLoad:
                    writeModuleResult(request, allocateModule("iop-buffer"));
                    break;
                case kFunctionModuleStop:
                    handleModuleStop(request);
                    break;
                case kFunctionModuleUnload:
                    handleModuleUnload(request);
                    break;
                case kFunctionSearchModuleByName:
                    handleSearchByName(request);
                    break;
                case kFunctionSearchModuleByAddress:
                    writeResult(request, 0u);
                    break;
                case kFunctionGetVersion:
                    writeResult(request, kVersion);
                    break;
                default:
                    writeResult(request, 0u);
                    break;
                }
                return result;
            }

            void appendDebugMetrics(std::vector<DebugMetric> &metrics) const override
            {
                metrics.push_back({"loaded_modules", m_modules.size(), false});
            }

        private:
            [[nodiscard]] std::string readGuestString(const RpcRequest &request,
                                                       uint32_t offset,
                                                       uint32_t maxBytes) const
            {
                if (request.send.address == 0u || request.send.size <= offset)
                {
                    return {};
                }

                const uint32_t available = std::min<uint32_t>(maxBytes, request.send.size - offset);
                std::string value(available, '\0');
                if (!m_host.readGuest(request.send.address + offset, value.data(), value.size()))
                {
                    return {};
                }
                const std::size_t terminator = value.find('\0');
                if (terminator != std::string::npos)
                {
                    value.resize(terminator);
                }
                return value;
            }

            [[nodiscard]] bool fileExists(std::string_view guestPath) const
            {
                const std::string translated = m_host.translateGuestPath(guestPath);
                if (translated.empty())
                {
                    return false;
                }

                const uint64_t handle = m_host.openHostFile(translated);
                if (handle == 0u)
                {
                    return false;
                }
                m_host.closeHostFile(handle);
                return true;
            }

            uint32_t allocateModule(std::string path)
            {
                const auto existing = m_modules.find(path);
                if (existing != m_modules.end())
                {
                    return existing->second;
                }

                const uint32_t id = m_nextModuleId++;
                m_modules.emplace(std::move(path), id);
                return id;
            }

            void handleModuleLoad(const RpcRequest &request)
            {
                const std::string path = readGuestString(request, kPathOffset, kPathSize);
                const uint32_t moduleId = fileExists(path) ? allocateModule(path) : 0u;
                writeModuleResult(request, moduleId == 0u ? -1 : static_cast<int32_t>(moduleId));
            }

            void handleElfLoad(const RpcRequest &request)
            {
                const std::string path = readGuestString(request, 8u, kPathSize);
                writeResult(request, fileExists(path) ? 0u : static_cast<uint32_t>(-1));
            }

            void handleModuleStop(const RpcRequest &request)
            {
                uint32_t moduleId = 0u;
                if (!readGuestU32(request, 0u, moduleId))
                {
                    writeModuleResult(request, -1);
                    return;
                }

                const auto it = std::find_if(m_modules.begin(), m_modules.end(),
                                             [moduleId](const auto &entry)
                                             { return entry.second == moduleId; });
                writeModuleResult(request, it == m_modules.end() ? -1 : 0);
            }

            void handleModuleUnload(const RpcRequest &request)
            {
                uint32_t moduleId = 0u;
                if (!readGuestU32(request, 0u, moduleId))
                {
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }

                const auto it = std::find_if(m_modules.begin(), m_modules.end(),
                                             [moduleId](const auto &entry)
                                             { return entry.second == moduleId; });
                if (it == m_modules.end())
                {
                    writeResult(request, static_cast<uint32_t>(-1));
                    return;
                }
                m_modules.erase(it);
                writeResult(request, 0u);
            }

            void handleSearchByName(const RpcRequest &request)
            {
                const std::string name = readGuestString(request, 8u, kPathSize);
                const auto it = m_modules.find(name);
                writeResult(request, it == m_modules.end() ? 0u : it->second);
            }

            [[nodiscard]] bool readGuestU32(const RpcRequest &request,
                                            uint32_t offset,
                                            uint32_t &value) const
            {
                value = 0u;
                return request.send.address != 0u &&
                       request.send.size >= offset + sizeof(value) &&
                       m_host.readGuest(request.send.address + offset, &value, sizeof(value));
            }

            void writeResult(const RpcRequest &request, uint32_t value) const
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(value))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
            }

            void writeModuleResult(const RpcRequest &request, int32_t moduleResult) const
            {
                std::array<int32_t, 2u> response{moduleResult, 0};
                if (request.receive.address != 0u && request.receive.size != 0u)
                {
                    (void)m_host.writeGuest(request.receive.address,
                                            response.data(),
                                            std::min<size_t>(request.receive.size, sizeof(response)));
                }
            }

            IopHost &m_host;
            const std::array<uint32_t, 1u> m_sids;
            std::unordered_map<std::string, uint32_t> m_modules;
            uint32_t m_nextModuleId = 1u;
        };
    }

    std::unique_ptr<IopService> createLoadFileService(IopHost &host)
    {
        return std::make_unique<LoadFileService>(host);
    }
}
