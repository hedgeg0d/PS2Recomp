#include "module_factories.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ps2x::iop::detail
{
    namespace
    {
        // These are the public CDVD RPC ports used by libcdvd. They are
        // interface identifiers, not game-specific service addresses.
        constexpr uint32_t kCdServerInit = 0x80000592u;
        constexpr uint32_t kCdServerScmd = 0x80000593u;
        constexpr uint32_t kCdServerNcmd = 0x80000595u;
        constexpr uint32_t kCdServerSearchFile = 0x80000597u;
        constexpr uint32_t kCdServerDiskReady = 0x8000059Au;

        constexpr uint32_t kCdSectorSize = 2048u;
        constexpr uint32_t kVirtualSectorBase = 0x00100000u;
        constexpr uint32_t kMaxPathBytes = 256u;
        constexpr uint32_t kMaxIsoDepth = 16u;

        constexpr uint32_t kScmdReadClock = 1u;
        constexpr uint32_t kScmdGetDiskType = 3u;
        constexpr uint32_t kScmdGetError = 4u;
        constexpr uint32_t kScmdTrayRequest = 5u;
        constexpr uint32_t kScmdStatus = 12u;
        constexpr uint32_t kScmdBreak = 22u;

        constexpr uint32_t kNcmdRead = 1u;
        constexpr uint32_t kNcmdCddaRead = 2u;
        constexpr uint32_t kNcmdDvdRead = 3u;
        constexpr uint32_t kNcmdSeek = 5u;
        constexpr uint32_t kNcmdStop = 7u;
        constexpr uint32_t kNcmdPause = 8u;
        constexpr uint32_t kNcmdStream = 9u;
        constexpr uint32_t kNcmdReadIopMemory = 13u;
        constexpr uint32_t kNcmdDiskReady = 14u;
        constexpr uint32_t kNcmdReadChain = 15u;

        enum class FileSource
        {
            HostFile,
            CdImage,
        };

        struct FileEntry
        {
            FileSource source = FileSource::HostFile;
            uint64_t hostHandle = 0u;
            uint32_t lbn = 0u;
            uint32_t size = 0u;
            uint32_t sectors = 0u;
        };

        uint32_t readLe32(const uint8_t *data)
        {
            return static_cast<uint32_t>(data[0]) |
                   (static_cast<uint32_t>(data[1]) << 8u) |
                   (static_cast<uint32_t>(data[2]) << 16u) |
                   (static_cast<uint32_t>(data[3]) << 24u);
        }

        std::string lowerAscii(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch)
                           { return static_cast<char>(std::tolower(ch)); });
            return value;
        }

        std::string normalizePath(std::string value)
        {
            std::replace(value.begin(), value.end(), '\\', '/');
            const std::string lower = lowerAscii(value);
            if (lower.rfind("cdrom0:", 0u) == 0u)
            {
                value.erase(0u, 7u);
            }
            else if (lower.rfind("cdrom:", 0u) == 0u)
            {
                value.erase(0u, 6u);
            }

            while (!value.empty() && value.front() == '/')
            {
                value.erase(value.begin());
            }
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())))
            {
                value.erase(value.begin());
            }
            while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))
            {
                value.pop_back();
            }

            const std::size_t version = value.find(';');
            if (version != std::string::npos)
            {
                bool numeric = version + 1u < value.size();
                for (std::size_t i = version + 1u; i < value.size(); ++i)
                {
                    numeric = numeric && std::isdigit(static_cast<unsigned char>(value[i]));
                }
                if (numeric)
                {
                    value.erase(version);
                }
            }
            return value;
        }

        bool pathComponentEqual(std::string_view lhs, std::string_view rhs)
        {
            return lowerAscii(std::string(lhs)) == lowerAscii(std::string(rhs));
        }

        uint32_t sectorsForBytes(uint64_t bytes)
        {
            const uint64_t sectors = (bytes + kCdSectorSize - 1u) / kCdSectorSize;
            return static_cast<uint32_t>(std::min<uint64_t>(
                std::max<uint64_t>(1u, sectors), std::numeric_limits<uint32_t>::max()));
        }

        class CdvdService final : public IopService
        {
        public:
            explicit CdvdService(IopHost &host)
                : m_host(host),
                  m_sids{kCdServerInit,
                         kCdServerScmd,
                         kCdServerNcmd,
                         kCdServerSearchFile,
                         kCdServerDiskReady}
            {
            }

            ~CdvdService() override
            {
                reset();
            }

            [[nodiscard]] std::string_view name() const override
            {
                return "cdvd";
            }

            [[nodiscard]] std::span<const uint32_t> sids() const override
            {
                return m_sids;
            }

            void reset() override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                for (const auto &[key, entry] : m_files)
                {
                    (void)key;
                    if (entry.source == FileSource::HostFile && entry.hostHandle != 0u)
                    {
                        m_host.closeHostFile(entry.hostHandle);
                    }
                }
                m_files.clear();
                m_nextVirtualLbn = kVirtualSectorBase;
                m_lastError = 0;
                m_initialized = false;
            }

            [[nodiscard]] RpcResult handleRpc(const RpcRequest &request) override
            {
                RpcResult result;
                switch (request.sid)
                {
                case kCdServerInit:
                    return handleInit(request, result);
                case kCdServerScmd:
                    return handleScmd(request, result);
                case kCdServerNcmd:
                    return handleNcmd(request, result);
                case kCdServerSearchFile:
                    return handleSearchFile(request, result);
                case kCdServerDiskReady:
                    return handleDiskReady(request, result);
                default:
                    return result;
                }
            }

            void appendDebugMetrics(std::vector<DebugMetric> &metrics) const override
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                metrics.push_back({"registered_files", m_files.size(), false});
                metrics.push_back({"initialized", m_initialized ? 1u : 0u, false});
                metrics.push_back({"last_error", static_cast<uint64_t>(static_cast<int64_t>(m_lastError)), false});
            }

        private:
            [[nodiscard]] RpcResult handled(RpcResult result, const RpcRequest &request) const
            {
                result.handled = true;
                result.resultAddress = request.receive.address;
                return result;
            }

            [[nodiscard]] RpcResult handleInit(const RpcRequest &request, RpcResult result)
            {
                result = handled(result, request);
                std::array<uint32_t, 4u> response{1u, 0u, 0u, 0u};
                if (request.receive.address != 0u && request.receive.size != 0u)
                {
                    (void)m_host.writeGuest(request.receive.address,
                                            response.data(),
                                            std::min<size_t>(request.receive.size, sizeof(response)));
                }
                std::lock_guard<std::mutex> lock(m_mutex);
                m_initialized = true;
                m_lastError = 0;
                return result;
            }

            [[nodiscard]] RpcResult handleDiskReady(const RpcRequest &request, RpcResult result)
            {
                result = handled(result, request);
                writeResult(request, 2u);
                return result;
            }

            [[nodiscard]] RpcResult handleScmd(const RpcRequest &request, RpcResult result)
            {
                result = handled(result, request);
                uint32_t value = 1u;
                switch (request.function)
                {
                case kScmdReadClock:
                    value = 1u;
                    if (request.receive.address != 0u && request.receive.size >= 12u)
                    {
                        std::array<uint8_t, 12u> clock{};
                        std::memcpy(clock.data(), &value, sizeof(value));
                        (void)m_host.writeGuest(request.receive.address, clock.data(), clock.size());
                    }
                    return result;
                case kScmdGetDiskType:
                    value = 0x14u; // SCECdPS2DVD
                    break;
                case kScmdGetError:
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        value = static_cast<uint32_t>(m_lastError);
                    }
                    break;
                case kScmdTrayRequest:
                case kScmdBreak:
                    value = 1u;
                    break;
                case kScmdStatus:
                    value = m_initialized ? 6u : 0u;
                    break;
                default:
                    value = 0u;
                    break;
                }
                writeResult(request, value);
                return result;
            }

            [[nodiscard]] RpcResult handleSearchFile(const RpcRequest &request, RpcResult result)
            {
                result = handled(result, request);
                const std::string path = normalizePath(readGuestString(request.send.address + 32u,
                                                                        std::min<uint32_t>(kMaxPathBytes,
                                                                                           request.send.size > 32u
                                                                                               ? request.send.size - 32u
                                                                                               : 0u)));
                uint32_t destination = 0u;
                (void)readGuestU32(request.send.address + 288u, destination);

                FileEntry entry{};
                const bool found = !path.empty() && resolveFile(path, entry);
                if (found)
                {
                    std::array<uint8_t, 32u> file{};
                    std::memcpy(file.data() + 0u, &entry.lbn, sizeof(entry.lbn));
                    std::memcpy(file.data() + 4u, &entry.size, sizeof(entry.size));
                    const std::size_t slash = path.find_last_of('/');
                    const std::string leaf = path.substr(slash == std::string::npos ? 0u : slash + 1u);
                    std::copy_n(leaf.begin(), std::min<size_t>(leaf.size(), 15u),
                                reinterpret_cast<char *>(file.data() + 8u));
                    if (destination != 0u)
                    {
                        (void)m_host.writeGuest(destination, file.data(), file.size());
                    }
                }
                writeResult(request, found ? 1u : 0u);
                return result;
            }

            [[nodiscard]] RpcResult handleNcmd(const RpcRequest &request, RpcResult result)
            {
                result = handled(result, request);
                switch (request.function)
                {
                case kNcmdRead:
                case kNcmdCddaRead:
                case kNcmdDvdRead:
                case kNcmdReadIopMemory:
                    handleRead(request);
                    return result;
                case kNcmdDiskReady:
                    writeResult(request, 2u);
                    return result;
                case kNcmdSeek:
                case kNcmdStop:
                case kNcmdPause:
                case kNcmdStream:
                case kNcmdReadChain:
                    writeResult(request, 1u);
                    return result;
                default:
                    writeResult(request, 0u);
                    return result;
                }
            }

            void handleRead(const RpcRequest &request)
            {
                uint32_t lbn = 0u;
                uint32_t sectors = 0u;
                uint32_t destination = 0u;
                uint32_t mode = 0u;
                uint32_t positionAddress = 0u;
                if (!readGuestU32(request.send.address + 0u, lbn) ||
                    !readGuestU32(request.send.address + 4u, sectors) ||
                    !readGuestU32(request.send.address + 8u, destination) ||
                    !readGuestU32(request.send.address + 12u, mode) ||
                    !readGuestU32(request.send.address + 20u, positionAddress) ||
                    destination == 0u || sectors == 0u)
                {
                    setLastError(-1);
                    return;
                }

                const uint32_t sectorBytes = ((mode >> 16u) == 2u) ? 2328u
                                             : ((mode >> 16u) == 3u) ? 2340u
                                                                     : kCdSectorSize;
                const uint64_t byteCount = static_cast<uint64_t>(sectors) * sectorBytes;
                if (byteCount > std::numeric_limits<size_t>::max())
                {
                    setLastError(-1);
                    return;
                }

                std::vector<uint8_t> output(static_cast<size_t>(byteCount), 0u);
                for (uint32_t sector = 0u; sector < sectors; ++sector)
                {
                    std::array<uint8_t, kCdSectorSize> payload{};
                    if (!readSector(lbn + sector, payload.data()))
                    {
                        setLastError(-1);
                        return;
                    }
                    std::memcpy(output.data() + static_cast<size_t>(sector) * sectorBytes,
                                payload.data(),
                                std::min<uint32_t>(kCdSectorSize, sectorBytes));
                }

                if (!m_host.writeGuest(destination, output.data(), output.size()))
                {
                    setLastError(-1);
                    return;
                }
                if (positionAddress != 0u)
                {
                    const uint32_t position = lbn + sectors;
                    (void)m_host.writeGuest(positionAddress, &position, sizeof(position));
                }
                setLastError(0);
            }

            [[nodiscard]] std::string readGuestString(uint32_t address, uint32_t maxBytes) const
            {
                if (address == 0u || maxBytes == 0u)
                {
                    return {};
                }
                std::vector<char> bytes(maxBytes, '\0');
                if (!m_host.readGuest(address, bytes.data(), bytes.size()))
                {
                    return {};
                }
                const auto end = std::find(bytes.begin(), bytes.end(), '\0');
                return std::string(bytes.begin(), end);
            }

            [[nodiscard]] bool readGuestU32(uint32_t address, uint32_t &value) const
            {
                value = 0u;
                return m_host.readGuest(address, &value, sizeof(value));
            }

            void writeResult(const RpcRequest &request, uint32_t value) const
            {
                if (request.receive.address != 0u && request.receive.size >= sizeof(value))
                {
                    (void)m_host.writeGuest(request.receive.address, &value, sizeof(value));
                }
            }

            [[nodiscard]] std::string rootPath() const
            {
                return m_host.hostPath(HostPathKind::CdRoot);
            }

            [[nodiscard]] std::string imagePath() const
            {
                return m_host.hostPath(HostPathKind::CdImage);
            }

            [[nodiscard]] bool openHostPath(const std::string &path, FileEntry &entry)
            {
                const uint64_t handle = m_host.openHostFile(path);
                if (handle == 0u)
                {
                    return false;
                }
                uint64_t size = 0u;
                if (!m_host.hostFileSize(handle, size))
                {
                    m_host.closeHostFile(handle);
                    return false;
                }

                std::lock_guard<std::mutex> lock(m_mutex);
                entry.source = FileSource::HostFile;
                entry.hostHandle = handle;
                entry.lbn = m_nextVirtualLbn;
                entry.size = static_cast<uint32_t>(std::min<uint64_t>(size, 0xFFFFFFFFull));
                entry.sectors = sectorsForBytes(size);
                m_nextVirtualLbn += entry.sectors + 1u;
                return true;
            }

            [[nodiscard]] bool resolveFile(const std::string &path, FileEntry &entry)
            {
                const std::string root = rootPath();
                if (!root.empty())
                {
                    const std::filesystem::path rootFs(root);
                    const std::filesystem::path relative(path);
                    if (relative.is_absolute() || std::any_of(relative.begin(), relative.end(), [](const auto &component)
                                                              { return component == ".."; }))
                    {
                        setLastError(-1);
                        return false;
                    }
                    const std::filesystem::path candidate = (rootFs / relative).lexically_normal();
                    if (candidate == rootFs)
                    {
                        return false;
                    }
                    if (openHostPath(candidate.string(), entry))
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        m_files[lowerAscii(path)] = entry;
                        return true;
                    }

                    std::filesystem::path current = rootFs;
                    bool found = true;
                    for (const auto &component : relative)
                    {
                        std::error_code ec;
                        if (std::filesystem::exists(current / component, ec) && !ec)
                        {
                            current /= component;
                            continue;
                        }
                        found = false;
                        std::error_code iterEc;
                        for (const auto &child : std::filesystem::directory_iterator(current, iterEc))
                        {
                            if (!iterEc && pathComponentEqual(child.path().filename().string(), component.string()))
                            {
                                current = child.path();
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            break;
                        }
                    }
                    if (found && openHostPath(current.string(), entry))
                    {
                        std::lock_guard<std::mutex> lock(m_mutex);
                        m_files[lowerAscii(path)] = entry;
                        return true;
                    }
                }

                if (resolveIsoFile(path, entry))
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_files[lowerAscii(path)] = entry;
                    return true;
                }
                setLastError(-1);
                return false;
            }

            [[nodiscard]] bool resolveIsoFile(const std::string &path, FileEntry &entry)
            {
                const std::string image = imagePath();
                if (image.empty())
                {
                    return false;
                }
                const uint64_t handle = m_host.openHostFile(image);
                if (handle == 0u)
                {
                    return false;
                }

                std::array<uint8_t, kCdSectorSize> pvd{};
                size_t bytesRead = 0u;
                const bool pvdOk = m_host.readHostFile(handle,
                                                       16ull * kCdSectorSize,
                                                       pvd.data(),
                                                       pvd.size(),
                                                       bytesRead) &&
                                   bytesRead == pvd.size() &&
                                   pvd[0] == 1u &&
                                   std::memcmp(pvd.data() + 1u, "CD001", 5u) == 0;
                if (!pvdOk)
                {
                    m_host.closeHostFile(handle);
                    return false;
                }

                const uint8_t *rootRecord = pvd.data() + 156u;
                if (rootRecord[0] < 34u)
                {
                    m_host.closeHostFile(handle);
                    return false;
                }
                const uint32_t rootLbn = readLe32(rootRecord + 2u);
                const uint32_t rootSize = readLe32(rootRecord + 10u);

                std::vector<std::string> components;
                std::string pending;
                for (const char ch : path)
                {
                    if (ch == '/')
                    {
                        if (!pending.empty())
                        {
                            components.push_back(pending);
                            pending.clear();
                        }
                    }
                    else
                    {
                        pending.push_back(ch);
                    }
                }
                if (!pending.empty())
                {
                    components.push_back(pending);
                }

                uint32_t currentLbn = rootLbn;
                uint32_t currentSize = rootSize;
                bool found = false;
                for (size_t componentIndex = 0u; componentIndex < components.size(); ++componentIndex)
                {
                    found = false;
                    const uint32_t sectorCount = sectorsForBytes(currentSize);
                    for (uint32_t sector = 0u; sector < sectorCount && !found; ++sector)
                    {
                        std::array<uint8_t, kCdSectorSize> directory{};
                        bytesRead = 0u;
                        if (!m_host.readHostFile(handle,
                                                 static_cast<uint64_t>(currentLbn + sector) * kCdSectorSize,
                                                 directory.data(),
                                                 directory.size(),
                                                 bytesRead) ||
                            bytesRead != directory.size())
                        {
                            break;
                        }
                        for (uint32_t offset = 0u; offset + 33u <= kCdSectorSize;)
                        {
                            const uint8_t recordSize = directory[offset];
                            if (recordSize == 0u)
                            {
                                break;
                            }
                            if (recordSize < 34u || offset + recordSize > kCdSectorSize)
                            {
                                break;
                            }

                            const uint8_t flags = directory[offset + 25u];
                            const uint8_t nameSize = directory[offset + 32u];
                            if (33u + nameSize <= recordSize && nameSize > 0u &&
                                directory[offset + 33u] != 0u && directory[offset + 33u] != 1u)
                            {
                                std::string name(reinterpret_cast<const char *>(directory.data() + offset + 33u),
                                                 nameSize);
                                const std::string normalizedName = normalizePath(name);
                                if (pathComponentEqual(normalizedName, components[componentIndex]))
                                {
                                    currentLbn = readLe32(directory.data() + offset + 2u);
                                    currentSize = readLe32(directory.data() + offset + 10u);
                                    found = true;
                                    if (componentIndex + 1u == components.size())
                                    {
                                        entry.source = FileSource::CdImage;
                                        entry.hostHandle = handle;
                                        entry.lbn = currentLbn;
                                        entry.size = currentSize;
                                        entry.sectors = sectorsForBytes(currentSize);
                                    }
                                    else if ((flags & 2u) == 0u)
                                    {
                                        found = false;
                                    }
                                    break;
                                }
                            }
                            offset += recordSize;
                        }
                    }
                    if (!found)
                    {
                        m_host.closeHostFile(handle);
                        return false;
                    }
                }

                if (found && !components.empty() && entry.source == FileSource::CdImage)
                {
                    return true;
                }
                m_host.closeHostFile(handle);
                return false;
            }

            [[nodiscard]] bool readSector(uint32_t lbn, uint8_t *destination)
            {
                FileEntry entry{};
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    for (const auto &[key, candidate] : m_files)
                    {
                        (void)key;
                        if (lbn >= candidate.lbn && lbn < candidate.lbn + candidate.sectors)
                        {
                            entry = candidate;
                            break;
                        }
                    }
                }

                if (entry.source == FileSource::HostFile && entry.hostHandle != 0u)
                {
                    const uint64_t offset = static_cast<uint64_t>(lbn - entry.lbn) * kCdSectorSize;
                    std::fill_n(destination, kCdSectorSize, uint8_t{0});
                    if (offset >= entry.size)
                    {
                        return true;
                    }
                    const size_t bytesToRead = static_cast<size_t>(std::min<uint64_t>(
                        kCdSectorSize, static_cast<uint64_t>(entry.size) - offset));
                    size_t bytesRead = 0u;
                    return m_host.readHostFile(entry.hostHandle,
                                               offset,
                                               destination,
                                               bytesToRead,
                                               bytesRead) &&
                           bytesRead == bytesToRead;
                }
                if (entry.source == FileSource::CdImage && entry.hostHandle != 0u)
                {
                    size_t bytesRead = 0u;
                    return m_host.readHostFile(entry.hostHandle,
                                               static_cast<uint64_t>(lbn) * kCdSectorSize,
                                               destination,
                                               kCdSectorSize,
                                               bytesRead) &&
                           bytesRead == kCdSectorSize;
                }

                const std::string image = imagePath();
                if (!image.empty())
                {
                    const uint64_t handle = m_host.openHostFile(image);
                    if (handle != 0u)
                    {
                        size_t bytesRead = 0u;
                        const bool ok = m_host.readHostFile(handle,
                                                            static_cast<uint64_t>(lbn) * kCdSectorSize,
                                                            destination,
                                                            kCdSectorSize,
                                                            bytesRead) &&
                                        bytesRead == kCdSectorSize;
                        m_host.closeHostFile(handle);
                        return ok;
                    }
                }
                return false;
            }

            void setLastError(int32_t error)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                m_lastError = error;
            }

            IopHost &m_host;
            const std::array<uint32_t, 5u> m_sids;
            mutable std::mutex m_mutex;
            std::unordered_map<std::string, FileEntry> m_files;
            uint32_t m_nextVirtualLbn = kVirtualSectorBase;
            int32_t m_lastError = 0;
            bool m_initialized = false;
        };
    }

    std::unique_ptr<IopService> createCdvdService(IopHost &host)
    {
        return std::make_unique<CdvdService>(host);
    }
}
