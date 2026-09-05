#include "runtime/code_overlays.h"

#include <algorithm>
#include <cstring>
#include <map>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace ps2x
{
    namespace
    {
        struct RuntimeOverlays
        {
            const uint8_t *ram;
            std::vector<CodeOverlay> overlays;
        };

        struct Registry
        {
            std::mutex mutex;
            std::map<const PS2Runtime *, RuntimeOverlays> runtimes;
        };

        Registry &registry()
        {
            // Safe for global VM instances during static destruction. Each VM
            // removes its own entries; only this registry has process lifetime.
            static auto *instance = new Registry;
            return *instance;
        }
    }

    void setCodeOverlays(const PS2Runtime *runtime, const uint8_t *ram,
                         std::vector<CodeOverlay> overlays)
    {
        if (!runtime || !ram)
            throw std::invalid_argument("Code overlays require a VM and RAM");
        for (auto &overlay : overlays)
        {
            if (overlay.begin >= overlay.end || overlay.end > 0x02000000u ||
                (overlay.begin & 3u) || (overlay.end & 3u) || overlay.identity.empty())
                throw std::invalid_argument("Invalid code overlay range or identity");
            for (const auto &word : overlay.identity)
            {
                if ((word.address & 3u) || word.address < overlay.begin ||
                    word.address > overlay.end - 4u)
                    throw std::invalid_argument("Overlay identity outside its code range");
            }
            std::sort(overlay.entries.begin(), overlay.entries.end(),
                      [](const auto &a, const auto &b) { return a.address < b.address; });
            for (size_t i = 0; i < overlay.entries.size(); ++i)
            {
                const auto &entry = overlay.entries[i];
                if (!entry.function || (entry.address & 3u) || entry.address < overlay.begin ||
                    entry.address >= overlay.end ||
                    (i && overlay.entries[i - 1].address == entry.address))
                    throw std::invalid_argument("Invalid or duplicate overlay entry");
            }
        }
        auto &state = registry();
        std::lock_guard<std::mutex> lock(state.mutex);
        state.runtimes.insert_or_assign(runtime, RuntimeOverlays{ram, std::move(overlays)});
    }

    void clearCodeOverlays(const PS2Runtime *runtime)
    {
        auto &state = registry();
        std::lock_guard<std::mutex> lock(state.mutex);
        state.runtimes.erase(runtime);
    }

    bool resolveCodeOverlay(const PS2Runtime *runtime, uint32_t pc, OverlayFunction &function)
    {
        function = nullptr;
        auto &state = registry();
        std::lock_guard<std::mutex> lock(state.mutex);
        const auto found = state.runtimes.find(runtime);
        if (found == state.runtimes.end())
            return false;
        bool matched = false;
        for (const auto &overlay : found->second.overlays)
        {
            if (pc < overlay.begin || pc >= overlay.end)
                continue;
            const bool sameCode = std::all_of(overlay.identity.begin(), overlay.identity.end(),
                [&](const OverlayWord &word)
                {
                    const auto *p = found->second.ram + word.address;
                    const uint32_t actual = uint32_t(p[0]) | (uint32_t(p[1]) << 8) |
                                            (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
                    return actual == word.value;
                });
            if (!sameCode)
                continue;
            if (matched)
                throw std::runtime_error("Ambiguous loaded code overlay identity");
            matched = true;
            const auto entry = std::lower_bound(overlay.entries.begin(), overlay.entries.end(), pc,
                [](const OverlayEntry &candidate, uint32_t address) { return candidate.address < address; });
            if (entry != overlay.entries.end() && entry->address == pc)
                function = entry->function;
        }
        return matched;
    }
}
