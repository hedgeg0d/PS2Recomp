#pragma once

#include <cstdint>
#include <vector>

class PS2Runtime;
struct R5900Context;

namespace ps2x
{
    using OverlayFunction = void (*)(uint8_t *, R5900Context *, PS2Runtime *);

    struct OverlayWord
    {
        uint32_t address;
        uint32_t value;
    };

    struct OverlayEntry
    {
        uint32_t address;
        OverlayFunction function;
    };

    struct CodeOverlay
    {
        uint32_t begin;
        uint32_t end; // Exclusive; includes unexported code within the module.
        // Immutable instruction words which identify this loaded variant.
        std::vector<OverlayWord> identity;
        std::vector<OverlayEntry> entries;
    };

    // Configure before guest execution. Definitions are copied; RAM is only
    // read and must remain valid until clearCodeOverlays or VM destruction.
    void setCodeOverlays(const PS2Runtime *runtime, const uint8_t *ram,
                         std::vector<CodeOverlay> overlays);
    void clearCodeOverlays(const PS2Runtime *runtime);

    // True means a matching loaded overlay owns this PC. A null function then
    // means a missing export, and must not fall back to stale boot code.
    bool resolveCodeOverlay(const PS2Runtime *runtime, uint32_t pc,
                            OverlayFunction &function);
}
