#include "ps2_runtime.h"

#include <cstdio>
#include <exception>
#include <filesystem>

#ifdef PS2X_GAME_HAS_OVERLAY
void installGeneratedOverlay(PS2Runtime *, const uint8_t *);
#endif

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::fprintf(stderr, "usage: %s <guest.elf> <disc.iso>\n", argv[0]);
        return 1;
    }
    try
    {
        if (!std::filesystem::is_regular_file(argv[1]) ||
            !std::filesystem::is_regular_file(argv[2]))
        {
            std::fprintf(stderr, "ELF and ISO must be existing regular files\n");
            return 1;
        }

        PS2Runtime runtime;
        // Resolve before loadELF derives the remaining host filesystem paths.
        auto paths = PS2Runtime::getIoPaths();
        paths.cdImage = std::filesystem::absolute(argv[2]).string();
        paths.cdRoot = std::filesystem::absolute(argv[1]).parent_path().string();
        PS2Runtime::setIoPaths(paths);
        if (!runtime.initialize("PS2Recomp - research runner") || !runtime.loadELF(argv[1]))
            return 2;
#ifdef PS2X_GAME_HAS_OVERLAY
        installGeneratedOverlay(&runtime, runtime.memory().getRDRAM());
#endif
        runtime.run();
        return 0;
    }
    catch (const std::exception &error)
    {
        std::fprintf(stderr, "runner: %s\n", error.what());
        return 3;
    }
}
