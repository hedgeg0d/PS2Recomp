# Katamari Damacy - PS2Recomp work branch

This branch is a working snapshot of PS2Recomp while recompiling the NTSC-U
release of *Katamari Damacy* (`SLUS_210.08`). It is not a finished port yet.

Katamari is proving unusually interesting to reverse-engineer. The game appears
to use a custom engine with a long initialization path, several IOP services,
disc streaming, and executable code loaded into RAM at runtime. That last part
is handled here as a code overlay: the runtime selects compiled functions only
when the loaded RAM contains the expected instruction identity. It does not
patch guest RAM, replace frames, or upload substitute graphics to GS.

The current build reaches the real Memory Card access screen. The next targets
are the memory-card dialog, title screen, and intro sequence.

## What is in this branch

- `docs/katamari/katamari_patched.csv` - current Ghidra function map.
- `docs/katamari/katamari_ghidra.toml` - recompiler configuration template.
- `docs/katamari/ExportPS2FunctionsHeadless.java` - headless Ghidra exporter.
- `ps2xRecomp/tools/package_overlay.py` - packages separately compiled runtime
  overlays without symbol collisions.
- `ps2xRuntime/include/runtime/code_overlays.h` - generic overlay selection API.
- Runtime, IOP, scheduler, CD/SIF, MPEG, pad, and GS progress needed by this
  game, plus control-flow and incremental-generation fixes.

The game-specific runner, ISO/ELF, generated C++, overlay package, dumps, and
logs remain outside Git.

## Build and run

These commands use the local development layout. Keep the existing build cache;
do not use `--clean-first`.

```bash
# Set these three paths first.
export ISO_PATH="/path/to/Katamari Damacy.iso"
export PS2RECOMP="/path/to/PS2Recomp"
export KATAMARI_DIR="/path/to/katamari-work"
export WORK_TMP="/path/to/temporary/katamari-work"

cd "$PS2RECOMP"
cmake --build out/build --target ps2_runtime -j8
cmake --build out/build --target ps2_recomp -j8

cd "$KATAMARI_DIR"
"$PS2RECOMP/out/build/ps2xRecomp/ps2_recomp" \
  "$KATAMARI_DIR/katamari_ghidra.toml"

cd "$KATAMARI_DIR/runner"
cmake -S . -B build -DPS2X_OVERLAY_PACKAGE="$WORK_TMP/stream1-package"
cmake --build build -j8
```

The runner expects the local `SLUS_210.08` and Katamari ISO. Set its local disc
image configuration to `$ISO_PATH`; the sample runner takes the ELF path as its
first argument and uses that disc image for CD reads.

```bash
cd "$KATAMARI_DIR/runner"
timeout --signal=TERM --kill-after=2s 90s \
  env PS2X_CODE_OVERLAY=1 PS2X_MCSERV_VER=210 PS2X_MCMAN_VER=226 \
  ./build/ps2x_katamari "$KATAMARI_DIR/SLUS_210.08"
pkill -x ps2x_katamari 2>/dev/null || true
pgrep -x ps2x_katamari || true
```

## Building the overlay

The overlay is a separate recompiler export of the executable region that the
game loads into RAM later. It is built from a RAM/ELF snapshot captured during
analysis and an identity snapshot of the loaded instructions; it is not read
from or embedded into the main repository. Package it with:

```bash
python3 "$PS2RECOMP/ps2xRecomp/tools/package_overlay.py" \
  --generated "$WORK_TMP/stream1-aot" \
  --identity-source "$WORK_TMP/identity.bin" \
  --begin <overlay-start> --end <overlay-end> \
  --output "$WORK_TMP/stream1-package"
```

Use the executable range exported from the matching snapshot for
`<overlay-start>` and `<overlay-end>`. The runner's CMake file includes the
resulting `overlay_sources.cmake` and links the generated units separately.

Run logs should be kept outside the repository. Do not commit commercial game
data, RAM/ELF snapshots, generated output, or build directories.

## Recompilation plans

This is exploratory work. The generic changes will be refined, split into
focused patches, and proposed as separate PS2Recomp pull requests after the
Katamari path is understood and regression-tested. The current test baseline is
449/449; tests are intentionally not run during the bounded game experiments.

### Describing Your Game Project

A game project built with PS2Recomp can describe itself in a `.recomp.json` file at the root of its repository. Lists of recomp and decomp projects, such as [recomp.board](https://recomp.fyi), read that file instead of guessing the game, system and status from the README.

> [!NOTE]
> This file is optional: PS2Recomp does not read it and works the same without it. `.recomp.json` and recomp.board are third-party projects; the PS2Recomp developers have no ties to them.

Starter file:

```json
{
  "$schema": "https://recomp.fyi/schema/v1.json",
  "game": "<title as it shipped>",
  "system": "PS2",
  "type": "recomp",
  "toolchain": "PS2Recomp",
  "status": "in-progress",
  "original": { "region": "USA", "serial": "SLUS-20312" }
}
```

* `original` is the release a user must own. On retail discs the ELF is named after the serial (`SLUS_203.12` is `SLUS-20312`), and the prefix gives the region: `SLUS`/`SCUS` USA, `SLES`/`SCES` Europe, `SLPS`/`SLPM`/`SCPS` Japan.
* `status` is one of `exploring`, `in-progress`, `playable`, `released`, `complete`, `paused`. Edit it when the project moves on: a stale status is worse than none.
* Never put a game file, or a link to one, in the file.

Other fields (Wikidata item, target platforms, maintainers, links, what help is wanted) and the JSON Schema are in the [specification](https://recomp.fyi/spec).
