# Building the Katamari research snapshot

## 1. Set the ISO and working directories

Use a legally obtained NTSC-U Katamari Damacy ISO (`SLUS_210.08`). Other
releases are not validated with this function map. Keep game files and generated
output outside the checkout.

```bash
export ISO_PATH="/path/to/Katamari Damacy.iso"
export PS2RECOMP="/path/to/PS2Recomp"
export GAME_WORK="/path/to/katamari-work"

test -f "$ISO_PATH"
mkdir -p "$GAME_WORK"
export ISO_PATH="$(realpath "$ISO_PATH")"
export PS2RECOMP="$(realpath "$PS2RECOMP")"
export GAME_WORK="$(realpath "$GAME_WORK")"
```

The commands below target a Linux desktop with a working display/audio setup.
Other host targets supported by upstream are not validated for this milestone.
The x86 generated helpers require SSE4.1; there is no `-march=native` tuning.

## 2. Install build dependencies

Required: Git, CMake 3.21+, a C++20 compiler, Python 3, pkg-config, 7-Zip for
ELF extraction, and the platform development libraries used by raylib.
FFmpeg development libraries enable the optional generic MPEG HLE path.
CMake fetches the remaining source dependencies on the first configuration.

For Debian/Ubuntu, the typical packages are:

```bash
sudo apt-get install build-essential cmake git python3 pkg-config p7zip-full \
  libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev \
  libxcursor-dev libxi-dev libasound2-dev \
  libavcodec-dev libavformat-dev libavutil-dev libswresample-dev libswscale-dev
```

Optional: install `ccache` or `sccache`. To select ccache explicitly, pass
`-DPS2X_ENABLE_SCCACHE=OFF -DCMAKE_C_COMPILER_LAUNCHER=ccache
-DCMAKE_CXX_COMPILER_LAUNCHER=ccache` to each configuration below. Cache
reuse does not avoid rebuilding translation units whose runtime headers changed.

## 3. Extract only the boot ELF and copy the map/configuration

```bash
7z e -y "-o$GAME_WORK" "$ISO_PATH" SLUS_210.08
cp "$PS2RECOMP/docs/katamari/katamari_patched.csv" "$GAME_WORK/"
cp "$PS2RECOMP/docs/katamari/katamari_ghidra.toml" "$GAME_WORK/"
test -f "$GAME_WORK/SLUS_210.08"
```

The TOML uses paths relative to the working directory. It includes experimental
game-specific instruction patches. Do not confuse these with generic fixes.
It also sets `discover_functions = false`: skip heuristic discovery before
loading the explicit CSV. The default for other configurations remains true.
Missing/invalid maps are errors in this mode. Direct branch-target recovery
after loading the map and generated continuation entries still apply; missing
indirect entries may need to be added to the CSV.

## 4. Configure and build the toolchain

```bash
cmake -S "$PS2RECOMP" -B "$PS2RECOMP/out/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DPS2X_BUILD_STUDIO=OFF -DPS2X_BUILD_ANALYZER=OFF \
  -DPS2X_BUILD_TEST=OFF -DPS2X_IOP_BUILD_TESTS=OFF \
  -DPS2X_ENABLE_LTO=OFF -DPS2X_ENABLE_FFMPEG=ON \
  -DPS2X_ENABLE_DEBUG_UI=OFF \
  -DPS2X_ENABLE_RUNTIME_LOGS=OFF -DPS2X_ENABLE_AGRESSIVE_LOGS=OFF
cmake --build "$PS2RECOMP/out/build" --target ps2_recomp ps2_runtime -j8
```

Keep this build directory and compiler cache for later incremental builds.
Do not use `--clean-first`. Tests are disabled in this recipe, not certified
as passing after the upstream merge.

## 5. Generate and build the base game

```bash
(
  cd "$GAME_WORK"
  "$PS2RECOMP/out/build/ps2xRecomp/ps2_recomp" katamari_ghidra.toml
)
cmake -S "$PS2RECOMP" -B "$PS2RECOMP/out/build" \
  -DPS2X_GAME_GENERATED_DIR="$GAME_WORK/output"
cmake --build "$PS2RECOMP/out/build" --target ps2xGameRunner -j8
```

The runner compiles only the generated manifest, not stale files left by older
exports. Continuation functions are placed in trailing unity groups to reduce
cache churn. No executable overlay is included by this step.

## 6. Where the overlay comes from

The retail game loads and decodes executable data during boot. That code is
absent from the main ELF's static function map. Development captured the loaded
region in a guest RAM/ELF snapshot, exported its functions separately, then
packaged those generated functions with instruction identity checks.

**Automatic ISO-to-overlay extraction is not implemented here.** You must
capture/export the matching loaded region yourself. Neither commercial game
instructions nor snapshots/generated game C++ are distributed in this repo.
The base build above works without this export but cannot reproduce the
reported overlay-dependent milestone.

If you already have a per-function overlay export, package it as follows:

```bash
export OVERLAY_EXPORT="/path/to/generated-overlay"
export IDENTITY_SOURCE="/path/to/instruction-identity.cpp"
export OVERLAY_BEGIN="0x1038140"
export OVERLAY_END="0x1264934"

python3 "$PS2RECOMP/ps2xRecomp/tools/package_overlay.py" \
  --generated "$OVERLAY_EXPORT" --identity-source "$IDENTITY_SOURCE" \
  --begin "$OVERLAY_BEGIN" --end "$OVERLAY_END" \
  --output "$GAME_WORK/overlay-package"

cmake -S "$PS2RECOMP" -B "$PS2RECOMP/out/build" \
  -DPS2X_GAME_OVERLAY_PACKAGE="$GAME_WORK/overlay-package"
cmake --build "$PS2RECOMP/out/build" --target ps2xGameRunner -j8
```

These bounds describe the analyzed NTSC-U overlay, not a universal address.
The identity input is **text**, with generated disassembly comments such as
`// 0x1038140: 0x27bdfff0  ...`, not a raw binary dump. Use immutable matching
instruction samples from your own export; the packaging tool does not discover
functions. Packages copy selected sources and use relative includes, so they
can be relocated. Do not publish the generated package.

The example runner installs a linked overlay automatically. Missing configured
overlay sources are a configuration error; it never silently reuses an old
binary archive.

## 7. Bounded launch

```bash
set +e
timeout --signal=TERM --kill-after=5s 100s \
  env PS2X_MCSERV_VER=210 PS2X_MCMAN_VER=226 \
  "$PS2RECOMP/out/build/examples/recompiled-game/ps2xGameRunner" \
  "$GAME_WORK/SLUS_210.08" "$ISO_PATH" \
  > "$GAME_WORK/run.log" 2>&1
run_status=$?
set -e
printf 'Runner exit status: %s\n' "$run_status"
pgrep -x ps2xGameRunner || true
```

The version overrides are hexadecimal, without `0x`. They affect HLE
memory-card module versions, not ISO data. GNU timeout status 124 is expected
when the 100-second budget expires; 137 can mean forced termination. Other
nonzero statuses need investigation. Timeout waits for the child to terminate;
check that no process remains.

This public command does **not** enable the private generated-code hooks.
The validated intro experiment additionally used
`PS2X_FORCE_MC_YES=1 PS2X_FORCE_MC_CONFIRM=1`. These variables do nothing in
freshly generated code unless the matching test edits are applied. They
simulate the memory-card decision only; they are not a normal-input fix.
No claim is made that this command alone plays the intro.

Check visible output as well as logs. The card screen may take tens of seconds
to appear. A zero nonblack counter from one frame is not proof that every frame
is black. See [STATUS.md](STATUS.md) for the validation checklist.
