# Running the Katamari Damacy recompilation

The game-specific runner and disc files live outside this repository. Set your
paths first, then run the commands below:

```bash
export ISO_PATH="/path/to/Katamari Damacy.iso"
export PS2RECOMP="/path/to/PS2Recomp"
export KATAMARI_DIR="/path/to/katamari-work"
export WORK_TMP="/path/to/temporary/katamari-work"

cd "$PS2RECOMP"
cmake --build out/build --target ps2_runtime -j8
cmake --build out/build --target ps2_recomp -j8

cd "$KATAMARI_DIR"
$PS2RECOMP/out/build/ps2xRecomp/ps2_recomp \
  $KATAMARI_DIR/katamari_ghidra.toml

cd "$KATAMARI_DIR/runner"
cmake -S . -B build -DPS2X_OVERLAY_PACKAGE="$WORK_TMP/stream1-package"
cmake --build build -j8
```

The runner currently expects the local `SLUS_210.08`, Katamari ISO, and the
optional generated overlay package at `$WORK_TMP/stream1-package`. Start
with the overlay enabled:

```bash
cd $KATAMARI_DIR/runner
timeout --signal=TERM --kill-after=2s 90s \
  env PS2X_CODE_OVERLAY=1 PS2X_MCSERV_VER=210 PS2X_MCMAN_VER=226 \
  ./build/ps2x_katamari $KATAMARI_DIR/SLUS_210.08
pkill -x ps2x_katamari 2>/dev/null || true
pgrep -x ps2x_katamari || true
```

Do not commit the ISO, ELF/RAM snapshots, generated output, overlay package,
runner build directory, or run logs.
