# Katamari Damacy — PS2Recomp research branch

Work on recompiling the NTSC-U release, `SLUS_210.08`. Not a finished port,
not playable yet, and not an upstream-ready patch set.

The game has a long custom-engine initialization path, IOP services, disc
streaming, and executable code loaded into RAM at runtime. The overlay support
selects separately compiled functions by matching loaded instruction words.
It does not inject replacement frames or graphics.

## Current progress

The validated development build reaches the memory-card dialog, Namco logo,
and several real intro frames with cows and stars. Full intro playback still
stops. These frames use guest MPEG code and software IPU decoding, then the GS;
they are not supplied by the FFmpeg HLE path.

The milestone currently uses opt-in **game-generated-code test hooks** to
choose/confirm the memory-card answer. Those hooks and the commercial-game
overlay export are not included here. A fresh checkout is therefore a research
toolchain snapshot, **not a one-command reproduction of the intro milestone**.

## Build and run

Start by setting the path to your own ISO. Follow [the setup guide](docs/katamari/RUNNING.md)
for dependencies, ELF extraction, CMake configuration, recompilation, overlay
requirements, and a bounded launch. No machine-specific paths or native-CPU
tuning are needed. A [generic runner](examples/recompiled-game/main.cpp) accepts
the ELF and ISO as command-line arguments.

[Status and validation checklist](docs/katamari/STATUS.md) records what works,
what remains blocked, and what has actually been checked.

## Contents and scope

- Current function map, TOML configuration, and Ghidra exporter in `docs/katamari`.
- Generic overlay packaging and identity-based runtime dispatch.
- Recompiler control-flow and incremental-generation fixes.
- Runtime/IOP work covering SIF, CD streaming, scheduling, memory cards, pads,
  GS transfers, and software IPU MPEG decoding.
- A path-configurable example runner using the runtime's CMake dependencies.

This branch still contains exploratory diagnostics and compatibility work.
The TOML also contains game-specific instruction patches; these are not
universal runtime fixes. Generic changes will be refined, regression-tested,
and split into focused upstream PRs.

ISOs, extracted game files, generated game C++, overlay snapshots, run logs,
private worklogs, and recovery archives stay outside Git. PCSX2-derived MPEG
code retains its notices; see [third-party attribution](docs/THIRD_PARTY.md).

The optional third-party `.recomp.json` project metadata format is described
in the [upstream project specification](https://recomp.fyi/spec). It is not
required by PS2Recomp.
