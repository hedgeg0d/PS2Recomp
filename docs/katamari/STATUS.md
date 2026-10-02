# Recompilation status

Last validated milestone: 2026-10-02. Target: NTSC-U `SLUS_210.08`.

| Stage | Observed result |
| --- | --- |
| Boot, streamed code and overlay dispatch | Reached |
| Memory-card access screen and Yes/No dialog | Rendered |
| Namco logo | Rendered |
| Intro video | Several cow/star frames rendered; playback incomplete |
| Title/tutorial/gameplay | Not validated |

The intro frames come from guest MPEG code using the software IPU decoder,
then normal GS transfers/presentation. FFmpeg HLE is available in the runtime
but is not the observed path for these frames.

## How the milestone was checked

- Upstream main through `c5a9d02` integrated; runtime/recompiler rebuilt.
- Bounded 100-second development run; process terminated and reaped.
- Memory-card/Namco milestones and visible intro frames checked.
- 28 coherent IPU CSC frames; no MPEG decode errors in that run.
- Decoded frame 10 matched the pre-merge output byte-for-byte:
  SHA-256 `ec8841cd8d36722a28e94100744c92780f943279bf9a6522a9f9a38436193808`.
- Historical test baseline was 449/449 before the newer upstream integration.
  Tests were not rerun for this milestone. This is not a current suite result.

## Known limitations

The experiment uses a separately exported commercial-game overlay and opt-in
generated-code memory-card answer/confirmation edits. These are outside Git.
Ordinary pad-driven confirmation is not yet validated. A clean base export
does not reproduce the assisted intro milestone without those prerequisites.

Two dynamic MPEG callbacks appear to enter after leading NOP padding, while
the CSV registers the preceding start. Entry metadata/dispatch needs further
investigation. The intro stalls later; several rendered frames are not proof
of full playback or correct timing.

Fresh regeneration still reports unsupported instructions in some mapped
regions. Successful generation and C++ syntax checks do not certify every
function boundary or instruction translation.

The branch contains exploratory probes and compatibility code, plus explicit
game-specific TOML instruction patches. It is public research code, not a
claim that every diff is a universal production fix. Upstream PRs will need
smaller patches, diagnostic cleanup, and fresh regression coverage.

## Publication check — 2026-10-02

- Tracked text scanned for personal filesystem paths and common credential
  patterns; no matches. Private notes/editor state moved outside the checkout.
- Private backup note removed from unpublished branch history only. Existing
  remote/upstream history preserved; normal fast-forward publication remains possible.
- Runtime/recompiler built incrementally with eight jobs. Generic runner built
  from a fresh CMake cache against the existing milestone export.
- Fresh CSV/TOML regeneration succeeded in explicit-map mode. All regenerated
  unity units passed C++ syntax checks; this is not a full game-play validation
  of that new export and is not a test-suite run.
- Relocated overlay package passed a C++ compile check without absolute includes.
- Generic runner repeated the 100-second milestone gate: memory-card dialog,
  cow/star frames, 28 CSC frames, matching frame-10 hash, no leftover processes.
- Recovery archives and the previously working game binary retained privately.
- No push performed.

## Acceptance checklist for future changes

- [ ] Incremental toolchain/game build succeeds using the active source manifest.
- [ ] Use the same ISO revision, overlay export, and documented test conditions.
- [ ] Bound each run; terminate/reap it and check for leftover processes.
- [ ] Preserve card dialog and logo before assessing new video progress.
- [ ] Check visible presented frames, nonblack samples, and IPU/GS markers.
- [ ] Compare decoded output with the known baseline where applicable.
- [ ] Do not introduce replacement frames or runtime game-address workarounds.
- [ ] Record exact validation conditions and limitations; retain private recovery data.
