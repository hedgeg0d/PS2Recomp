# Additional decoder attribution

The software IPU MPEG decoder added in this branch follows PCSX2's IPU decoder
and its mpeg2dec-derived VLC/IDCT implementation.

- `ps2xRuntime/src/lib/ipu_mpeg2_vlc.h` retains the PCSX2 copyright and
  GPL-2.0-or-later notice, along with the original mpeg2dec notices.
- `ps2xRuntime/src/lib/ipu_mpeg2_decoder.cpp` is an adapted implementation.
  It is not a wholly original algorithm; the source records its provenance.
- Original mpeg2dec notices identify Michel Lespinasse (2000–2002) and
  Aaron Holtzman (1999–2000).

PS2Recomp's license is in [LICENSE](../LICENSE). Existing upstream dependency
notices remain applicable; this document is not an exhaustive dependency list.
