# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- Clean-room rebuild from a fresh orphan `master`. The previous
  implementation was retired by the OxideAV docs audit dated
  2026-05-06; the prior history is preserved on the `old` branch.
  See `README.md` for the rebuild scope and the strict-isolation
  workspace the Implementer rounds will draw from.

### Blocked

- Round 84 (round 1 of the rebuild) targeted a foundational SV8
  stream-header parser. The only post-rebuild material under
  `docs/audio/musepack/` is `wiki/Musepack.wiki` — a 72-line
  multimedia.cx overview that links outward (to
  `trac.musepack.net`) for the SV7 and SV8 format specs but does
  **not** include any byte-level field layout, magic identifier,
  packet taxonomy, or table. Concretely missing for round 1:
  - SV7 header magic + sample-frequency / max-band / max-level /
    title / VBR fields and the 20-bit per-frame length prefix.
  - SV8 file magic + the SH / RG / EI / SO / ST / CT packet
    taxonomy, varint key/size framing, and the SH packet's
    sample-count / beginning-silence / sample-freq-index /
    max-used-bands / channel-count / ms-used / audio-block-frames
    field layout.
  - Huffman / VLC tables for SV7 (SCFI, DSCF, header, the seven
    quant-VLC sets) and SV8 (band, scfi, dscf, res, q1 / q2 / q3 /
    q4 / q5..q8 / q9up plus the CNS Pascal-grid / huffq2[125] /
    CC[19] / SCF[256] constants).
  Decoder code cannot land without violating the clean-room wall
  (no third-party Musepack implementation source consulted;
  `trac.musepack.net` not snapshotted). Unblock: docs-collaborator
  round that stands up `docs/audio/musepack/spec/` (SV7 + SV8
  byte-level field maps) and `docs/audio/musepack/tables/` (the
  Huffman / CNS / SCF tables), mirroring the layout the
  2026-05-04 audit catalogued for the now-retired writeup.
