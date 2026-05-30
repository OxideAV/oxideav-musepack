# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.2](https://github.com/OxideAV/oxideav-musepack/releases/tag/v0.0.2) - 2026-05-30

### Other

- Round 191 — wire SV7/SV8 requantiser constants
- Round 186 — refresh docs-blocker assessment against 2026-05-25 staging
- Round 84 — file docs gap blocking SV7/SV8 header parse
- Round 0 — clean-room rebuild scaffold (orphan master)

### Added

- **Round 191** — `requant` module exposing the SV7 / SV8
  requantiser constants against
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.5 / §2.6:
  - `RES_BITS: [u8; 18]` — bits per quantised sample per
    `band_type` 0..=17 (0 for the entropy-coded ladder, 7..=16 for
    the linear-PCM escape ladder), transcribed from
    `docs/audio/musepack/tables/requant-res-bits.csv`.
  - `QUANTIZER_OFFSET_D: [i16; 19]` — offset `D` per indexed band
    entry (number of quantiser steps = `2 * D + 1`; index 0 = CNS /
    noise band entry), transcribed from
    `docs/audio/musepack/tables/requant-quantizer-offset-Dc.csv`.
  - `DEQUANT_COEFFICIENT_C: [f64; 19]` — dequant coefficient
    (`C = 65536 / (2 * D + 1)` for normal entries; index 0 carries
    the CNS / noise dequant constant), transcribed from
    `docs/audio/musepack/tables/requant-coefficient-Cc.csv`.
  - `SCF_STEP_RATIO: f64` — geometric ratio between adjacent
    scalefactor-index gains (downward direction), transcribed from
    `docs/audio/musepack/tables/scf-step-ratio.csv`.
  - `band_type_index` / `band_type_to_res_bits` helpers.
  - 8 unit tests exercising lengths, the §2.6 product relation
    `C * (2D + 1) == 65536` (max round-trip error `< 1e-6` across
    all 18 non-CNS entries), boundary values, and the helper
    mapping; total crate test count 12 lib tests, all green.

### Changed

- **Round 191** — README "Status" + "Docs status" sections rewritten
  against the docs-staging round that closed `#927`. The crate's
  module-level docs now point at the staged structural spec at
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` and the
  Feist-extracted tables at `docs/audio/musepack/tables/` as the
  source-of-record for further work, with the project-shipped
  reference material remaining link-only / off-limits to
  Implementer rounds.

### Pending (next-round candidates)

- **SV7 header field map** — the 20-bit per-frame length-prefix
  encoding and the fixed-header layout (sample-frequency /
  max-band / max-level / title / VBR fields). The structural spec
  at `docs/audio/musepack/musepack-sv7-sv8-spec.md` §2.1 defers
  this to the project's Trac wiki page, which is link-only per the
  clean-room policy; a clean-room observer-trace round is the
  recommended unblock.
- **SV8 packet payload field maps** — the KEY / SIZE varint
  packet framing is documented in the structural spec §3, but
  the per-packet payload bodies (SH stream header, RG replaygain,
  EI encoder info, SO seek offset, ST seek table) need a
  clean-room observer-trace round to map.
- **Huffman codebooks** — fully staged under
  `docs/audio/musepack/tables/` (SV7 `sv7-huffman-*`, SV8
  `sv8-canonical-*` + `sv8-symbols-*`) but not yet wired into
  Rust modules.
- **CNS / noise-substitution generator** — staged under
  `docs/audio/musepack/tables/cns-prng-*` (LFSR taps / seeds +
  256-byte `Parity` table) but not yet wired.
- **Frame driver + 32-band polyphase synthesis filter** — the
  ISO/IEC 11172-3 Annex B Table 3-B.3 synthesis window and
  matrix `N_ik` live under `docs/audio/mp3/` and are reusable.
- **Encoder.** Out of scope for the early Implementer rounds.

### Historical

- **Round 0** — clean-room rebuild from a fresh orphan `master`;
  the previous implementation was retired by the OxideAV docs
  audit dated 2026-05-06.
- **Round 84** — targeted a foundational SV8 stream-header parser
  and confirmed `docs/audio/musepack/` then contained only the
  multimedia.cx wiki overview; deferred to a future docs round.
- **Round 186** — README refresh restating the docs-blocker against
  the 2026-05-25 staging (workspace docs commit `78e2487`).
- **Round 191** — this round. See "Added" / "Changed" above.
