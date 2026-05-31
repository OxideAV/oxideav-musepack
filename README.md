# oxideav-musepack

Pure-Rust Musepack audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Clean-room rebuild in progress.** This `master` branch is a fresh
orphan; the previous implementation was retired alongside the docs
audit dated 2026-05-06
([`AUDIT-2026-05-06.md`](https://github.com/OxideAV/docs/blob/master/AUDIT-2026-05-06.md)),
which found that the source-of-record trace document for this codec
was authored with a methodology that did not satisfy clean-room
separation. The prior history is preserved on the `old` branch for
archival but is forbidden input for the rebuild.

The `oxideav_core::CodecResolver` registration this crate will
expose through a future `register(ctx)` function is not wired yet;
the public API today surfaces the crate-local `Error` placeholder
plus the wired `requant`, `framing`, `huffman`, and `cns` modules
(see below).

## Docs status (round 191 — NEWLY UNBLOCKED)

A docs round closed `#927`, staging under `docs/audio/musepack/`:

- `musepack-sv7-sv8-spec.md` — a clean-room **structural / framing**
  spec authored in-repo from independent sources only (the
  multimedia.cx wiki snapshot, the Wikipedia article, the in-repo
  ISO/IEC 11172-3 Layer-II PDF, and general DSP knowledge). It
  establishes the Layer-II heritage, the SV7 frame container,
  band-type loop, SCF coding-method, per-band-type quantiser
  switch, SV8 KEY / SIZE / PAYLOAD packet vocabulary, and the SV8
  first-order-context quantiser switch.
- `tables/` — the SV7 / SV8 **entropy-coding, quantiser, CNS, and
  scalefactor numeric tables** extracted as Feist-v.-Rural-style
  facts (numeric initialisers only) by a walled extraction round
  documented in `provenance/01-musepack-table-extraction.md`. CSV
  + `.meta` sidecar per table, mirroring the
  `docs/audio/g729/tables/` convention.

## Round 191 — requantiser constants wired

Round 191 lands the `requant` module against the structural spec
§2.5 (frame body — quantised subband samples) and §2.6
(reconstruction), reading only `docs/audio/musepack/`:

- `RES_BITS: [u8; 18]` — bits per quantised sample per `band_type`
  in `0..=17`. Entropy-coded band types (`0..=7`) carry their
  sample width inside the Huffman codebook, so this array reports
  `0` for them; the linear-PCM escape ladder (`8..=17`) carries
  `band_type − 1` raw bits per sample.
- `QUANTIZER_OFFSET_D: [i16; 19]` — quantiser offset `D` per
  indexed band entry (number of distinct levels = `2 * D + 1`).
- `DEQUANT_COEFFICIENT_C: [f64; 19]` — dequant coefficient. The
  spec's relation `C = 65536 / (2 * D + 1)` is enforced as a unit
  test (max round-trip error `< 1e-6`) for all 18 non-CNS entries.
- `SCF_STEP_RATIO: f64` — geometric ratio between adjacent
  scalefactor-index gains (downward; upward is the reciprocal).
- `band_type_index(signed)` + `band_type_to_res_bits(unsigned)`
  helper functions.

12 unit tests cover length, spot-value, and the §2.6 product
relation; `cargo test` is green.

## Round 197 — SV7 Huffman entropy tables + CNS PRNG

Round 197 ingests the freshly-staged
`docs/audio/musepack/tables/` SV7 Huffman + CNS PRNG
CSVs into typed Rust tables via a new `build.rs` driver:

- New `huffman` module exposes the 10 SV7 `mpc_huffman`-shape
  tables (`SV7_BANDTYPE_HEADER_TABLE` / `SV7_SCFI_TABLE` /
  `SV7_DSCF_TABLE` / `SV7_Q1_TABLE` .. `SV7_Q7_TABLE`) as
  `[Sv7Entry; N]` constants. The `[2][N]` quantiser tables
  also offer a `sv7_q{1..=7}_ctx(ctx)` accessor returning the
  context-0 or context-1 half-slice per the staged sidecars'
  `notes:` line.
- `huffman::Sv7BitReader<'_>` is a small MSB-first bit reader
  over `&[u8]`; `huffman::decode(&mut reader, &table)` runs the
  staged "table sorted by Code descending, walk for first row
  with `code <= peek16()`" convention end-to-end. The SV7
  per-frame 20-bit length prefix + "read in 32-LSB units" outer
  packing per spec §2.2 is intentionally NOT here (it belongs
  to the frame-driver round).
- New `cns` module wires the CNS / noise-substitution PRNG
  from `cns-prng-{parity,params}.csv`: the 256-byte `PARITY`
  table plus six scalar constants drive a `CnsPrng` two-LFSR
  state machine whose step is transcribed verbatim from the
  `.meta` `notes:` line. The first step from the reset state
  is verified against a hand-cranked walk (samples bounded to
  `-510..=510`).
- `Error::HuffmanNoMatch` variant added for unmatched 16-bit
  windows.

22 new unit tests cover the bit reader, the per-table entry
count + last-entry assertion (one per staged CSV against its
`.meta` `resolved_dims` line), the context-pair split, the
end-to-end decode walk against three hand-traced
`bandtype-header` rows, the CNS parity table (full
popcount-mod-2 cross-check across all 256 bytes), and the
generator's first step / determinism / sample-range
invariants. Total crate test count `34 → 56`. `cargo test`,
`cargo clippy -- -D warnings`, `cargo fmt --check` all green.

The `build.rs` reads only the `.csv` numeric initialisers (the
Feist facts of the format) and emits typed `Sv7Entry` arrays
into `OUT_DIR`. An `OXIDEAV_MUSEPACK_DOCS_DIR` env-var override
lets the crate build outside the workspace checkout.

### Still gapped (post round 197)

- **SV8 canonical-Huffman entropy** —
  `docs/audio/musepack/tables/sv8-canonical-*.csv` (length
  tables) + `sv8-symbols-*.csv` (symbol maps) are staged but
  unwired. They're a different decoder shape (canonical length
  table + parallel symbol map, not the SV7 left-justified
  walker), and folding them in cleanly needs a canonical-Huffman
  builder that's the natural next round.
- **SV7 fixed-header field map** — same blocker as round 194.
- **SV7 frame container** — the per-frame 20-bit length prefix
  and the "read in 32-LSB units" bitstream packing (§2.2).
- **SV8 packet payload field maps** — SH / RG / EI / SO / ST.
- **Frame driver + synthesis subband filter** — ISO Layer-II
  tables live under `docs/audio/mp3/` and are reusable.

## Round 194 — SV7 / SV8 container magic + SV8 packet walker

Round 194 lands the `framing` module against the structural spec
§2.1 (SV7 identification), §3.1 (SV8 packet outer frame), and §3.2
(SV8 packet vocabulary), reading only `docs/audio/musepack/`:

- `SV7_MAGIC = b"MP+"` + `SV7_VERSION_NIBBLE = 7`, recognised by
  `SV7Header::parse_magic(&[u8])`. Returns the version byte and a
  slice over the still-GAP rest of the fixed header without
  interpreting any internal fields.
- `SV8_MAGIC = b"MPCK"`, recognised by `parse_sv8_magic(&[u8])`.
- `PacketKey` enum covering `SH` / `RG` / `EI` / `SO` / `ST` /
  `AP` / `SE` plus an `Unknown([u8; 2])` catch-all; round-trips
  through `from_bytes` / `as_bytes`.
- `PacketHeader` + `parse_packet_header` walking the
  `[2-byte key][varint size][payload]` SV8 outer frame plus a
  `parse_varint` decoder for the continuation-bit big-endian shape.
  The "is the size inclusive of the header?" convention is GAP per
  §3.1, so the parsed header exposes both `payload_len_inclusive()`
  and `payload_len_exclusive()` and the caller picks once the
  observer trace lands.
- `StreamKind` + `identify_stream` for magic-bytes-only dispatch
  between SV7 and SV8 streams.
- Crate `Error` extended with `InvalidMagic`, `UnexpectedEof`,
  `UnsupportedVersion(u8)`, and `VarintTooLong` variants.

22 new unit tests (`framing::tests::*`) cover the magic round-trips
for both versions, the full §3.2 packet-key vocabulary, varint
single / two / three-byte values + truncation + overlong rejection,
packet header parsing for `SH` / `AP` / `SE`, and a synthetic SV8
stream end-to-end walk reconstructing the packet sequence. Total
crate test count `12 → 34`; `cargo test` / `cargo clippy
-- -D warnings` / `cargo fmt --check` all clean.

### Still gapped

- **SV7 fixed-header field map** — sample count, intensity / MS
  flags, `max_band`, encoder profile / quality, gapless trailing-
  sample count, ReplayGain title / album gain + peak. The
  structural spec §2.1 defers all of this to the project's walled
  Trac `SV7Specification` page. **Blocked on workspace task
  #1263** (Musepack observer-trace round).
- **SV7 frame container** — the per-frame 20-bit length prefix and
  the "read in 32-LSB units" bitstream packing (§2.2) belong to the
  frame-body decoder, not the header parser, and are not wired yet.
- **SV8 packet payload field maps** — SH stream header, RG
  replaygain, EI encoder info, SO seek offset, ST seek table. Per
  §3.2 these are GAP and likewise blocked on task #1263. The
  packet outer frame is implemented this round; the inner bytes
  are returned as opaque slices.
- **SV8 varint convention** — whether the size field is inclusive
  of the key + size header. Both interpretations are exposed on
  `PacketHeader`; the choice will be made once the observer trace
  lands.
- **Huffman codebooks** — staged in `tables/` (SV7
  `sv7-huffman-*`, SV8 `sv8-canonical-*` + `sv8-symbols-*`) but
  not yet wired here.
- **CNS / noise-substitution generator** — staged in
  `tables/cns-prng-*` but not yet wired.
- **Frame driver + synthesis subband filter** — the ISO Layer-II
  filterbank tables live in `docs/audio/mp3/` and are reusable.

See `CHANGELOG.md` `[Unreleased]` for the per-round gap tracker.

## Codec category

Per the workspace's codec/container discipline, this crate owns the
**Musepack bitstream** only — SV7 frame layout and SV8 packet
structure (since SV8's packet framing is intrinsic to the format,
not a separate generic container the bitstream might-or-might-not
be carried in, akin to FLAC / MP3 / TTA / Shorten in the codecs-
with-dedicated-native-containers list). Container-level concerns
beyond the codec's intrinsic framing (e.g. APE-tag parsing for
ReplayGain metadata) route through the relevant sibling crate, not
here.

## Licence

MIT — see `LICENSE`.
