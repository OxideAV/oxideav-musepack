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

## Round 206 — SV7 §2.6 per-sample reconstruction primitives

Round 206 lands a new `reconstruct` module wiring the per-sample
dequantisation step that follows the per-band level decode of round
201, reading only `docs/audio/musepack/`:

- `DEQUANT_DIVISOR: f64 = 65536.0` constant tied to the requantiser
  relation `C = 65536 / (2D + 1)`.
- `centre_pcm_level(band_type, raw_unsigned) -> i32` — single-sample
  centring of a PCM-escape raw level (band_type 8..=17), subtracting
  `D = QUANTIZER_OFFSET_D[band_type + 1]`. Returns
  `Error::UnsupportedBandType` outside the PCM-escape range.
- `centre_pcm_band(band_type, &mut [i32; 36])` — same step applied
  in place to a full 36-sample band buffer.
- `dequantise_sample(band_type, centred_level) -> f64` — single-sample
  dequantise via `centred_level * C / 65536`, covering both the CNS
  / noise band (`-1`) and the normal `0..=17` range.
- `dequantise_band(band_type, &centred, &mut out)` — whole-band
  variant; the entropy-coded path (band_types 3..=7) feeds the
  function with the i8 → i32 widened Q-table values directly via the
  `dequantise_huffman_band` wrapper.
- `dequantise_cns_band(&cns_levels, &mut out)` — CNS-specific
  wrapper keyed off `DEQUANT_COEFFICIENT_C[0]` (the
  `32768/2/255*sqrt(3)` constant per the `cns-prng-params.meta`
  notes).
- `pcm_escape_d(band_type) -> Option<i32>` helper returning the `D`
  associated with a PCM-escape band_type, for callers that need to
  bounds-check raw input before centring.

18 new unit tests cover: PCM-centring at band_types 8 and 17
(boundaries `0`, `D`, `2D`), full-band centring in place,
out-of-range rejection for centring; single-sample dequant for
band_types 0 (identity scaling), 3, 17 (= `D / (2D + 1)`), and CNS
(`-1`); whole-band dequant against the single-sample path for
band_type 5; Huffman-band dequant for band_type 3 (signed i8 round
trip); CNS dequant magnitude bound check against the `-510..=510`
PRNG range; the `pcm_escape_d` helper across 8..=17; and a
cross-module integration test that wires the PCM-escape Sv7 reader,
the round-201 PCM-escape decoder, the round-206 centring step, and
the round-206 dequant multiply end-to-end against a known synthetic
input. Total crate test count `67 → 85`. `cargo test`,
`cargo clippy --all-targets --no-deps -- -D warnings`, and
`cargo fmt --check` are all green.

### Still gapped (post round 206)

- **SCF base-index gain table.** §2.6 needs a 256-entry SCF
  index → gain mapping; the geometric step ratio
  `SCF_STEP_RATIO ≈ 0.8330` is wired but the *anchor point*
  (gain at the reference index) is not specified by the structural
  prose. Treated as a follow-up that needs a one-paragraph spec
  addendum pinning the reference index and its gain value.
- **SV7 §2.5 grouped codewords** — cases 1 / 2 — same as round 201.
- **SV8 canonical-Huffman entropy walk** — same as round 201.
- **SV7 fixed-header field map** — same blocker as round 194.
- **SV7 32-LSB word packing** — same blocker as round 201.
- **SV8 packet payload field maps** — same blocker as round 194.
- **M/S undo + 32-band polyphase synthesis filter** — downstream
  of per-band sample reconstruction; deferred.

## Round 214 — SV7 §2.4 SCF coding-method decoder

Round 214 lands the `scf` module wiring the per-non-zero-band SCF
VLC loop documented in `docs/audio/musepack/musepack-sv7-sv8-spec.md`
§2.4 ("Frame body — scalefactor (SCF) coding") on top of the
round-197 staged `sv7-huffman-scfi` selector + `sv7-huffman-dscf`
delta tables. The new module:

- `ScfCodingMethod` typed wrapper around the `0..=3` SCFI value;
  `from_raw(i8)` validates the decoded SCFI VLC value and rejects
  anything outside that range with a new
  `Error::InvalidScfCodingMethod(i8)` carrying the offending value.
- `GranuleSchedule { deltas_to_read(), granule_to_slot() }` —
  classifies each SCFI value into the (count, per-granule
  delta-slot) pair specified by the Layer-II SCFSI convention
  §1 lines 79-82 ("scfsi==0: three SCFs, one per granule";
  "scfsi==1: two — first for granules 0+1, second for 2"; "scfsi==2:
  one shared across all three"; "scfsi==3: two — first for granule
  0, second for granules 1+2").
- `reconstruct_scf_from_deltas(reader, base, schedule)` — reads
  `1..=3` DSCF deltas, accumulates them against `base` (the §2.4
  "delta-coded against the previous one" rule), and projects
  through the granule mapping into the three per-granule SCF
  indices.
- `decode_band_scf(reader, base)` — end-to-end per-band entry:
  one SCFI VLC followed by N DSCF VLCs; returns the recovered
  method alongside the three SCF indices.
- `SCF_GRANULES_PER_BAND = 3` + `SCF_MAX_DISTINCT = 3` constants
  pinning the Layer-II-inherited band geometry.

The base anchor is sourced upstream (SV7 fixed-header field map,
GAP per §2.1); this module never touches the band-type header
VLC nor the per-sample quantiser VLC.

16 new unit tests cover: SCFI round-trip + reject-out-of-range
across `-1..=4` plus `i8` extremes; the four granule schedules
against their §1 source rows; delta reconstruction for all four
methods against hand-packed DSCF bit streams; end-to-end
`decode_band_scf` for method 2; `UnexpectedEof` propagation in
both phases; the invariant that every schedule's mapping
references only valid transmitted slots; and the
`SCF_GRANULES_PER_BAND == 3` constant sanity. Total crate test
count `85 → 101`. `cargo test`, `cargo clippy --all-targets
--no-deps -- -D warnings`, and `cargo fmt --check` are all green.

### Still gapped (post round 214)

- **Per-band SCF anchor**. The per-band base index the `scf`
  module accepts is sourced upstream — the SV7 fixed-header
  field map (`max_band`, etc.) and the per-band-vs-per-frame
  anchor convention are both DOCS-GAP under §2.1 / §2.2 and
  blocked on workspace task #1263.
- **SCF base-index gain table.** §2.6 still needs a 256-entry
  SCF index → gain mapping; the geometric `SCF_STEP_RATIO` is
  wired but the gain at the reference index is unspecified.
- **SV7 §2.5 grouped codewords** — cases 1 / 2 — same as round 201.
- **SV8 canonical-Huffman entropy walk** — same as round 201.
- **SV7 fixed-header field map** — same blocker as round 194.
- **SV7 32-LSB word packing** — same blocker as round 201.
- **SV8 packet payload field maps** — same blocker as round 194.
- **M/S undo + 32-band polyphase synthesis filter** — downstream
  of per-band sample reconstruction; deferred.

## Round 201 — SV7 §2.5 per-band sample-decode dispatcher

Round 201 wires the SV7 frame-body inner loop (`switch (band_type)`
per spec §2.5) as a new `sv7_band_decode` module on top of the
already-staged Huffman / CNS / requant tables, reading only
`docs/audio/musepack/`:

- `BandDecodeCase` classifier enum covers every spec case:
  `Cns` (-1), `Empty` (0), `Grouped3` (1), `Grouped2` (2),
  `HuffmanPerSample` (3..=7), `PcmEscape` (8..=17), `OutOfRange`
  (everything else). `band_type_case(i8) -> BandDecodeCase` is a
  pure `const fn` dispatch — no bit-stream access.
- `fill_zero_band(out)` — case 0, fills 36 zero samples.
- `fill_cns_band(prng, out)` — case -1, pass-through to the
  already-wired `CnsPrng::fill_samples`; each sample in `-510..=510`.
- `decode_huffman_band(reader, band_type, ctx, out)` — cases
  3..=7. Selects the right `Q{band_type}` table and the right half
  of the staged `[2][N]` context-pair (via `sv7_q{3..=7}_ctx`),
  then reads 36 Huffman codewords into the supplied `[i8; 36]`
  buffer. Returns `Error::UnsupportedBandType(bt)` for out-of-range
  `band_type` or `ctx`.
- `decode_linear_pcm_band(reader, band_type, out)` — cases
  8..=17. Reads `band_type - 1` (= 7..=16) unsigned bits per
  sample MSB-first via the existing `Sv7BitReader::read_bits` and
  stores raw pre-centring levels in `[i32; 36]`. The §2.6
  reconstruction step centres them by subtracting
  `D = QUANTIZER_OFFSET_D[band_type + 1]`; this round leaves the
  dequant arithmetic to the caller.
- New crate-level `Error::UnsupportedBandType(i8)` variant carries
  the offending `band_type` value (for both the structurally-
  documented-but-unimplemented grouped cases and the out-of-range
  default).

11 new unit tests cover the classifier across `-2..=18` plus
i8 extremes, the zero / CNS fill paths, both Huffman context halves
(3 + 7), an explicit ramp-pattern round-trip for PCM-escape cases 8
(7 bits/sample) and 17 (16 bits/sample), the `UnsupportedBandType`
rejection edges for every dispatcher, and EOF propagation through
the PCM-escape reader. Total crate test count `56 → 67`. `cargo
test`, `cargo clippy --all-targets --no-deps -- -D warnings`, and
`cargo fmt --check` are all green.

### Still gapped (post round 201)

- **SV7 §2.5 grouped codewords** — cases 1 (3 samples/codeword) and
  2 (2 samples/codeword): the per-codeword sample-unpack convention
  is not in the structural prose; the classifier knows the cases
  and the dispatcher fails loudly with `UnsupportedBandType`.
- **§2.6 reconstruction** — centring (subtract `D`) and dequant
  multiply (`* C / 65536`) and SCF scaling and synthesis filterbank
  are downstream of the per-band level decode; left to a later
  round once an end-to-end frame driver is in place.
- **SV8 canonical-Huffman entropy walk** — the staged
  `sv8-canonical-*` + `sv8-symbols-*` CSVs are present but the
  exact decode-walk convention (how the matched length-table row's
  `cumulative_index` + `code` map to a symbol-map offset) is
  underspecified in the staged prose. The cum-deltas suggest more
  codes per length than the strict prefix-free assignment allows
  for some rows; the structural spec doesn't disambiguate. Treated
  as DOCS-GAP this round; needs a one-paragraph addendum spelling
  out the offset arithmetic.
- **SV7 fixed-header field map** — same blocker as round 194
  (workspace task #1263 observer trace).
- **SV7 32-LSB word packing** — bit-within-word ordering of the
  per-frame 20-bit length prefix is underspecified in §2.2.
- **SV8 packet payload field maps** — SH / RG / EI / SO / ST,
  blocked on #1263.
- **Synthesis subband filter** — ISO Layer-II `D_i` window +
  `N_ik` matrix transcription deferred to a later round.

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
into `OUT_DIR`. The script tries three input locations in order:
`$OXIDEAV_MUSEPACK_DOCS_DIR/audio/musepack/tables/`, the
umbrella's live `docs/audio/musepack/tables/` (when built inside
the workspace), and the vendored snapshot at `<crate>/tables/`
which ships in the crate for standalone / CI / crates.io
builds. The vendored snapshot must stay byte-equal with the
umbrella's `docs/` staging; refreshing it is a manual step when
the docs collaborator restages.

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
