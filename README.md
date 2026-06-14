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
plus the wired `requant`, `framing`, `huffman`, `cns`,
`sv7_band_decode`, `reconstruct`, `scf`, `sv7_band_header`,
`packet_stream`, `typed_packet`, `stream_shape`, `sv8_band_decode`,
`sv8_huffman`, `sv8_sample_decode`, and `sv8_band_header` modules
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

## Round 294 — SV8 §3.4 frame-body band-resolution header walk

Round 294 lands the new `sv8_band_header` module: the §3.4
audio-packet frame-body **outer loop** that decides how many
subbands are coded and the per-band resolution, feeding the
round-288 `sv8_band_decode::decode_sv8_band` per-band dispatcher.
Reads only `docs/audio/musepack/`:

- `decode_used_subbands` reads one `sv8-canonical-bands`
  canonical-Huffman codeword (round-278 walk) into a used-subbands
  count in `0..=SV8_MAX_USED_SUBBANDS` (32 — the §1 Layer-II
  subband bound). The `bands` table's `.meta` `spec_role` pins this
  as the "number-of-used-subbands" selector; its shortest codeword
  decodes to `0` (no coded bands).
- `decode_band_resolutions` walks `nbands` bands, reading one
  `sv8-canonical-res-{1,2}` codeword each in ascending band order
  and returning the raw VLC value of each wrapped in `RawResVlc`.

Three §3.4 conventions stay **DOCS-GAP** and are threaded as caller
knobs (the established `decode_sv8_context_band` precedent) rather
than guessed:

- the `res-1`/`res-2` context-pair selection rule → a
  caller-supplied `ctx_for_prev_res` closure + `initial_ctx`
  (out-of-range context → `Error::UnsupportedBandType(i8::MIN)`, a
  reserved sentinel);
- the raw `res`-symbol (`0..=16`) → §3.4 `band_type` (`-1..=17`)
  remap, kept honest by the `RawResVlc` newtype (the SV8 sibling of
  `sv7_band_header::RawBandTypeVlc`) so it cannot be applied
  accidentally;
- per-channel ordering and any SH-packet `max_band` clamp on the
  count — left to a future round once the §3.2 SH field map and the
  channel-loop shape are pinned.

14 new unit tests (crate lib total `259 → 273`): per-row count /
resolution decode against every staged `bands` / `res` codeword,
context switching driven by the previous res, both context-fault
paths, EOF propagation, and the raw-`0..=16`-alphabet pin.
`cargo test` / `fmt --check` / `clippy -D warnings` all green.

## Round 288 — SV8 §3.4 classifier-driven band dispatcher

Round 288 lands `sv8_band_decode::decode_sv8_band`, the single
entry point that walks one SV8 band from its `band_type` alone:
route through the round-245 `sv8_band_type_case` classifier, then
call the matching per-arm decoder. It composes the grounded arms
that earlier rounds wired one at a time —

- `band_type == -1` (CNS) and `0` (empty) → the SV7-shared
  `fill_cns_band` / `fill_zero_band`;
- `2` (Grouped3), `3..=4` (Grouped2), `5..=8` (context) →
  `sv8_sample_decode`'s grouped/context decoders, their `[i8; 36]`
  output loss-free-widened into the dispatcher's unified buffer;
- `9..=17` (escape) → `decode_sv8_escape_band`'s native `[i32; 36]`,
  passed through unwidened.

All arms unify on an `[i32; 36]` output (the widest of the two
per-arm output types). The context knobs the staged tables do not
pin (`grouped_ctx` for the case-2 table-half pick; `initial_ctx` +
`ctx_for_prev` for the case-5..8 first-order context) are threaded
through verbatim as caller knobs — the dispatcher makes no choice
the staged material does not already determine.

**Fail-loud, never silently-wrong:** the sparse band (case 1)
stays a DOCS-GAP — the staged `sv8-symbols-q1` map is a 19-symbol
alphabet (`0..=18`) that cannot literally carry the §3.4 "flags for
18 samples" bitmap (which needs 2¹⁸ symbols), so its symbol →
flag-pattern mapping is underdetermined; the dispatcher returns
`Error::UnsupportedBandType(1)`. The `OutOfRange` catch-all
(`band_type < -1`) is likewise promoted from the classifier's
fail-quiet `case_emits_samples == false` to a hard error.

10 new unit tests across `sv8_band_decode::tests` cover: each
routed arm against its direct per-arm decoder as the oracle (CNS
PRNG-advance equality, empty zero-fill + zero-bit-read, grouped3 /
context widening, escape pass-through at band_type 9 and the
`(sym << 1) | raw` composition at band_type 10); the `grouped_ctx`
knob reaching only the case-2 table-half pick; and both fail-loud
arms. Crate lib test count `250 → 259`.

## Round 284 — SV8 §3.4 large-coefficient escape (default arm)

Round 284 closes the §3.4 `default` arm ("for each sample, read a
VLC plus a fixed number of raw bits", `band_type` 9..=17) in
`sv8_sample_decode`, reading only `docs/audio/musepack/` (spec
§3.4/§3.6, the wiki snapshot's quantizer ladder, and the
`sv8-{canonical,symbols}-q9up` + `requant-*` CSVs/`.meta`s). The
"fixed number" round 281 listed as unpinned is in fact derivable
from three already-staged facts:

1. the `sv8-symbols-q9up` map is an exact permutation of
   `-128..=127` — the full **signed-byte** alphabet, one table for
   the whole "9-and-up" range per its `.meta` `spec_role`;
2. `requant-res-bits.meta` scopes its bits-per-sample ladder to
   "SV7 §2.5 / **SV8 §3.4**", so an SV8 escape sample is
   `RES_BITS[band_type] = band_type − 1` bits wide in total;
3. `requant-quantizer-offset-Dc` pins the level range `±D`,
   `D = 2^(band_type−2) − 1`.

The only composition consistent with all three: the VLC symbol is
the sign-bearing top 8 bits, the `n = band_type − 9` raw bits are
the low bits — `sample = (symbol << n) | raw` tiles exactly the
`(band_type−1)`-bit two's-complement range `[−(D+1), D]` ⊇ `±D`
(raw-as-high-bits would put the sign in the raw field,
contradicting the signed map). New surface:

- `escape_raw_bits(band_type) -> Option<u8>` — the
  `RES_BITS[band_type] − 8` ladder (`0` for band_type 9 … `8` for
  17; `None` outside `9..=17`, where the staged requant tables
  define no quantiser) + `ESCAPE_VLC_SYMBOL_BITS = 8`.
- `decode_sv8_escape_band(reader, band_type, out: &mut [i32; 36])`
  — 36 × (q9up codeword + raw field). Output is the
  already-centred `i32` level (the staged map is signed), unlike
  the SV7 escape which emits uncentred levels for caller-side
  centring. The raw field is read MSB-first via the same
  `read_bits` primitive as the SV7 §2.5 escape, backed by §3.6's
  lossless SV7↔SV8 relationship (identical coefficient payload).

8 new unit tests: the signed-byte alphabet pin; the raw-bit ladder
vs `RES_BITS`; exact `[−(D+1), D]` range tiling against
`QUANTIZER_OFFSET_D` for all nine escape band_types; pure-VLC
band_type 9 with exact bit accounting; high/low composition under
alternating ± codewords with asymmetric raw patterns (band_type
13); MSB-first raw-field order at the widest field (band_type 17);
band_type rejection; EOF at the codeword and inside the raw field.
Crate lib test count `242 → 250`. `cargo test`, `cargo clippy
--all-targets --no-deps -- -D warnings`, and `cargo fmt --check`
all green under `CARGO_TARGET_DIR=/tmp/oxideav-musepack-r284-target`.

### Still gapped (post round 284)

- **SV8 §3.4 sparse band (case 1)** — the one §3.4 arm left. The
  staged q1 alphabet is 19 symbols (`0..=18`), which cannot
  literally carry the prose's "flags for 18 samples" (an 18-flag
  bitmap needs 2¹⁸ symbols), and the prose covers 18 samples
  while the band has 36. Newly grounded this round:
  `requant-quantizer-offset-Dc` pins `D = 1` for band_type 1, so
  sparse samples are levels in `{−1, 0, +1}` and the per-set-flag
  raw bit is presumably the ±1 sign. Still needed from a docs
  observer trace: (a) how many q1 codewords a 36-sample band
  reads, (b) the symbol → flag-pattern mapping, (c) the raw-bit →
  sign polarity.
- **q2 ctx-pair selection rule + `5..=8` context predicate +
  within-group emission order + escape raw-field bit order** —
  caller knobs / isolated conventions; an observer trace pins all
  four.
- **SCF absolute anchor**, **§3.2 packet payload field maps**,
  **§3.1 varint convention**, **§2.3 VLC-symbol → §2.5-case
  remap**, **SV7 §2.5 grouped codewords**, **SV7 fixed-header
  field map**, **SV7 32-LSB word packing**, **M/S undo + 32-band
  polyphase synthesis filter** — all unchanged from round 281.

## Round 281 — SV8 §3.4 per-case sample decoders (grounded subset)

Round 281 lands the `sv8_sample_decode` module: the §3.4 per-case
sample decoders for the grounded subset of the eight-variant ladder,
composing the round-278 canonical-Huffman decoder walk with the
round-245 `Sv8BandDecodeCase` classifier, reading only
`docs/audio/musepack/` (spec §3.4 + the wiki snapshot's quantizer
ladder, the `sv8-symbols-*` / `sv8-canonical-*` CSVs, and their
`.meta` `spec_role` lines, which pin the grouped-symbol structure
the round-278 README still listed as unspecified):

- **Grouped fan-out arithmetic is now data-derived.** The `.meta`
  `spec_role` strings name the packings — q2 "5x5x5 grouped", q3
  "7x7 grouped", q4 "9x9 grouped, padded" — and the staged maps
  prove them: both q2 maps are exact permutations of `0..=124`
  whose most-probable symbol is `62 = 2·25 + 2·5 + 2` (the all-zero
  triplet → base-5 digits, digit = sample + 2, samples `-2..=2`);
  the q3 map is an exact bijection onto `(-3..=3)²` and q4's first
  81 entries onto `(-4..=4)²` under signed two's-complement
  **nibble-pair** splitting of the int8 symbol (e.g. `0x11 → (1,1)`,
  `0x3F → (3,-1)`, `0xF0 → (0,-1)`); q4's 10 padding slots stay
  zero and unreachable per the round-278 tiling proof. Every
  q5..q8 map is a permutation of `-D..=D` (`D = 7/15/31/63`), so
  the per-sample symbol IS the centred level.
- `unpack_grouped3_symbol` / `unpack_grouped2_symbol` — the
  case-2 base-5 triplet and case-3/4 nibble-pair unpacks
  (`band_type` doubles as the per-nibble bound `D`); new defensive
  `Error::GroupedSymbolOutOfRange(i8)`.
- `decode_sv8_grouped3_band(reader, ctx, out)` — case 2: 12
  codewords via `sv8-canonical-q2-{1,2}`, 3 consecutive samples
  each. The q2 pair-selection rule is GAP (case 2 sits outside the
  §3.4 `5..=8` context range) so `ctx` is a caller knob, the
  `PacketSizeConvention` precedent.
- `decode_sv8_grouped2_band(reader, band_type, out)` — cases
  3..=4: 18 codewords via `sv8-canonical-q3` / `-q4`, 2 samples
  each.
- `decode_sv8_context_band(reader, band_type, initial_ctx,
  ctx_for_prev, out)` — cases 5..=8: 36 per-sample VLCs from the
  `q{5..8}` context pair; §3.4 pins that each sample's table is
  "chosen by the previously decoded sample" but not the predicate,
  which is a caller-supplied closure.
- **Residual convention:** the within-group emission order (which
  radix digit / nibble is the *first* of the consecutive samples)
  is underivable from the staged material — both assignments are
  bijections. The module emits least-significant-digit / low-nibble
  first, isolated inside the two `unpack_*` helpers for a one-line
  flip if a future observer trace pins the opposite order.
- Cases `-1` / `0` reuse the shared SV7 arms (`fill_cns_band` /
  `fill_zero_band`); the sparse band (case 1) and the
  large-coefficient escape (default arm) stay DOCS-GAP and fail
  loudly (the staged 19-symbol `0..=18` q1 alphabet cannot
  literally carry the prose's "flags for 18 samples", and the
  escape's "fixed number of raw bits" is unpinned).

16 new unit tests: staged-fact pins for the q2 / q3 / q4 / q5..q8
map structures; unpack hand-vectors, alphabet bijection, and
rejection edges; per-case band decodes with exact bit accounting;
codeword-order traces (alternating distinct codewords land on the
right groups); a context-switching trace driving the q5 pair off
the previous sample's sign; ctx / band_type rejection; EOF
propagation; and classifier-arm composition. Crate lib test count
`219 → 242`. `cargo test`, `cargo clippy --all-targets --no-deps
-- -D warnings`, and `cargo fmt --check` all green under
`CARGO_TARGET_DIR=/tmp/oxideav-musepack-r281-target`.

### Still gapped (post round 281)

- **SV8 §3.4 sparse band (case 1)** — the q1 symbol → 18-flag
  semantics are underdetermined by the staged material (19-symbol
  alphabet vs an 18-flag bitmap); needs a one-paragraph docs
  addendum.
- **SV8 §3.4 large-coefficient escape (default arm)** — the
  per-sample raw-bit count (and its `band_type` dependence) is
  unpinned by the structural prose.
- **q2 ctx-pair selection rule + `5..=8` context predicate +
  within-group emission order** — parameterised as caller knobs /
  an isolated convention this round; an observer trace pins all
  three.
- **SCF absolute anchor**, **§3.2 packet payload field maps**,
  **§3.1 varint convention**, **§2.3 VLC-symbol → §2.5-case
  remap**, **SV7 §2.5 grouped codewords**, **SV7 fixed-header
  field map**, **SV7 32-LSB word packing**, **M/S undo + 32-band
  polyphase synthesis filter** — all unchanged from round 278.

## Round 278 — SV8 §3.4 canonical-Huffman decoder walk

Round 278 closes the round-260 cumulative-index DOCS-GAP and lands
the SV8 canonical-Huffman **decoder walk** in the `sv8_huffman`
module, reading only `docs/audio/musepack/` (spec §3.4/§4, the
provenance §6 "running cumulative index into the paired symbol map"
sentence, and the staged `sv8-canonical-*` / `sv8-symbols-*` CSV
facts):

- The round-260 ambiguity ("two plausible sub-index assignments,
  not derivable from the table values alone") turned out to be
  resolvable from the staged numeric facts after all: a complete
  prefix code paired with an `N`-entry symbol map must tile the
  2^16 peek windows onto **exactly** the index range `0..N`.
  Exhaustively checking both candidate formulas over all 65536
  peeks for all 21 staged tables shows only one is a bijection:
  `index = (cum_index − (peek16 >> (16 − length))) mod 256`
  against the first row (code descending) with `code <= peek16`.
  The rejected alternative leaves holes (e.g. `bands` index 3 and
  `q1` indices 0, 1, 18 unreachable).
- `Sv8CanonicalTable::decode_symbol_index(reader) -> Result<usize>`
  — the walk itself: 16-bit MSB-first peek, descending-code row
  match (length-0 rows — the staged q4 sentinel — are skipped),
  mod-256 cumulative fold, `length`-bit consume. Defensive
  `HuffmanNoMatch` on an out-of-map index (unreachable for the
  staged tables).
- `Sv8CanonicalTable::decode(reader) -> Result<i8>` — index walk +
  paired symbol-map lookup; the SV8 sibling of the SV7
  `huffman::decode` entry point.
- The mod-256 fold is exact for `q9up`'s signed-int8 cumulative
  wrap (`..., 63, 125, -45, -7, -2, -1` → indices spanning the
  full 256-entry map) and the identity for the 20 unsigned-cum
  tables. The lone anomaly is `q4`: its rows tile indices
  `0..=80`; map entries `81..=90` are unreachable zero padding
  (the staged length-0 sentinel points at padding slot 90 and
  plays no part in the walk).

7 new unit tests cover: the full 2^16-peek tiling proof re-run per
table as `decode_tiles_symbol_map_bijectively` (every index
reachable exactly `2^(16−length)` times, q4 padding unreachable);
hand-traced `q1` decode vectors across four length classes; the
`q9up` signed-wrap vectors (`0xFFFF → −128`, `0x8000 → −87`,
`0x4000 → 65`, `0x0000 → −2`); back-to-back decode chaining with
exact bit consumption on `bands`; the q4 sentinel-skip path; the
q4 zero-padding pin; and 16-bit-peek EOF propagation. Crate lib
test count `212 → 219`. `cargo test`, `cargo clippy --all-targets
--no-deps -- -D warnings`, and `cargo fmt --check` all green under
`CARGO_TARGET_DIR=/tmp/oxideav-musepack-r278-target`.

### Still gapped (post round 278)

- **SV8 §3.4 per-case sample decoders** — now UNBLOCKED by the
  decoder walk: the sparse-band flag VLC read, grouped-codeword
  sample unpack, first-order-context table selection, and
  large-coefficient escape raw-bit count are the natural next
  rounds (the grouped-codeword *unpack arithmetic* — how one
  symbol fans out into 2 or 3 samples — is still unspecified in
  the structural prose, as for SV7 §2.5).
- **SCF absolute anchor**, **§3.2 packet payload field maps**,
  **§3.1 varint convention**, **§2.3 VLC-symbol → §2.5-case
  remap**, **SV7 §2.5 grouped codewords**, **SV7 fixed-header
  field map**, **SV7 32-LSB word packing**, **M/S undo + 32-band
  polyphase synthesis filter** — all unchanged from round 272.

## Round 272 — §2.6 relative scalefactor (SCF) gain ladder

Round 272 extends the `reconstruct` module with the §2.6 *relative*
SCF gain ladder, reading only
`docs/audio/musepack/tables/scf-step-ratio.meta` plus the staged spec
§2.6. The `.meta` notes line pins both the geometric step ratio
(`f *= 0.83298066476582673961` per index) and the ladder dimension
("256 indices"). The *absolute* anchored gain table remains GAP — its
reference-index gain value is unspecified in the structural prose —
but the gain *ratio* between any two SCF indices is anchor-independent
and therefore fully determined:

- `SCF_INDEX_COUNT = 256` — the SCF index ladder size.
- `scf_relative_gain(from, to) -> f64` — infallible
  `SCF_STEP_RATIO^(to − from)`; unity at `from == to`, quieter
  (`< 1.0`) for a higher index, louder (`> 1.0`) for a lower one.
- `scf_gain_relative_to_anchor(anchor, &mut [f64; 256])` — the full
  anchor-relative ladder (unity at `anchor`, strictly decreasing).
- `apply_scf_relative(from, to, &mut [f64; 36])` — scales a
  dequantised band by the relative gain, applying a per-granule SCF
  index *difference* off a shared base without needing the GAP
  absolute anchor (result exact up to one global constant scale).

13 new unit tests cover the 256-index dimension, identity, one-step
up/down (ratio / reciprocal), inverse symmetry, exponent additivity,
`n`-step == `ratio^n`, anchor-unity + per-entry agreement, strict
monotonic decrease, and `apply_scf_relative` scaling / identity /
inverse round trip. Crate lib test count `199 → 212`. `cargo test`,
`cargo clippy --all-targets --no-deps -- -D warnings`, and
`cargo fmt --check` all green under
`CARGO_TARGET_DIR=/tmp/oxideav-musepack-r272-target`.

### Still gapped (post round 272)

- **SCF absolute anchor.** The relative ladder is now wired; the
  *absolute* gain at the reference SCF index is still unspecified in
  the structural prose. A fully-anchored decode needs a one-paragraph
  spec addendum pinning the reference index and its gain value.
- **SV8 §3.4 canonical-Huffman cumulative-index decoder walk**,
  **§3.2 packet payload field maps**, **§3.1 varint convention**,
  **§2.3 VLC-symbol → §2.5-case remap**, **SV7 §2.5 grouped
  codewords**, **SV7 fixed-header field map**, **SV7 32-LSB word
  packing**, **M/S undo + 32-band polyphase synthesis filter** — all
  unchanged from round 260.

## Round 260 — SV8 canonical-Huffman length-tables + symbol maps wired

Round 260 vendors the 21 staged `sv8-canonical-*.csv` length-tables
and 21 paired `sv8-symbols-*.csv` symbol maps under
`docs/audio/musepack/tables/` into the crate's `tables/` snapshot
and wires them as typed Rust statics through a new `sv8_huffman`
module. Reading material stayed inside
`docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.4 / §4 plus the
companion `provenance/01-musepack-table-extraction.md` §6 (the
21-pair table inventory).

- `Sv8CanonicalEntry` — typed row of an SV8 canonical length
  table: `(code: u16 left-justified, length: u8, cum_index: i16)`,
  matching the staged `.meta` `value_encoding` line. The
  `cum_index` field widens to `i16` to accommodate the `q9up`
  large-coefficient-escape map's signed-int8 cumulative wrap
  (`..., 63, 125, -45, -7, -2, -1`).
- `Sv8CanonicalTable` — paired (length-table, symbol-map) view
  carrying both slices plus the staged CSV stem (e.g.
  `"sv8-canonical-bands"`) as a diagnostic `name` field.
- 21 catalogue tables wired as `pub static` arrays under
  identifier prefixes `SV8_BANDS`, `SV8_RES_{1,2}`,
  `SV8_SCFI_{1,2}`, `SV8_DSCF_{1,2}`, `SV8_Q1`, `SV8_Q2_{1,2}`,
  `SV8_Q3`, `SV8_Q4`, `SV8_Q5_{1,2}`..`SV8_Q8_{1,2}`,
  `SV8_Q9UP`, each emitted by `build.rs` from the CSV pair. A
  `SV8_CANONICAL_CATALOGUE` array exposes the 21 tables in a
  single ordered slice for iteration.
- `Sv8TableRole` enum + `table_for_role(role, ctx) -> Option<&Sv8CanonicalTable>`
  dispatcher: maps a §3.4 / §3.5 spec role plus a first-order
  context bit (0 or 1) into the matching physical table.
  Context-pair roles (`Res`, `Scfi`, `Dscf`, `Q2`, `Q5`, `Q6`,
  `Q7`, `Q8`) return `None` for out-of-range `ctx ≥ 2`; non-pair
  roles ignore `ctx`.

`build.rs` grows a third emitter (`emit_sv8_canonical_tables`)
that mirrors the existing SV7 `mpc_huffman` walk: parse the
length-table CSV `(code_hex, length, cum_index)`, parse the
companion symbol-map CSV (`int8` per row), emit a
`Sv8CanonicalEntry` array, a `[i8; N]` symbol array, and a single
`Sv8CanonicalTable` paired-view static per pair. The 21 vendored
CSV pairs (84 files) are committed under `tables/` to keep the
crate buildable standalone for crates.io / CI consumers.

The build script tolerates the only anomalous staged row: the
`sv8-canonical-q4.csv` final `(0x0000, 0, 90)` sentinel that
follows the real length-10 terminator. The sentinel is preserved
verbatim in the static array (the CSV is source-of-truth) and
explicitly pinned + documented by the `q4_sentinel_row_is_documented`
test.

24 new unit tests under `sv8_huffman::tests` cover: catalogue
shape (21 entries; all names unique + `sv8-canonical-`
prefixed); per-table row counts against every staged `.meta`
`resolved_dims` value (`bands` 12/33; `res-{1,2}` 16/12 + 17/17;
`scfi-{1,2}` 3/5 + 4/16; `dscf-{1,2}` 12/13 + 64/65; `q1` 10/19;
`q3` 7/49; `q4` 8/91; `q2-{1,2}` 10/9 + 125/125; `q5..=q8`
context pairs; `q9up` 6/256); first/last row equality for `bands`;
data-row code descending; data-row length non-decreasing;
data-row length within `1..=16`; data-row terminating at code
`0x0000`; `code` low `16 - length` bits zero (left-justification
invariant); cumulative-index progression (strict-increase for the
20 unsigned-cum tables, modular-256 progression for `q9up`); the
q4 sentinel pin; `min_length` / `max_length` helpers; `table_for_role`
context-ignore for non-pair tables; `table_for_role` context-pair
dispatch for all 8 pair roles; `table_for_role` rejecting
ctx ≥ 2; the catalogue `name` field carrying the CSV stem;
`Sv8CanonicalTable::{len_table_rows, sym_table_rows}` helpers;
`bands` symbol map spanning `0..=32`; `q9up` symbol-map endpoints
(`-128, ..., -2`).

Crate test count `176 → 200`; `cargo test`, `cargo clippy
--all-targets --no-deps -- -D warnings`, and `cargo fmt --check`
all green under `CARGO_TARGET_DIR=/tmp/oxideav-musepack-r260-target`.

### Decoder convention — DOCS-GAP (carried forward)

The structural spec names the canonical Huffman layer and pins
the row layout but does **not** pin the arithmetic that maps a
peeked 16-bit code window into a symbol index against the
cumulative-index column. A Kraft-McMillan count check rules out
the naive "one row covers `2^(16 − Length)` peek bins"
formulation: the staged tables routinely skip intermediate
lengths (e.g. `q1` rows go length 3, 4, 5, 6, 7, … with the
length-3 row's `cum_index = 7` exceeding the 5 length-3
peek-bins the row's code range admits). Two plausible
interpretations (forward-ascending vs descending-from-cum
sub-index assignment within a length class) give incompatible
symbol mappings, and the choice is not derivable from the table
values alone. Per the project's "ask for docs, don't fish" rule,
the decoder walk is left for the round that follows a §3.4 docs
patch resolving the cumulative-index convention; this round
stops cleanly at the typed-table surface.

### Still gapped (post round 260)

- **SV8 §3.4 canonical-Huffman cumulative-index decoder walk** —
  the arithmetic mapping `(peek, code, length, cum_index)` to a
  symbol-index. Tables now wired; walk semantics DOCS-GAP per
  §3.4.
- **SV8 §3.4 per-case sample decoders** — the actual sparse-band
  flag VLC read, grouped-codeword sample unpack, first-order-
  context table selection, and large-coefficient escape raw-bit
  count. Blocked downstream of the cumulative-index walk above.
- **§3.2 packet payload field maps**, **§3.1 varint convention**,
  **§2.3 VLC-symbol → §2.5-case remap**, **per-band SCF anchor**,
  **SCF base-index gain table**, **SV7 §2.5 grouped codewords**,
  **SV7 fixed-header field map**, **SV7 32-LSB word packing**,
  **M/S undo + 32-band polyphase synthesis filter** — all
  unchanged from round 245.

## Round 245 — SV8 §3.4 per-band sample-decode case classifier

Round 245 lands the `sv8_band_decode` module: a pure structural
case classifier mirroring [`sv7_band_decode::BandDecodeCase`] for
the SV8 §3.4 audio-packet frame-body `switch (band_type)` ladder,
reading only `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.4:

- `Sv8BandDecodeCase` — eight-variant enum routing each `band_type`
  to its §3.4 `switch` arm: `Cns` (`-1`), `Empty` (`0`),
  `SparseBand` (`1`), `Grouped3` (`2`), `Grouped2` (`3` / `4`),
  `ContextHuffmanPerSample` (`5..=8`), `LargeCoeffEscape`
  (`band_type >= 9`, the `default` arm), and `OutOfRange` (every
  value below `-1`).
- `sv8_band_type_case(band_type: i8) -> Sv8BandDecodeCase` — pure
  `const fn` dispatch, total over the full `i8` range. Sibling of
  [`sv7_band_decode::band_type_case`]; the SV8 classifier captures
  the §3.4 ladder shifts relative to the §2.5 ladder (sparse band
  insertion at `case 1`, grouped cases shifted up by one,
  first-order context arm at `case 5..=8`, large-coefficient escape
  at `band_type >= 9`).
- `case_emits_samples(case) -> bool` predicate isolating the §3.4
  outer-loop "for each non-zero band" arms (every variant except
  `Empty` / `OutOfRange`).
- `case_uses_first_order_context(case) -> bool` predicate isolating
  the §3.4 SV8-specific first-order context-modelled per-sample
  Huffman arm (`band_type` in `5..=8`) — the "Huffman table chosen
  by the previously decoded sample" detail the §3.4 prose
  highlights as the heart of the "2% smaller files / faster
  decoding" Wikipedia note.

16 new unit tests across `sv8_band_decode::tests` cover:
classification of `band_type == -1` (Cns), `0` (Empty), `1`
(SparseBand), `2` (Grouped3), `3` / `4` (Grouped2), `5..=8`
(ContextHuffmanPerSample), `9..=64` and `i8::MAX`
(LargeCoeffEscape), and `-2 / -10 / -100 / i8::MIN` (OutOfRange);
full-`i8`-range classifier totality; the `case_emits_samples`
truth table per §3.4 arm; the `case_uses_first_order_context`
truth table; band_type-range cross-check of
`case_uses_first_order_context` against `5..=8`; const-evaluation
sanity at five representative band types; the SV7-vs-SV8 ladder
divergence on the grouped-case indices (SV7 case 1 = Grouped3,
SV8 case 1 = SparseBand; SV7 case 2 = Grouped2, SV8 case 2 =
Grouped3; SV7 case 3 = HuffmanPerSample, SV8 case 3 = Grouped2;
SV7 case 5 = HuffmanPerSample, SV8 case 5 =
ContextHuffmanPerSample); the shared-arm agreement on `case -1`
(Cns) and `case 0` (Empty) between the two sibling classifiers;
and the `Copy` / `Eq` / `Debug` invariants. Total crate test
count `160 → 176`. `cargo test`, `cargo clippy --all-targets
--no-deps -- -D warnings`, and `cargo fmt --check` all green.

### Still gapped (post round 245)

- **SV8 §3.4 per-case sample decoders** — the actual sparse-band
  flag VLC read, grouped-codeword sample unpack, first-order-
  context table selection, and large-coefficient escape raw-bit
  count live downstream of the SV8 canonical-Huffman entropy layer
  (`sv8-canonical-*` length tables + `sv8-symbols-*` symbol maps
  under `docs/audio/musepack/tables/`, GAP per the structural
  spec's §4 caveat until the length-table-to-code-table builder
  lands).
- **§3.2 packet payload field maps**, **§3.1 varint convention**,
  **§2.3 VLC-symbol → §2.5-case remap**, **per-band SCF anchor**,
  **SCF base-index gain table**, **SV7 §2.5 grouped codewords**,
  **SV8 canonical-Huffman entropy walk**, **SV7 fixed-header
  field map**, **SV7 32-LSB word packing**, **M/S undo + 32-band
  polyphase synthesis filter** — all unchanged from round 239.

## Round 239 — SV8 stream-shape observer

Round 239 lands the `stream_shape` module on top of the round-228
`PacketStream` walker and the round-232 `TypedPacket` classifier,
reading only `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 +
§3.2:

- `StreamShape { sh_count, rg_count, ei_count, so_count, st_count,
  ap_count, se_count, unknown_count, total_payload_bytes,
  first_kind, last_kind }` — structural summary of one SV8 byte
  stream. Per-§3.2-kind counters cover the full vocabulary; the
  `Unknown` aggregator absorbs every 2-byte key outside the
  vocabulary; `total_payload_bytes` is the cumulative opaque payload
  byte tally (header bytes excluded); `first_kind` / `last_kind`
  record the classified key of the first and last emitted packet.
- `scan_sv8_stream(input, convention) -> Result<StreamShape>` —
  pure observer entry point. Validates the leading `MPCK` magic via
  `framing::parse_sv8_magic`, drives a `PacketStream` over the
  post-magic slice with the caller-supplied
  `PacketSizeConvention` pick (the still-GAP §3.1 varint
  convention), classifies each emitted `PacketRef` via
  `TypedPacket::classify`, and accumulates the shape until the
  walker reports `Ok(None)`.
- `StreamShape::total_packets()`, `is_empty()`, `saw_stream_end()`,
  and `count_for(PacketKey)` accessors. `count_for` routes the
  `Unknown` variant to the single `unknown_count` aggregator
  regardless of the raw 2-byte key value.
- No payload field interpretation, no ordering enforcement: the
  shape simply records what the upstream walker emitted in the
  order it emitted it. SH-first / SE-last invariants stay GAP per
  the structural prose.

15 new unit tests across `stream_shape::tests` cover: rejection of
non-`MPCK` magic and short input; empty post-magic slice yielding
the all-zero shape; single-`SE` terminator path; full §3.2
vocabulary walk (`SH` + `RG` + `EI` + `SO` + `ST` + `AP` + `SE`) in
spec-table order with correct first/last; repeated `AP` packet
aggregation into a single counter; multiple unknown 2-byte keys
aggregated into `unknown_count` while `first_kind` / `last_kind`
preserve the actual raw bytes / known kind; `count_for` routing for
every §3.2 key plus the `Unknown` catch-all; `UnexpectedEof`
propagation on a truncated payload; trailing-garbage-after-`SE`
ignored by the walker so it does not contribute to the shape;
`se`-less stream still reporting `first_kind` / `last_kind` for
what was actually seen; `total_payload_bytes` measuring payload
only (header bytes excluded); inclusive-convention scan with
`raw_size` matching `header_len + payload_len`; default shape
all-zero / empty; `Copy` + `Eq` invariants on `StreamShape`; and
`first_kind` locking in on the first packet only (not overwritten
by later packets). Total crate test count `145 → 160`. `cargo
test`, `cargo clippy --all-targets --no-deps -- -D warnings`, and
`cargo fmt --check` all green.

### Still gapped (post round 239)

- **§3.2 packet payload field maps** — SH / RG / EI / SO / ST
  inner-byte layouts are still DOCS-GAP per the structural spec's
  "Field layout" column; the stream-shape observer aggregates
  payload byte counts only.
- **§3.1 varint convention**, **§2.3 VLC-symbol → §2.5-case
  remap**, **per-band SCF anchor**, **SCF base-index gain table**,
  **SV7 §2.5 grouped codewords**, **SV8 canonical-Huffman entropy
  walk**, **SV7 fixed-header field map**, **SV7 32-LSB word
  packing**, **M/S undo + 32-band polyphase synthesis filter** —
  all unchanged from round 232.

## Round 232 — typed SV8 packet surface

Round 232 lands the `typed_packet` module on top of the round-228
`PacketStream` walker, reading only
`docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 + §3.2:

- Per-kind borrowed newtypes covering the full §3.2 packet
  vocabulary: `StreamHeaderPacket<'a>` (`SH`),
  `ReplayGainPacket<'a>` (`RG`), `EncoderInfoPacket<'a>` (`EI`),
  `SeekTableOffsetPacket<'a>` (`SO`), `SeekTablePacket<'a>` (`ST`),
  `AudioPacket<'a>` (`AP`), `StreamEndPacket<'a>` (`SE`). Each
  newtype carries the opaque payload slice the upstream walker
  emitted and exposes a single `payload_bytes() -> &'a [u8]`
  accessor; per-field maps remain GAP per the §3.2 "Field layout"
  column.
- `TypedPacket<'a>` sum routing each known key to its per-kind
  newtype plus an `Unknown { key: [u8; 2], payload: &'a [u8] }`
  catch-all that preserves the raw bytes of any 2-byte key outside
  the §3.2 vocabulary (forward-compat for the pending observer-
  trace round).
- `TypedPacket::classify(PacketRef<'a>) -> TypedPacket<'a>` — pure
  infallible classification of one walker-surfaced packet.
- `TypedPacket::key()` / `payload_bytes()` accessors plus
  `is_stream_end()` / `is_audio()` / `is_metadata()` predicates for
  log / filter loops; the three predicates are mutually exclusive
  (and all `false` for `Unknown`).

10 new unit tests across `typed_packet::tests` cover: routing of
every known §3.2 key into its matching typed variant; `Unknown`
preservation of raw key bytes and payload; a seven-packet
end-to-end walk (`SH` + `RG` + `EI` + `SO` + `ST` + `AP` + `SE`)
through `PacketStream::next_packet` + `TypedPacket::classify`;
metadata-only filter counting on a mixed `SH` / `AP` / `RG` /
`AP` / `SE` stream; the `payload_bytes()` accessor agreeing with
the inner newtype's accessor across every variant; empty-payload
round-trip across every variant; an unknown 2-byte key (`ZQ`)
traversing the walker into `TypedPacket::Unknown` without an
error; the `Copy` / `Eq` invariants on both `TypedPacket` and its
inner newtypes; classification independence from
`PacketHeader::raw_size` / `header_len`; and the mutual-exclusion
property of `is_metadata` / `is_audio` / `is_stream_end`. Total
crate test count `135 → 145`. `cargo test`, `cargo clippy
--all-targets --no-deps -- -D warnings`, and `cargo fmt --check`
all green.

### Still gapped (post round 232)

- **§3.2 packet payload field maps** — SH / RG / EI / SO / ST
  inner-byte layouts are still DOCS-GAP per the structural spec's
  "Field layout" column; this round adds typed wrappers but does
  not introduce any field decode.
- **§3.1 varint convention**, **§2.3 VLC-symbol → §2.5-case
  remap**, **per-band SCF anchor**, **SCF base-index gain table**,
  **SV7 §2.5 grouped codewords**, **SV8 canonical-Huffman entropy
  walk**, **SV7 fixed-header field map**, **SV7 32-LSB word
  packing**, **M/S undo + 32-band polyphase synthesis filter** —
  all unchanged from round 228.

## Round 228 — SV8 packet-stream walker

Round 228 lands the `packet_stream` module on top of the round-194
SV8 packet outer-frame parser (`framing::parse_packet_header`),
reading only `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 +
§3.2:

- `PacketSizeConvention { Inclusive, Exclusive }` — explicit pick
  for the still-GAP varint convention (spec §3.1 flags as GAP
  whether `raw_size` counts the key + size header bytes or only the
  payload). Callers stand a walker up against one interpretation;
  the pending observer-trace round will pin one as the only valid
  reading.
- `PacketRef<'a> { key, header, payload: &'a [u8] }` — one decoded
  packet, with `payload` borrowed from the underlying byte slice
  (no allocation per packet).
- `PacketStream<'a>` — walker built from the post-`MPCK`-magic
  slice plus a `PacketSizeConvention` pick. `next_packet() ->
  Result<Option<PacketRef<'a>>>` yields one packet per call,
  returns `Ok(None)` after the §3.2 `SE` terminator or an empty
  input, propagates `UnexpectedEof` / `VarintTooLong` from the
  outer-frame parser, and locks into a stopped state on either an
  `SE` or a hard error so subsequent calls quietly return
  `Ok(None)`.
- Crate-root `SAMPLES_PER_FRAME_PER_CHANNEL = 1152` constant pinning
  the Layer-II 32-subband × 36-samples-per-band frame geometry per
  §1 lines 65-71, cross-checked by a unit test against
  `SV7_SUBBAND_COUNT * SAMPLES_PER_BAND`.

15 new unit tests across `packet_stream::tests` cover: the
`Inclusive` / `Exclusive` convention round-trip; empty input
yielding `None` and stopping; single-`SE` terminator path; a
three-packet stereo walk (`SH` + `AP` + `SE`); stop-at-`SE` with
trailing garbage in the buffer (the walker leaves the stream
exhausted without erroring on the leftover bytes); the inclusive
convention with a synthetic packet whose `raw_size` includes the
3-byte header; inclusive-mode rejection of a sub-header `raw_size`
with `Error::VarintTooLong`; `UnexpectedEof` propagation on a
declared-but-truncated payload; malformed-header `UnexpectedEof`;
the `remaining_bytes()` cursor shrinking on each successful read;
forward-compat surfacing of unknown 2-byte keys via
`PacketKey::Unknown`; full-walk count of a five-packet synthetic
stream; the post-error stopped-state invariant (the walker does
not re-emit the same error on subsequent calls); and the lifetime
guarantee that `PacketRef::payload` borrows from the input slice
rather than copying. Plus one new crate-root test pinning the
`SAMPLES_PER_FRAME_PER_CHANNEL` constant. Total crate test count
`120 → 135`. `cargo test`, `cargo clippy --all-targets --no-deps
-- -D warnings`, and `cargo fmt --check` all green.

### Still gapped (post round 228)

- **§3.1 varint convention** — inclusive vs exclusive of the
  header bytes — still DOCS-GAP per the structural spec; the new
  walker takes both interpretations as a `PacketSizeConvention`
  knob so it can wire up either reading the moment the observer
  trace pins one.
- **§2.3 VLC-symbol → §2.5-case remap**, **per-band SCF anchor**,
  **SCF base-index gain table**, **SV7 §2.5 grouped codewords**,
  **SV8 canonical-Huffman entropy walk**, **SV7 fixed-header field
  map**, **SV7 32-LSB word packing**, **SV8 packet payload field
  maps** (SH / RG / EI / SO / ST), **M/S undo + 32-band polyphase
  synthesis filter** — all unchanged from round 223.

## Round 223 — SV7 §2.3 band-type header loop

Round 223 wires the §2.3 per-band header loop — the structural
iteration block that drives `band_type` VLC + conditional `msflag`
across `0..=max_band` — into a new `sv7_band_header` module on top of
the round-197 `SV7_BANDTYPE_HEADER_TABLE` Huffman table and the
`Sv7BitReader`, reading only `docs/audio/musepack/`:

- `SV7_SUBBAND_COUNT = 32` + `SV7_MAX_BAND_INCLUSIVE = 31` constants
  pinning the Layer-II 32-subband geometry inherited per spec §1
  lines 53-71.
- `RawBandTypeVlc(i8)` typed wrapper around the raw `i8` value
  produced by one invocation of the `sv7-huffman-bandtype-header`
  VLC. The wrapper exposes `as_i8()` and `is_nonzero()` but not
  arithmetic; it keeps the (still-DOCS-GAP) §2.3-VLC-symbol →
  §2.5-dispatcher-case remap honest by preventing accidental
  composition with the [`sv7_band_decode`] dispatchers.
- `BandHeader { band_type: [RawBandTypeVlc; 2], ms_flag:
  Option<bool> }` — one entry per band, with `ms_flag == None`
  when §2.3's conditional suppressed the flag (both channels'
  `band_type == 0`) and `ms_flag == Some(true|false)` otherwise
  (true = M/S, false = L/R). `has_samples()` short-cuts the §2.5
  inner loop's "for non-zero bands" predicate.
- `decode_band_header(reader, nch) -> Result<BandHeader>` — one
  band's read: `nch` (1 or 2) bandtype-header VLCs followed by the
  conditional 1-bit `msflag`. Mono treats the single channel's VLC
  as occupying both slots so the predicate fires the same way.
- `decode_header_loop(reader, max_band, nch) ->
  Result<Vec<BandHeader>>` — the full §2.3 outer loop walking
  `i = 0..=max_band`. Returns `max_band as usize + 1` entries.
- New crate-level `Error::MaxBandOutOfRange(u8)` and
  `Error::ChannelCountInvalid(u8)` variants for the structural
  parameter-validation surface.

19 new unit tests cover: the Layer-II 32-subband geometry constants;
`RawBandTypeVlc` round-trip + `is_nonzero` across `-5..=4`;
`BandHeader::has_samples` across the four channel-zero-pattern
combinations; `decode_band_header` for stereo both-zero (no msflag),
stereo left-non-zero (msflag=1 read), stereo right-non-zero
(msflag=0 read), mono both-zero (no msflag), and mono non-zero
(msflag-consumed); rejection of `nch` outside `{1, 2}` (`0`, `3`,
`8`, `255`); `UnexpectedEof` propagation in the left-VLC phase and
the msflag phase; `decode_header_loop` rejection of `max_band > 31`
(values `32`, `200`); rejection of `nch` outside `{1, 2}` in the
loop entry point; `max_band == 0` returning a single-band vector;
a three-band stereo walk covering all three msflag outcomes; the
maximally-wide stereo frame (`max_band == 31` → 32 bands, 64 bits
all-zero); and mid-loop `UnexpectedEof` propagation. Total crate
test count `101 → 120`. `cargo test`, `cargo clippy --all-targets
--no-deps -- -D warnings`, and `cargo fmt --check` all green.

### Still gapped (post round 223)

- **§2.3 VLC-symbol → §2.5-case remap**. The bandtype-header VLC's
  symbol alphabet (`-5..=4` per the staged
  `sv7-huffman-bandtype-header.csv`) does not cover §2.5's
  dispatcher domain (`-1..=17`). The structural §2.5 prose uses
  `band_type` directly in its `switch`, so an upstream remap is
  implied — but the **shape** of that remap (delta-from-previous,
  context-keyed transform, lookup table) is unspecified in the
  structural prose. Tracked as DOCS-GAP alongside the §2.5
  grouped-case unpack and the §2.2 word-packing.
- **Per-band SCF anchor**, **SCF base-index gain table**, **SV7
  §2.5 grouped codewords**, **SV8 canonical-Huffman entropy
  walk**, **SV7 fixed-header field map**, **SV7 32-LSB word
  packing**, **SV8 packet payload field maps**, **M/S undo +
  32-band polyphase synthesis filter** — all unchanged from round
  214.

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
