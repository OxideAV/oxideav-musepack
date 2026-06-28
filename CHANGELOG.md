# Changelog

All notable changes to this crate are documented in this file. The format
follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Round 378** — the SV7 **32-bit-word body byte-swap** (`sv7_word_swap`),
  closing the body-byte-extraction half of the standing SV7 whole-stream
  word-swap gap. `word_swap_sv7_body` turns a raw SV7 body buffer (the
  continuous bit run after the §1 fixed header) into the byte order the
  `huffman::Sv7BitReader` walks: each aligned 4-byte group is reversed
  (little-endian 32-bit word → big-endian byte order for the MSB-first
  reader), per `spec/musepack-headers-and-coding.md` §4 ("read in 32-LSB
  units"). A partial trailing group zero-extends to a full word before
  the reversal so every real body byte lands at its word-swapped
  position; `word_swap_sv7_body_in_place` is the allocation-free variant
  for already-word-aligned buffers. SV8 needs no analogue (§4: SV8 bytes
  load in natural order). 12 unit tests (single/multi-word reversal, all
  three trailing-partial-word cases, word-padded output length, in-place
  vs allocating agreement, double-swap identity, LE→BE equivalence).
- **Round 378** — `sv7_stream::Sv7StreamDecoder::from_header` builds a
  stream decoder straight from a parsed `sv7_header::Sv7HeaderFields`,
  pulling `max_band` (§1 field 4) and the stream-wide M/S flag (field 3)
  from the header rather than having the caller re-extract them. The §1
  word-swap is now defined once: `sv7_header`'s private `word_swap_sv7`
  is a `#[cfg(test)]` alias of `sv7_word_swap::word_swap_sv7_body` and
  the header `parse` path calls the shared module directly (the SV7
  header and body share the identical §4 swap). `Sv7HeaderFields` gains
  a `Default` derive for ergonomic construction. 2 tests.
- **Round 378** — `sv7_stream::Sv7StreamDecoder::decode_body_bytes`
  wires the §4 swap into the stereo stream driver: it takes a **raw**
  (non-swapped) SV7 body byte buffer, applies `word_swap_sv7_body`
  internally, and runs the existing frame loop — so a caller no longer
  has to hand-swap the body before constructing the bit reader. The
  whole-stream *positioning* (where the header ends / body begins) stays
  the caller's job; this method owns only the word-swap once the body
  bytes are in hand. 2 tests (PCM-identical to the pre-swapped-reader
  path over three silent frames; empty-body → no PCM). Lib suite
  547 → 561.
- **Round 371** — the **stream-level decode loop**, taking the
  reconstruction path end-to-end to PCM (relative loudness) for the
  grounded subset of both stream generations:
  - `sv7_stereo_frame::decode_sv7_stereo_frame` — SV7 §5 **two-channel**
    frame decode + reconstruction. Composes the §5.1 shared band-type
    header (both channels + per-band M/S flags), the §5.3/§5.4 "Left
    channel is decoded first, then right" per-channel body sweeps, and
    per-channel `reconstruct_frame_channel` into an `Sv7StereoFrame
    { channels, ms_flags }` — the input `ms_stereo::undo_ms_stereo` + the
    synthesis filterbank consume. Shared CNS PRNG threads across both
    channels. No new format facts (composition of grounded sub-walks in
    the documented §5 phase order).
  - `sv7_stream::Sv7StreamDecoder` — SV7 **stereo stream driver** owning
    the cross-frame state (persistent `MultiChannelSynthesis` — the
    filterbank overlap spans 15 frames — shared CNS PRNG, §2.6 M/S-undo
    closure). `decode_frame` runs the full §2.6 per-frame pipeline
    (stereo decode + reconstruct → M/S undo → synthesis, interleaved);
    `decode_frames` loops it across the non-byte-aligned (§2.2) bit run.
    The whole-stream word-swap body bit-alignment (§2.2/§4) is *not*
    assumed — takes a caller-positioned reader.
  - `sv8_stream::Sv8MonoStreamDecoder` — SV8 **mono stream driver**, the
    SV8 counterpart: persistent single-channel `SynthesisFilter` + shared
    CNS PRNG threaded across the `block_power`-derived frames of an `AP`
    packet; `decode_frame`/`decode_frames` over a per-frame
    `Sv8FrameParams { nbands, new_block }` schedule.
  - `sv8_decode::decode_sv8_mono_stream` — first wiring of the SV8
    **packet layer → audio decode**: walks an `MPCK` buffer, reads the
    `SH` header, drives an `Sv8MonoStreamDecoder` over every `AP` packet
    as one §6.2 key frame (own `Max_used_Band` log code), emitting
    `Sv8DecodedStream { header, audio_packets, pcm }`. Supported subset
    is mono + `block_power == 0` + key-frame `AP`; out-of-subset streams
    are rejected with precise errors (`ChannelCountInvalid`,
    new `Error::UnsupportedBlockPower`) rather than decoded wrong.
  - 23 new tests; lib suite 524 → 547.
  - **Still GAP** (unchanged): the §2.6 M/S-undo *arithmetic* (closure
    knob), the absolute SCF anchor gain, the SV8 per-channel band
    interleaving, the SV8 per-frame `Max_used_Band` position in a
    multi-frame `AP`, and the SV7 whole-stream word-swap body
    bit-alignment (no in-repo SV7 fixture corpus to validate it).
- **Round 366** — the §2.6 final step: the **32-band polyphase
  synthesis subband filter** (`synthesis`), the last stage that turns a
  frame's reconstructed `frame_reconstruct::SubbandMatrix` into PCM
  samples. This closes the largest standing reconstruction gap and is
  the PCM-producing stage for **both** stream generations (SV7 and SV8
  share the identical inherited Layer-II filterbank, spec §1 lines
  55-66). The §2.6 "GAP" framing was resolved: the algorithm + tables
  live in the in-repo ISO/IEC 11172-3:1993 PDF under `docs/audio/mp3/`,
  a standards-body source the Musepack spec (§1, source S3) explicitly
  authorises transcribing — not a gap.
  - `SynthesisFilter` holds the persistent 1024-entry `V` FIFO
    (zero-initialised at startup per ISO Figure 3-A.2 footnote 1) and
    its `synthesize(&[f64; 32]) -> [f64; 32]` runs the five-step
    reconstruction the ISO Annex A Figure A.2 "Synthesis subband filter
    flow chart" lays out **verbatim**: (1) shift the `V` FIFO up by 64,
    (2) matrix `V[i] = Σ_k N_ik·S_k` into `V[0..64]`, (3) build the
    512-value `U` vector by the documented `V[i·128+…]` gather, (4)
    window `W[i] = U[i]·D[i]`, (5) sum 16 windowed taps per output
    sample `out_j = Σ_{i=0}^{15} W[j+32i]`. `reset()` re-zeroes the FIFO
    at a seek / channel boundary.
  - `SYNTHESIS_WINDOW: [f64; 512]` — ISO Table 3-B.3 "Coefficients D_i
    of the synthesis window", all 512 values transcribed verbatim by
    visual reading of 400-DPI page renders of the in-repo ISO PDF
    (Annex B). The table is the artefact of the original numerical-
    optimisation run (not closed-form), so it is loaded as data; it is
    magnitude-symmetric about D[256] (the windowed-sinc peak
    1.144989014). Transcription was cross-checked region-by-region for
    the sign-transition boundaries the table's multi-lobe structure
    makes error-prone (D[128]/D[192]/D[320]/D[377] etc.).
  - `matrix_coefficient(i, k)` — the matrixing coefficient
    `N_ik = cos[(16+i)(2k+1)π/64]`, computed from the closed-form
    formula the ISO figure prints (no `N_ik` table is transcribed; the
    spec gives the formula, not a table).
  - `synthesize_frame_channel(filter, matrix) -> [f64; 1152]` — the
    frame driver: consumes a per-channel `SubbandMatrix` **column by
    column** (one sample from every subband per time slot) over the 36
    time slots, emitting the 1152 PCM samples of one channel-frame in
    output order. The `filter` carries inter-frame `V` overlap and must
    be reused across consecutive frames of one channel.
  - `MultiChannelSynthesis` — persistent multi-channel synthesis state
    holding one `SynthesisFilter` per channel (`new(nch)` for `nch` ∈
    {1, 2}). Because the filterbank's windowed sum reaches back into the
    previous 15 frames' matrixed `V` blocks, a stream needs one filter
    **per channel reused across all frames**; this type owns them so the
    per-frame driver continues the overlap rather than discontinuing it
    every 1152 samples. `synthesize_channel_frame(ch, matrix)` advances
    one channel's filter; `reset()` re-zeroes all.
    `synthesize_stereo_frame_interleaved(state, left, right)` runs both
    channels and interleaves them into `2 × 1152` PCM samples in
    `L, R, L, R, …` order (the post-M/S-undo L/R matrices in, ready for
    a PCM sink).
  - 22 new unit tests: window length / endpoints / peak-at-256 /
    full magnitude-symmetry over all 255 mirror pairs (the strongest
    transcription guard) / sign-boundary pins; `N_ik` formula endpoints
    (cos π/4, cos π/2, cos π); fresh-FIFO-is-zero, zero-in-zero-out,
    first-slot-uses-only-fresh-V, filter linearity
    (`synth(a·x)=a·synth(x)`), inter-call state carry, reset; and
    frame-driver silence, slot-by-slot-driver agreement, and full-1152
    geometry; plus multi-channel bad-`nch` rejection, per-channel
    overlap == standalone-filter agreement across two consecutive
    frames, channel independence, out-of-range channel + mono-state
    rejects, reset, and stereo-interleave `L,R,…` ordering vs the
    per-channel reference. Crate lib `502 → 524`.
  - Still downstream of PCM: the M/S undo arithmetic and the absolute
    SCF anchor gain remain GAP (both documented), and a stream-level
    decode loop that wires header → frame-decode → reconstruct → M/S →
    this filterbank → interleaved PCM output is the next integration
    step.

- **Round 363** — SV7 single-channel **frame-body assembler**
  (`sv7_frame_decode`), the SV7 counterpart of `sv8_frame_decode`.
  `decode_sv7_frame_channel(reader, res_per_band, first_scf_ref, cns)`
  takes one channel's §5.1 `Res` sequence and walks each band in the §5
  phase order, emitting a `frame_reconstruct::BandLevels` sequence ready
  for `reconstruct_frame_channel`:
  - `Res == 0` (empty) ⇒ no record (subband stays silent);
  - `Res == -1` (CNS) ⇒ 36 PRNG samples, no SCF layer;
  - `Res` in `1..=17` (coded) ⇒ the §5.3 SCF decode (threading the
    previous coded band's `SCF[2]`), the §5.4 **1-bit context selector**
    (read only for the grouped `1`/`2` and per-sample-Huffman `3..=7`
    cases — gated by `band_type_uses_context_selector`), then 36 sample
    levels via `decode_sv7_band`.
  - Negative pre-anchor SCF indices saturate to `0` in the `BandLevels`
    `u32` triple (relative-anchor convention).
  - 11 new unit tests (selector predicate, empty/CNS/coded/PCM-escape
    bands, prev-band SCF threading, subband-index interleaving, negative
    SCF saturation, EOF propagation, band-count + band-type rejects).
  - Cross-channel interleaving, the M/S undo, and the absolute SCF anchor
    remain GAP (the same gaps the SV8 assembler documents).
- **Round 363** — SV7 §5.1 **grounded** `Res` (band-type) header decode
  (`sv7_band_header::decode_res_header_grounded`). The staged §5.1 closes
  the `RawBandTypeVlc → band_type` remap GAP the structural §2.3 walker
  left open: the header VLC codes a **per-channel `Res` delta chain** that
  produces the §5.4 sample-switch `band_type` directly.
  - Band 0: each channel's `Res` is a raw 4-bit absolute.
  - Bands 1+: `Res[n] = Res[n-1] + idx` off the *same channel's* previous
    band, with VLC symbol `idx == 4` (`RES_HEADER_ESCAPE_SYMBOL`) an
    escape to a raw 4-bit absolute.
  - Per-band M/S bit read only when the stream-wide M/S flag is set, the
    stream is stereo, and the band has a non-zero channel; surfaced as
    `Sv7ResBand::ms_flag`.
  - New `Sv7ResBand` type (per-channel `res` + `ms_flag`) with
    `has_samples()`; 8 new unit tests (band-0 raw, later-band delta,
    escape, stereo msflag present/absent/stream-off, channel/max-band
    rejects, constants).
- **Round 363** — SV7 §5.3 **grounded** scalefactor decode (`sv7_scf_decode`).
  The staged `spec/musepack-headers-and-coding.md` §5.3 pins the SV7 SCF
  model precisely, and it differs from the simpler Layer-II-schedule path
  in `scf`. New `decode_sv7_band_scf(reader, prev_band_scf2) -> Sv7BandScf`:
  - `SCF[0]` is **always** independently coded, delta vs the *previous
    band's* `SCF[2]` (threaded forward via `Sv7BandScf::last_index`) — not
    a free caller anchor.
  - `SCF[1]`/`SCF[2]` are each coded (delta off the *immediately preceding
    index*) or copied from it, per the cell-for-cell §5.3 SCFI table
    (`0`→code,code,code; `1`→code,code,copy; `2`→code,copy,code;
    `3`→code,copy,copy). The "code off a copied predecessor" case (SCFI 2)
    is one the `scf` slot table cannot express.
  - The §5.3 `idx == 8` DSCF **escape** (`DSCF_ESCAPE_SYMBOL`) switches any
    coded index to a raw 6-bit absolute, ignoring its delta reference.
  - The §5.3 "decoded index exceeding 1024 ⇒ sentinel" clamp is surfaced
    as the observable `Sv7BandScf::clamped` flag (indices returned
    verbatim; silent-band substitution left to reconstruction).
  - 10 new unit tests (all four SCFI cases, escape on `SCF[0]` and on a
    later index, prev-band threading, clamp trip/no-trip, read-count
    helper, EOF propagation).
- **Round 359** — SV8 frame-decode → reconstructed subband-sample bridge
  (`sv8_reconstruct`). The SV8 counterpart of `frame_reconstruct` for
  SV7: turns the `Vec<Sv8BandDecode>` output of
  `decode_sv8_frame_channel` into a per-channel `SubbandMatrix` of `f64`
  subband samples (dequant + per-granule SCF multiply), ready for the
  §2.6 M/S undo + synthesis filterbank.
  - `reconstruct_sv8_band(band, anchor, out)` — per-band dispatch on the
    signed `Res`: `-1` CNS (CNS dequant constant, no SCF layer), `0`
    empty (silent), `1..=17` coded (dequant the already-signed levels by
    the SV7-shared quantiser, then the three signed per-granule SCF gains
    relative to `anchor`).
  - `reconstruct_sv8_frame_channel(bands, anchor) -> SubbandMatrix` —
    lands each band in its `subband` row; uncoded subbands stay silent.
  - `decode_and_reconstruct_sv8_channel(reader, nbands, new_block, cns,
    anchor) -> SubbandMatrix` — the single integration point from raw
    frame-body bits straight to reconstructed subband samples for a mono
    (or one resolved channel of a stereo) stream.
  - A **dedicated** SV8 path (not `frame_reconstruct`) because SV8 levels
    are already-signed/centred for *every* arm (the §6.4 large-coeff
    escape carries the sign in its symbol), so there is no SV7-style
    PCM-escape centring; and SV8 SCF indices are signed (`−6..=121`),
    using the new signed SCF primitives. SV8 reuses the SV7 quantiser
    (§3), so the `DEQUANT_COEFFICIENT_C` / `QUANTIZER_OFFSET_D` entries
    are shared by `Res` number.
  - 12 new unit tests (silence, empty-row, uncoded-stay-zero, direct
    dequant of centred levels, negative SCF granule scaling, CNS dequant,
    multi-band rows, subband / band-type rejects, per-band == frame-path
    agreement, end-to-end == two-step + zero-band silence).
  - The absolute SCF anchor, multi-channel interleaving, M/S undo
    arithmetic, and synthesis filterbank remain GAP / out-of-scope-of-
    `docs/audio/musepack/`. Crate lib `461 → 473`.

- **Round 359** — signed-index SCF gain primitives for the SV8
  reconstruction path (`reconstruct`): `scf_relative_gain_signed(from,
  to) -> f64` and `apply_granule_scf_relative_signed(anchor,
  granule_scf, band)`. The SV8 §6.3 DSCF fold `SCF = ((prev − 25 +
  delta) & 127) − 6` recenters by `−6`, so a reconstructed SV8 SCF index
  lies in the signed range `−6..=121` rather than the SV7 `u8` ladder.
  The SCF gain is purely geometric (`SCF_STEP_RATIO^(to − from)`, anchor-
  and sign-independent), so these lift the `u8` bound of
  `scf_relative_gain` / `apply_granule_scf_relative` to `i32` without an
  offset hack. 4 new unit tests. The absolute anchor gain remains GAP —
  relative loudness between granules and anchor-sharing bands is exact.

- **Round 356** — SV8 single-channel audio-packet frame-body assembler
  (`sv8_frame_decode`). `decode_sv8_frame_channel(reader, nbands,
  new_block, cns) -> Vec<Sv8BandDecode>` is the integrating layer that
  joins the grounded SV8 sub-walks in the documented frame-body phase
  order (§2.3–§2.6):
  - **Resolution sweep** — one `decode_band_resolutions_grounded` reads
    every coded band's `band_type` (§6.2, ascending order).
  - **Per non-zero band** — the §6.3 SCFI decode (`decode_sv8_scfi`,
    `nonzero_channels = 1`), the §6.3 per-granule SCF reconstruction
    (`decode_sv8_band_scf`, threading the previous band's `SCF[2]` as the
    next band's `SCF[0]` reference), then the §3.4 sample decode
    (`decode_sv8_band_grounded`). Empty (`band_type 0`) bands emit a
    silent record with no reads; CNS (`band_type -1`) bands fill from the
    shared PRNG with no SCF layer.
  - Output `Sv8BandDecode { subband, band_type, granule_scf: [i32; 3],
    levels: [i32; 36] }` — the structured per-channel decode the §2.6 /
    §3.6 reconstruction (dequant + per-granule SCF multiply + synthesis
    filterbank) will consume. Multi-channel interleaving, the M/S undo,
    and the cross-phase SCF/sample ordering remain GAP.
  - 8 new unit tests: zero-band, `nbands > 32` reject, single empty band
    (resolution-only read), single CNS band (PRNG match + state advance,
    no SCF), single coded band (SCFI → new-block SCF → 36 samples),
    multi-band `prev_scf2` threading across two coded bands, ascending
    subband numbering, and resolution-phase EOF. Crate lib `436 → 457`.

- **Round 356** — SV8 §6.3 DSCF → SCF-index reconstruction, now grounded
  (was a GAP-knob raw walk). §6.3 pins the full base-plus-delta SCF index
  decode `decode_sv8_band_scf(reader, scfi, new_block, prev_scf2) ->
  [i32; 3]`:
  - **`SCF[0]`** — `new_block` ⇒ raw 7-bit absolute index minus 6; else
    a `sv8-canonical-dscf-2` delta (escape symbol 64 ⇒ `+ raw 6 bits`)
    folded `((SCF_prev2 − 25 + delta) & 127) − 6`.
  - **`SCF[1]` / `SCF[2]`** — copied from the previous granule when the
    SCFI marks them shared (§5.3 case table, `scfi_coded_granules`), else
    a `sv8-canonical-dscf-1` delta (escape symbol 31 ⇒ `64 + raw 6 bits`)
    folded the same way.
  - The DSCF context is no longer a caller knob: `SCF[0]` reads `dscf-2`,
    later granules `dscf-1`. The legacy GAP-knob `decode_dscf_deltas` is
    retained for callers wanting the pre-arithmetic raw values.
  - 8 new unit tests: the §5.3 coded/shared schedule, the new-block
    absolute path, the non-new-block dscf-2 fold off `prev_scf2`, all
    three granules coded with forward-folding dscf-1 deltas, both escape
    paths (dscf-2 64 / dscf-1 31), a `scfi > 3` reject, and EOF.

- **Round 356** — SV8 §6.3 SCFI selector decode, now grounded (was a
  GAP-knob). The freshly-staged `spec/musepack-headers-and-coding.md`
  §6.3 closes the `scfi-1` vs `scfi-2` context-selection GAP and pins
  the packed-value L/R split that `decode_scfi_selectors` left to the
  caller:
  - **Context** — chosen by the band's non-zero-channel count: `0`/`1`
    non-zero ⇒ `sv8-canonical-scfi-1` (ctx 0); both ⇒ `-2` (ctx 1).
  - **Packed L/R split** — `left = value >> (2·cnt)`, `right = value &
    3`, with `cnt` = additional non-zero channels beyond the first
    (`1` for a stereo both-non-zero band, else `0`). So a stereo band's
    single `scfi-2` codeword carries both channels' SCFI selectors and a
    single-channel band's `scfi-1` codeword is the lone selector.
  - `decode_sv8_scfi(reader, nonzero_channels) -> Sv8BandScfi {left,
    right}`: the knob-free §6.3 decode, returning two `0..=3` SCFI
    selectors ready to drive the SV7-shape granule schedule
    (`ScfCodingMethod`). The legacy GAP-knob `decode_scfi_selectors`
    (closure + `RawScfiVlc` output) is retained.
  - 5 new unit tests: single-channel direct value across `0..=3`, the
    both-channels packed split across the full `scfi-2` `0..=15`
    alphabet, exhaustive `(L, R)` combo recovery, a `>2`-channel reject,
    and grounded EOF.

- **Round 353** — SV8 §6.2 band-resolution (`Res`) header walk, now
  grounded (was two GAP-knobs). The freshly-staged
  `spec/musepack-headers-and-coding.md` §6.2 closes the
  `decode_band_resolutions` caller closures with two pinned rules:
  - **Context selection** — the `res-1` (ctx 0) vs `res-2` (ctx 1)
    pick is "whether the band-above `Res` exceeds 2" (`> 2 ⇒ ctx 1`);
    the top used band reads ctx 0. New `res_ctx_for_above`.
  - **Top-down delta + signed wrap** — bands decode highest-index
    first; the top band's raw value is the `band_type` after "values
    above 15 wrap by −17" (raw `16 ⇒ −1` CNS), and each lower band
    folds `Res[n] = canon(Res, ctx) + Res[n+1]`, re-wrapped. New
    `wrap_res`. This closes the `RawResVlc → band_type` remap GAP the
    module documented: the grounded walk emits signed `i8` band_types
    in ascending band order, ready for `sv8_band_type_case`.
  - `decode_band_resolutions_grounded(reader, nbands) -> Vec<i8>`: the
    knob-free §6.2 walk. The legacy GAP-knob `decode_band_resolutions`
    (closure + `RawResVlc` output) is retained.
  - 8 new unit tests: `wrap_res` ring pins (pass-through + `16→−1` +
    post-delta `30→13`), the `>2` context predicate, single-band
    ctx-0 wrap across the full res-1 alphabet, an equivalence
    cross-check against a hand-replicated §6.2 walk over five varied
    band chains, ascending-order verification, and mid-walk EOF.
    Crate lib total `416 → 424`.

- **Round 353** — SV8 §6.2 `Max_used_Band` decode + §6.5 bounded "log"
  code, grounded. The new staged §6.2/§6.5 pins how the per-packet
  coded-band count is read:
  - `decode_log_code(reader, max)`: the §6.5 phased-/truncated-binary
    "log" code over `0..max` — reads `floor(log2(max−1))` bits, plus
    one extra bit when the value lands in the `lost = 2^bitlen − max`
    tail (`bitlen = ceil(log2(max))`). Reuses the same lost-codes
    convention as the §6.5 enumerative coder
    (`sv8_sample_decode::enum_decode_subset`'s prefix step); `max ≤ 1`
    reads nothing. Also serves §6.2's M/S `cnt`.
  - `decode_keyframe_max_used_band(reader, max_band)`: §6.2 key-frame
    rule — a log code over `0..max_band+1`.
  - `decode_nonkey_max_used_band(reader, last_max_band)`: §6.2 non-key
    rule — `Max_used_Band = last_max_band + canon(Bands)` (signed delta
    via `sv8-canonical-bands`), "results > 32 wrap by subtracting 33".
  - 7 new unit tests: exhaustive log-code roundtrip (every value for
    `max` 1..=40 via a reference phased-binary encoder), `max ≤ 1`
    no-read, power-of-two `max` = plain fixed width, key-frame log-code
    over `0..max_band+1` for several `max_band`, non-key delta-fold +
    the >32→−33 wrap, and EOF propagation. Crate lib total `424 → 431`.

- **Round 353** — SV8 §6.2 mid/side band-selection **bitmap decode**,
  closing the last band-header GAP that fed the `ms_stereo` undo step.
  `decode_sv8_ms_flags(reader, tot) -> Vec<bool>` decodes which of the
  `tot` non-zero-channel bands carry a per-band M/S flag:
  - `cnt` (how many bands are M/S) via the §6.5 bounded log code over
    `0..tot+1`; `cnt == 0` ⇒ none, `cnt == tot` ⇒ all (no enumerative
    bits).
  - otherwise a §6.5 enumerative codeword selecting `min(cnt, tot−cnt)`
    of `tot`, bit-inverted when `cnt > tot/2` (the smaller subset is
    always coded). This reuses the same enumerative coder the sparse
    arm uses (`enum_decode_subset`, now `pub(crate)`).
  - the bitmap is returned **top-down**: `out[0]` is the topmost
    non-zero band, matching the §6.2 application order. The caller maps
    these `tot` flags onto the actual band indices for the
    `ms_stereo::undo_ms_stereo` step.
  - 5 new unit tests incl. an **exhaustive** roundtrip over every flag
    pattern for `tot` 1..=12 (exercises `cnt==0`, `cnt==tot`, and the
    complement-inversion middle), top-down ordering on single-band
    flags, and EOF. Crate lib total `431 → 436`.

- **Round 348** — SV8 §6.4.2 first-order context model, now grounded
  (was two GAP-knobs). The staged
  `spec/musepack-headers-and-coding.md` §6.4.2 closes the
  context-selection predicate the `sv8_sample_decode` arms carried as
  caller knobs — the cases-5..=8 "table chosen by the previously
  decoded sample" rule and the wholly-unspecified case-2 (`q2`)
  context-pair rule.
  - New `sv8_context` module: `context_threshold()` (per-`Res`
    thresholds `Res 2→3, 5→1, 6→3, 7→4, 8→8`), `Sv8Context` accumulator
    (`new()` inits `idx = 2×thres`, `table_ctx()` = context-1 when
    `idx > thres`, `update_sample()` folds `idx = (idx>>1)+|q|`,
    `update_group()`/`case2_magnitude()` folds `idx = (idx>>1)+var[tmp]`
    with `var[tmp]` the summed magnitude of the three base-5 samples
    `tmp` encodes — computed, no new table).
  - `decode_sv8_context_band_grounded` (cases 5..=8) and
    `decode_sv8_grouped3_band_grounded` (case 2): knob-free per-sample /
    per-group table selection driven by `Sv8Context`.
  - `decode_sv8_band_grounded`: the canonical band dispatcher routing
    cases 2 and 5..=8 through the grounded decoders (CNS / empty /
    sparse / grouped-2 / escape arms unchanged; the knob variants
    `decode_sv8_context_band` / `decode_sv8_grouped3_band` /
    `decode_sv8_band` are retained).
  - 22 new unit tests: threshold/accumulator pins, an exhaustive
    cross-check that `case2_magnitude` matches `unpack_grouped3_symbol`
    for all 125 `tmp`, first-sample-uses-ctx-1 (`idx` init), equivalence
    to a hand-replicated §6.4.2 accumulator for both arms, and
    dispatcher cross-checks. Crate lib total `394 → 416`.

- **Round 344** — SV7/SV8 §2.6 mid/side (M/S) stereo-undo *structure*,
  new `ms_stereo` module. `undo_ms_stereo(stereo, ms_flags, undo)`
  applies the §2.6 "undo M/S where `msflag` set" reconstruction step
  across a stereo `StereoSubbandMatrix` (`[SubbandMatrix; 2]`): each
  subband whose `ms_flags[b]` is set has its two channels' row `b`
  (mid, side) transformed sample-by-sample into (left, right); L/R
  subbands pass through unchanged.
  - The **exact per-sample channel arithmetic** (whether `L = M + S` /
    `R = M − S`, and any 0.5 / √2 normalisation) is a documented GAP —
    unspecified anywhere under `docs/audio/musepack/` — so it is
    threaded as a caller-supplied `undo(m, s) -> (l, r)` closure (the
    crate's established GAP-knob pattern, cf.
    `sv8_band_header`'s `ctx_for_prev_res`). The closure is the one
    edit that pins the arithmetic once a docs trace lands; the module
    wires the *structure* (per-subband `msflag` gating, channel-row
    pairing, L/R pass-through) without committing to it.
  - `StereoSubbandMatrix` type alias. `Error::MaxBandOutOfRange` for an
    `ms_flags` schedule longer than the 32-subband frame; a shorter
    schedule is allowed (subbands past its end pass through as L/R, per
    the §2.3 "msflag only for coded bands" convention).
  - 8 new unit tests (M/S subband transformed via a test-only
    `L=M+S`/`R=M−S` stand-in, L/R pass-through, selective row gating,
    empty/short/full-width/overlong schedules, per-sample index
    pairing). Crate lib total `386 → 394`.

- **Round 344** — SV7 §2.6 frame-level reconstruction assembler, new
  `frame_reconstruct` module. `reconstruct_frame_channel(bands, anchor)`
  composes the per-band
  `reconstruct::reconstruct_sv7_band_from_levels` over the Layer-II
  32-subband frame geometry (spec §1: 32 subbands × 36 samples = 1152
  per channel), producing the per-channel `SubbandMatrix`
  (`[[f64; 36]; SV7_SUBBAND_COUNT]`) — the structure the remaining §2.6
  steps (M/S undo, then the synthesis filterbank) consume.
  - `SubbandMatrix` type alias + `zero_subband_matrix()` constructor.
  - `BandLevels { subband, band_type, levels: [i32; 36], granule_scf:
    [u32; 3] }` — one decoded band's input; bands absent from the slice
    (uncoded subbands) reconstruct to silence (the §2.3 / §2.5 "data
    stored only for non-zero bands" convention).
  - Fail-loud, never silently-wrong: `Error::MaxBandOutOfRange` for a
    `subband >= 32` or an SCF index outside the `0..SCF_INDEX_COUNT`
    (256) ladder (the §5.3 "clamp to sentinel" note is surfaced as an
    error rather than silently clamped); `Error::UnsupportedBandType`
    propagated for a `band_type` outside `-1..=17`.
  - Pure composition — no new format facts beyond the documented frame
    geometry + the already-grounded per-band reconstruction.
  - 11 new unit tests (zero-matrix silence, empty/absent-band silence,
    single-band agreement with the direct per-band path, multi-band
    row placement + sign preservation, per-granule SCF reaching the
    right 12-sample slice with monotone gain, subband / SCF-ladder /
    band_type rejection, CNS-band row placement, and the 32×36 == 1152
    geometry cross-check). Crate lib total `375 → 386`.

- **Round 344** — SV8 `RG` (ReplayGain) and `EI` (Encoder Info) packet
  payload field-map decoders, two new modules `rg_header` / `ei_header`,
  closing the README "RG / EI packet payload field maps" pick. Both
  layouts are now fully specified in
  `spec/musepack-headers-and-coding.md` §2 (they were GAP when
  `typed_packet`'s `RG`/`EI` newtypes were first wired in round 232).
  - `rg_header::ReplayGainFields::parse(payload)` decodes the §2 `RG`
    layout: 8-bit version (rejected via the new
    `Error::InvalidReplayGainVersion` unless it equals 1), then four
    big-endian 16-bit fields in order — title gain, title peak, album
    gain, album peak — read straight off the natural-order SV8 byte
    stream (§4). Gain/peak values are surfaced verbatim (raw 16-bit);
    no dB/linear rescale is pinned by the staged facts.
    `SV8_REPLAYGAIN_VERSION = 1` / `RG_PAYLOAD_LEN = 9` constants.
  - `ei_header::EncoderInfoFields::parse(payload)` decodes the §2 `EI`
    layout: a packed first byte (high 7 bits = `profile × 8`, low bit =
    PNS noise-substitution flag), then three whole bytes — encoder
    major, minor, build. Helpers `profile()` (fractional, `raw / 8`),
    `profile_int()` (`raw >> 3`), and `version_word()`
    (`(major<<24)|(minor<<16)|(build<<8)` per §2). `EI_PAYLOAD_LEN = 4`.
  - Wired to the typed-packet surface as `ReplayGainPacket::fields()`
    and `EncoderInfoPacket::fields()`, the §3.2 siblings of the
    round-325 `StreamHeaderPacket::fields()`. The two newtypes' doc
    comments are updated to point at the field maps instead of "GAP".
  - New crate-level `Error::InvalidReplayGainVersion(u8)` variant.
  - 14 new unit tests (canonical-payload decode, per-field-order pin
    with distinct values, version rejection, truncation `UnexpectedEof`,
    trailing-byte tolerance, payload-length constants for `RG`; profile
    divide-by-8 + integer truncation, PNS low-bit isolation,
    version-word packing, full-7-bit profile, truncation, trailing
    bytes, length constant for `EI`). Crate lib total `361 → 375`.

- **Round 336** — SV8 §3.4 / §6.4.1 sparse band (`band_type == 1`)
  sample decode in `sv8_sample_decode`, closing the last unimplemented
  SV8 per-band arm and wiring it into the
  `sv8_band_decode::decode_sv8_band` dispatcher. The persistent
  "19-symbol q1 alphabet cannot carry an 18-flag bitmap" DOCS-GAP
  (fail-loud across rounds 245/281/284/288) is resolved by the newly
  staged `spec/musepack-headers-and-coding.md` §6.4.1 + §6.5: the q1
  `0..=18` symbol is the per-group **non-zero count**, not a flag
  bitmap.
  - `decode_sv8_sparse_band(reader, out: &mut [i8; 36])` — decodes a
    band as **two halves of 18** (`SPARSE_GROUP_SIZE`). For each half:
    one `sv8-canonical-q1` codeword gives the non-zero count `cnt`;
    then `decode_sparse_group` reads the §6.5 enumerative codeword
    selecting which `min(cnt, 18 − cnt)` positions are non-zero
    (bit-inverting the mask when `cnt > 9`, since the smaller
    complement is always coded), and one raw sign bit per present
    position sets it to `±1` (`requant-quantizer-offset-Dc` pins
    `D = 1` for `band_type` 1). `cnt == 0` ⇒ all-zero; `cnt == 18` ⇒
    all-present (no enumerative bits).
  - `enum_decode_subset(reader, k, n)` — the §6.5 enumerative
    (combinatorial) coder: a phased-/truncated-binary index read
    (`bitlen − 1` bits, with a conditional extra "lost-codes" bit and
    rebase) over the `C(n, k)`-sized code space, followed by a
    combinadic peel (walk positions high→low, subtract `C(m, k)` when
    the running code admits it). Returns the selected positions as an
    `n`-bit mask. All binomials are computed via a `const fn binomial`
    multiplicative recurrence — no new staged tables
    (`C(18, 9) = 48620` is the largest value reached).
  - `SPARSE_GROUP_SIZE = 18` constant; `enum_bitlen_lost(total)`
    helper (`bitlen = ceil(log2(total))`, `lost = 2^bitlen − total`).
    A malformed q1 count (> 18) yields
    `Error::GroupedSymbolOutOfRange`; EOF in any phase propagates.
  - New unit tests: a full sparse-band round-trip harness (test-side
    phased-binary + combinadic encoder) covering every count `0..=18`,
    single-non-zero halves, the `cnt > 9` complement inversion,
    all-present halves, the all-zero fast path with exact bit
    accounting, malformed-count rejection, EOF propagation, the
    `(bitlen, lost)` / binomial-code-space identity, and the binomial
    recurrence against a reference; plus the dispatcher's sparse arm
    re-tested against the direct decoder (replacing the prior
    fail-loud test). Crate lib total `339 → 348`.

- **Round 329** — SV7 (`MP+`) fixed-header field-map decoder, new
  `sv7_header` module. `Sv7HeaderFields::parse(input)` decodes the spec
  (`spec/musepack-headers-and-coding.md` §1) layout: all 17 fields in
  order (32-bit frame count assembled from two 16-bit halves, the
  intensity / M/S flags, 6-bit `max_band`, 4-bit profile, 2-bit link,
  2-bit sample-freq index, 16-bit max-level, the four 16-bit ReplayGain
  title/album gain+peak fields, true-gapless flag, 11-bit last-frame
  samples, fast-seek flag, 19-bit reserved, 8-bit encoder version). The
  field reader runs over the SV7 32-bit-word byte-swap framing (§4 —
  each aligned 4-byte group reversed; field reads begin at bit 32, after
  the `MP+`+version prefix word) via a local `word_swap_sv7` helper; the
  `MP+` magic is validated on the raw input first. Enforces the §1
  sanity gate `1 ≤ max_band ≤ 31` (reusing `Error::MaxBandOutOfRange`).
  Helpers: `sample_rate_hz()` (index → {44100, 48000, 37800, 32000} Hz),
  `channels()` (stereo-only constant 2), and `total_samples()`
  (`frame_count × 1152`). 9 unit tests. Closes the README "SV7
  fixed-header field map / SV7 32-LSB word packing" gap.
- **Round 325** — SV8 `SH` (Stream Header) packet payload field-map
  decoder, new `sh_header` module. `StreamHeaderFields::parse(payload)`
  decodes the spec (`spec/musepack-headers-and-coding.md` §2) layout:
  the 32-bit CRC (surfaced verbatim, not validated), the version byte
  (rejected via the new `Error::InvalidStreamVersion` unless it equals
  8), the total / beginning-silence sample counts (byte-aligned varints
  via the existing `parse_varint`), and the packed 16-bit tail
  (sample-freq index, the −1-biased `max_band`, the −1-biased
  `channels`, stream M/S, block power) read MSB-first off the
  natural-order SV8 byte stream (§4, no SV7 word-swap). Helpers
  `sample_rate_hz()` (index → {44100, 48000, 37800, 32000} Hz, `None`
  outside the four defined indices) and `frames_per_audio_packet()`
  (`2^(block_power × 2)`, §2 field 9). Wired to the typed-packet
  surface as `StreamHeaderPacket::fields()`. 10 unit tests.
- **Round 320** — SV7 §2.5 unified per-band sample-decode dispatcher
  `decode_sv7_band(reader, band_type, cns, ctx, out)` in
  `sv7_band_decode`, the SV7 sibling of the SV8
  `decode_sv8_band` dispatcher (round 288). It walks the §2.5
  `switch (band_type)` ladder end to end from `band_type` alone,
  routing through the existing `band_type_case` classifier to the
  matching per-arm decoder and unifying every arm on an
  `[i32; 36]` output:
  - CNS (`-1`), empty (`0`), and linear-PCM escape (`8..=17`) arms
    write the unified `[i32; 36]` buffer directly.
  - The grouped (`1` / `2`) and per-sample Huffman (`3..=7`) arms
    decode into a scratch `[i8; 36]` and are widened into the
    buffer via the new `widen_into` helper (loss-free `i8 -> i32`).
  - The single `ctx` context knob is threaded verbatim into the
    table-pair arms; CNS / empty / PCM-escape ignore it; a
    `ctx > 1` reaches the per-arm fail-loud
    `Error::UnsupportedBandType` channel.
  - `band_type` outside `-1..=17` (`BandDecodeCase::OutOfRange`)
    returns `Error::UnsupportedBandType(band_type)` rather than
    silently zeroing the band, matching the SV8 dispatcher's
    fail-loud posture; EOF from any arm propagates unchanged.
  - 10 new dispatcher tests (one per arm matched against its
    direct per-arm decoder, PRNG-state advance on the CNS arm,
    context threading, out-of-range / bad-ctx / EOF fail-loud).
    No DOCS-GAP touched — pure composition of already-grounded
    arms.

- **Round 314** — SV7 §2.5 grouped-codeword sample decode (cases 1
  and 2) in `sv7_band_decode`, closing the two arms earlier rounds
  left as fail-loud `UnsupportedBandType`. The per-codeword fan-out,
  previously flagged DOCS-GAP, is uniquely determined by the same
  staged Feist facts the SV8 grouped round (281) used:
  - `sv7-huffman-q1` carries exactly 27 distinct `value`s spanning
    `0..=26` per context; `requant-quantizer-offset-Dc` pins
    band_type 1 to `D = 1` (3 levels/sample). The only composition
    consistent with "3 samples per codeword" is a **base-3-packed
    triplet** (digit = sample + 1, samples `-1..=1`); the all-zero
    triplet is value `13`, the shortest-code q1 ctx-0 entry.
  - `sv7-huffman-q2` carries exactly 25 `value`s spanning `0..=24`;
    band_type 2 has `D = 2` (5 levels). The unique composition is a
    **base-5-packed pair** (digit = sample + 2, samples `-2..=2`);
    the all-zero pair is value `12`, the shortest-code q2 ctx-0
    entry. The §3.6 lossless SV7↔SV8 relationship corroborates the
    coefficient model.
  - `unpack_grouped3_value(value) -> Result<[i8; 3]>` /
    `unpack_grouped2_value(value) -> Result<[i8; 2]>` — pure unpack
    helpers, defensive `Error::GroupedSymbolOutOfRange` outside the
    `0..=26` / `0..=24` alphabet.
  - `decode_grouped3_band(reader, ctx, out)` (12 q1 codewords) /
    `decode_grouped2_band(reader, ctx, out)` (18 q2 codewords), each
    fanned into the band's 36 samples from the `ctx`-selected
    `[2][N]` table half; out-of-range `ctx` yields
    `Error::UnsupportedBandType(1|2)`.
  - `GROUPED3_CODEWORDS_PER_BAND = 12` /
    `GROUPED2_CODEWORDS_PER_BAND = 18` constants.
  - The one convention the staged values cannot pin — the
    within-group emission order — is taken **least-significant
    digit first** (matching `sv8_sample_decode`) and isolated inside
    the two `unpack_*` helpers for a one-line flip if a future
    observer trace pins the opposite order.
  - 14 new unit tests (base-N hand-vectors, bijection-onto-`(-D..=D)^n`
    proofs, alphabet-bound rejection, all-zero / corner band decodes
    on both context halves, bad-ctx rejection, EOF propagation, and
    the codeword-count tiling pin). Crate lib total `297 → 311`.
- **Round 307** — SV8 §3.5 frame-body DSCF delta-loop walk in a new
  `sv8_dscf_loop` module, the per-band delta-scalefactor read that runs
  after the round-301 SCFI selector, feeding off the staged
  `sv8-canonical-dscf-{1,2}` context pair:
  - `decode_dscf_deltas` walks a caller-supplied `deltas_per_band`
    slice, reading that many `sv8-canonical-dscf-{1,2}` canonical-Huffman
    codewords per band in ascending band order, returning one inner
    `Vec<RawDscfVlc>` per band.
  - Three §3.5 conventions stay DOCS-GAP and are threaded as caller
    knobs: the per-band delta count (1..=3, GAP because the SV8 SCFI
    value → count table is GAP — `scfi-2` spans `0..=15`); the
    `dscf-1`/`dscf-2` context-selection rule (caller `ctx_for_prev_dscf`
    closure + `initial_ctx`, context carrying across band boundaries; an
    out-of-range context yields `Error::UnsupportedBandType(i8::MIN)`);
    and the DSCF symbol → signed-delta centring offset (the `dscf-{1,2}`
    maps are unsigned `0..=63` / `0..=64`, unlike SV7's directly-signed
    `-7..=8`), kept honest by the `RawDscfVlc` newtype (the SV8 DSCF
    sibling of `RawScfiVlc` / `RawResVlc`).
  - 14 unit tests (crate lib total `283 → 297`).
- **Round 301** — SV8 §3.5 frame-body SCFI-selector header walk in a
  new `sv8_scf_header` module, the per-non-zero-band SCF-coding-method
  selector read that precedes the (still-GAP) DSCF delta walk, feeding
  off the staged `sv8-canonical-scfi-{1,2}` context pair:
  - `decode_scfi_selectors` walks `nbands` coded bands, reading one
    `sv8-canonical-scfi-{1,2}` canonical-Huffman codeword each in
    ascending band order. The `scfi-1`/`scfi-2` context-pair selection
    rule is a §3.5 DOCS-GAP, threaded as a caller-supplied
    `ctx_for_prev_scfi` closure + `initial_ctx` (the same caller-knob
    precedent `sv8_band_header::decode_band_resolutions` uses); an
    out-of-range context yields `Error::UnsupportedBandType(i8::MIN)`
    (the reserved `CONTEXT_FAULT_SENTINEL`, distinct from any genuine
    band_type).
  - Each raw SCFI value is wrapped in `RawScfiVlc` (the SV8 SCFI sibling
    of `sv8_band_header::RawResVlc`) so the GAP SCFI-value → granule
    schedule semantics cannot be applied accidentally — the staged
    `scfi-2` symbol map spans `0..=15`, which does **not** match the
    four-value SV7 §2.4 SCFI schedule (`scf::ScfCodingMethod`)
    cell-for-cell. The wrapper blocks feeding a raw value straight into
    `ScfCodingMethod::from_raw` (which would reject every value `>3`).
  - The SV8 SCFI-value → (count, granule-mapping) schedule, the
    context-selection predicate, and the SV8 DSCF symbol → signed-delta
    centring offset (`dscf-{1,2}` maps span an unsigned `0..=63`, unlike
    SV7's directly-signed `-7..=8`) all stay DOCS-GAP and are documented
    as such in the module header.
  - 10 new unit tests (crate lib total `273 → 283`): single-band decode
    against each context half, two-band context switching with the
    closure observed, zero-bands no-op, both context-fault paths
    (out-of-range initial ctx + closure-returned ctx), EOF propagation,
    the raw-newtype round-trip, and the sentinel-domain pin.
- **Round 294** — SV8 §3.4 frame-body band-resolution header walk in
  a new `sv8_band_header` module, the outer loop that feeds the
  round-288 `sv8_band_decode::decode_sv8_band` per-band dispatcher:
  - `decode_used_subbands` reads one `sv8-canonical-bands` canonical-
    Huffman codeword into a used-subbands count in
    `0..=SV8_MAX_USED_SUBBANDS` (32, the §1 Layer-II subband bound),
    rejecting an out-of-range count defensively.
  - `decode_band_resolutions` walks `nbands` bands, reading one
    `sv8-canonical-res-{1,2}` codeword each in ascending band order.
    The `res-1`/`res-2` context-pair selection rule is a §3.4
    DOCS-GAP, threaded as a caller-supplied `ctx_for_prev_res`
    closure + `initial_ctx` (the same caller-knob precedent the
    per-sample context arm `decode_sv8_context_band` uses); an
    out-of-range context yields `Error::UnsupportedBandType(i8::MIN)`
    (a reserved sentinel distinct from any genuine band_type).
  - Each raw res value is wrapped in `RawResVlc` (the SV8 sibling of
    `sv7_band_header::RawBandTypeVlc`) so the GAP `res`-symbol
    (`0..=16`) → §3.4 `band_type` (`-1..=17`) remap cannot be applied
    accidentally — only an explicit, not-yet-specified remap step may
    consume the raw value.
  - Per-channel ordering, the `res`→`band_type` remap, and whether
    the count is clamped by an SH-packet `max_band` field all stay
    DOCS-GAP and are documented as such in the module header.
  - 14 new unit tests (crate lib total `259 → 273`): per-row count /
    resolution decode against every staged `bands` / `res` codeword,
    context switching driven by the previous res, both context-fault
    paths, EOF propagation, and the raw-alphabet-confinement pin.
- **Round 288** — SV8 §3.4 classifier-driven band dispatcher
  `sv8_band_decode::decode_sv8_band`: the single entry point that
  routes one band through the round-245 `sv8_band_type_case`
  classifier to its matching per-arm decoder, composing the grounded
  arms that earlier rounds wired one at a time.
  - CNS (`-1`) / empty (`0`) reuse the SV7-shared `fill_cns_band` /
    `fill_zero_band`; grouped (`2`, `3..=4`) and context (`5..=8`)
    call `sv8_sample_decode`'s decoders with their `[i8; 36]` output
    loss-free-widened; escape (`9..=17`) passes
    `decode_sv8_escape_band`'s native `[i32; 36]` through. All arms
    unify on an `[i32; 36]` output.
  - Context knobs the staged tables do not pin (`grouped_ctx`,
    `initial_ctx` + `ctx_for_prev`) are threaded through verbatim as
    caller knobs.
  - Fail-loud, never silently-wrong: the sparse band (case 1) stays a
    DOCS-GAP (the staged `sv8-symbols-q1` 19-symbol alphabet cannot
    carry an 18-flag bitmap) and the `OutOfRange` catch-all
    (`band_type < -1`) both return `Error::UnsupportedBandType`.
  - 10 new unit tests (crate lib total `250 → 259`): each routed arm
    vs its direct per-arm decoder as oracle, the `grouped_ctx` knob
    reaching only the case-2 table-half, and both fail-loud arms.
- **Round 284** — SV8 §3.4 large-coefficient escape (`default` arm,
  `band_type` 9..=17) in `sv8_sample_decode`, closing the arm round
  281 had left as unpinned. The "fixed number of raw bits" is
  derivable from three already-staged facts: the `sv8-symbols-q9up`
  map is an exact permutation of `-128..=127` (the full signed-byte
  alphabet, one table for the whole "9-and-up" range);
  `requant-res-bits.meta` scopes its ladder to "SV7 §2.5 / SV8
  §3.4", making an escape sample `band_type - 1` bits wide in
  total; and `requant-quantizer-offset-Dc` pins the level range
  `±D` with `D = 2^(band_type-2) - 1`. The unique consistent
  composition: the VLC symbol carries the sign-bearing top 8 bits,
  `n = band_type - 9` raw bits the low bits —
  `sample = (symbol << n) | raw` tiles exactly the
  `(band_type-1)`-bit two's-complement range `[-(D+1), D]`.
  - `escape_raw_bits(band_type) -> Option<u8>` — the
    `RES_BITS[band_type] - 8` ladder (`None` outside `9..=17`,
    where the staged requant tables define no quantiser) +
    `ESCAPE_VLC_SYMBOL_BITS = 8`.
  - `decode_sv8_escape_band(reader, band_type, out: &mut [i32; 36])`
    — 36 × (q9up codeword + MSB-first raw field), emitting
    already-centred `i32` levels (the staged map is signed; the
    SV7 escape by contrast emits uncentred levels). The raw-field
    read mirrors the SV7 §2.5 escape's `read_bits` convention,
    backed by the §3.6 lossless SV7↔SV8 relationship.
  - 8 new unit tests (crate lib total `242 → 250`): signed-byte
    alphabet pin, raw-bit ladder vs `RES_BITS`, `[-(D+1), D]`
    range tiling vs `QUANTIZER_OFFSET_D` for all nine escape
    band_types, pure-VLC band_type 9 bit accounting, high/low
    composition under alternating ± codewords (band_type 13),
    MSB-first widest raw field (band_type 17), band_type
    rejection, EOF at the codeword and inside the raw field.
  - The §3.4 sparse band (case 1) remains the one DOCS-GAP arm
    (19-symbol q1 alphabet vs the prose's "flags for 18 samples";
    newly grounded: `D = 1` for band_type 1, so sparse samples are
    `{-1, 0, +1}` levels).

- **Round 281** — new `sv8_sample_decode` module: SV8 §3.4 per-case
  sample decoders for the grounded subset of the eight-variant
  ladder, composing the round-278 canonical-Huffman decoder walk
  with grouped fan-out facts pinned by the staged
  `sv8-symbols-*.{csv,meta}` material:
  - `unpack_grouped3_symbol(symbol) -> Result<[i8; 3]>` — case-2
    grouped unpack. The staged `sv8-symbols-q2-{1,2}` maps are
    exact permutations of `0..=124` ("5x5x5 grouped" per the
    `.meta` `spec_role`) whose most-probable symbol is the all-zero
    triplet `62 = 2·25 + 2·5 + 2`, so a symbol is a base-5-packed
    triplet of already-centred samples in `-2..=2` (digit = sample
    + 2; 5 levels = `2D+1` with `D = 2` per the §2.6 requant
    relation).
  - `unpack_grouped2_symbol(symbol, band_type) -> Result<[i8; 2]>`
    — case-3/4 grouped unpack. The 49-entry q3 map ("7x7 grouped")
    is an exact bijection onto `(-3..=3)²` and the first 81 q4
    entries ("9x9 grouped, padded") onto `(-4..=4)²` under signed
    two's-complement **nibble-pair** splitting of the int8 symbol
    (q4's 10 padding slots stay zero and unreachable per the
    round-278 tiling proof); `band_type` doubles as the per-nibble
    bound `D`.
  - `decode_sv8_grouped3_band(reader, ctx, out)` — case 2: 12
    codewords from the `sv8-canonical-q2-{1,2}` pair, each fanned
    into 3 consecutive samples. The q2 pair-selection rule is GAP
    (case 2 sits outside the §3.4 `5..=8` context range), so `ctx`
    is a caller knob per the `PacketSizeConvention` precedent.
  - `decode_sv8_grouped2_band(reader, band_type, out)` — cases
    3..=4: 18 codewords from `sv8-canonical-q3` / `-q4`, each
    fanned into 2 consecutive samples.
  - `decode_sv8_context_band(reader, band_type, initial_ctx,
    ctx_for_prev, out)` — cases 5..=8: one VLC per sample from the
    `sv8-canonical-q{5..8}-{1,2}` pair; every staged q5..q8 map is
    a permutation of `-D..=D` (`D = 7/15/31/63`) so the symbol IS
    the centred level. §3.4 pins that the table is "chosen by the
    previously decoded sample" but not the predicate, which is
    taken as a caller-supplied closure.
  - `GROUPED3_CODEWORDS_PER_BAND = 12` /
    `GROUPED2_CODEWORDS_PER_BAND = 18` constants and a new
    crate-level `Error::GroupedSymbolOutOfRange(i8)` defensive
    variant.
  The within-group emission order (which radix digit / nibble is
  the first of the consecutive samples) is the one convention the
  staged material cannot pin (both assignments are bijections);
  the module emits least-significant-digit / low-nibble first,
  isolated inside the two `unpack_*` helpers for a one-line flip
  if a future observer trace pins the opposite order. The sparse
  band (case 1 — the staged 19-symbol `0..=18` q1 alphabet cannot
  literally carry the prose's "flags for 18 samples") and the
  large-coefficient escape (default arm — the "fixed number of raw
  bits" is unpinned) remain DOCS-GAP and fail loudly. 16 new unit
  tests (staged-fact pins for the q2/q3/q4/q5..q8 map structures,
  unpack hand-vectors + bijection + rejection, per-case band
  decodes with exact bit accounting, codeword-order and
  context-switching traces, ctx/band_type rejection, EOF
  propagation, classifier-arm composition). Crate lib test count
  `219 → 242`.

- **Round 278** — `sv8_huffman` module gains the SV8 §3.4
  canonical-Huffman **decoder walk**, closing the round-260
  cumulative-index DOCS-GAP from the staged facts alone. The
  round-260 ambiguity between two candidate per-row sub-index
  assignments is resolved by an exhaustive tiling argument: a
  complete prefix code paired with an `N`-entry symbol map must
  map the 2^16 peek windows bijectively onto indices `0..N`, and
  checking both candidates over all 65536 peeks for all 21 staged
  tables shows only one satisfies it —
  `index = (cum_index − (peek16 >> (16 − length))) mod 256`
  against the first row (code descending) with `code <= peek16`
  (the alternative leaves holes, e.g. `bands` index 3 and `q1`
  indices 0/1/18 unreachable). New API:
  - `Sv8CanonicalTable::decode_symbol_index(reader)` — the walk:
    16-bit MSB-first peek, descending-code row match (length-0
    rows, i.e. the staged q4 sentinel, are skipped), mod-256
    cumulative fold, `length`-bit consume; defensive
    `Error::HuffmanNoMatch` on an out-of-map index.
  - `Sv8CanonicalTable::decode(reader) -> Result<i8>` — index walk
    plus paired symbol-map lookup; SV8 sibling of the SV7
    `huffman::decode` entry point.
  The mod-256 fold is exact for `q9up`'s signed-int8 cumulative
  wrap and the identity for the 20 unsigned-cum tables; `q4`'s
  rows tile indices `0..=80`, with map entries `81..=90` proven
  unreachable zero padding. 7 new unit tests: the per-table
  2^16-peek tiling proof (each reachable index hit exactly
  `2^(16−length)` times), hand-traced `q1` vectors across four
  length classes, `q9up` signed-wrap vectors, back-to-back decode
  chaining with exact bit consumption, the q4 sentinel-skip path,
  the q4 padding pin, and 16-bit-peek EOF propagation. Crate lib
  test count `212 → 219`. Downstream §3.4 per-case sample decoders
  (sparse-band flags, grouped unpack, first-order context
  selection, escape raw bits) are now unblocked on the entropy
  side; the grouped-codeword fan-out arithmetic remains GAP in the
  structural prose.

- **Round 272** — `reconstruct` module gains the §2.6 *relative*
  scalefactor (SCF) gain ladder, reading only
  `docs/audio/musepack/tables/scf-step-ratio.meta` (the geometric
  step-ratio fact + the "256 indices" dimension) plus the staged
  spec §2.6. The *absolute* anchored SCF gain table stays GAP (its
  reference-index gain is unspecified in the structural prose), but
  the geometric relation between any two indices is anchor-
  independent and therefore fully determined:
  - `SCF_INDEX_COUNT: usize = 256` — the SCF index ladder size,
    pinned by the `scf-step-ratio.meta` "256 indices" note.
  - `scf_relative_gain(from: u8, to: u8) -> f64` — infallible
    `SCF_STEP_RATIO^(to − from)`; the multiplicative gain at index
    `to` relative to index `from`. Identity at `from == to`; a
    higher index is quieter (`< 1.0`), a lower index louder
    (`> 1.0`).
  - `scf_gain_relative_to_anchor(anchor, &mut [f64; 256])` — fills
    the full 256-entry gain ladder relative to `anchor` (unity at
    `anchor`, strictly decreasing in index).
  - `apply_scf_relative(from_index, to_index, &mut [f64; 36])` —
    scales a dequantised band in place by the relative SCF gain,
    applying a per-granule SCF index *difference* off a shared base
    without needing the GAP absolute anchor (result correct up to
    one global constant scale).
  13 new unit tests cover: the 256-index dimension; identity at
  equal indices; one-step-up == `SCF_STEP_RATIO`; one-step-down ==
  reciprocal; inverse symmetry `g(a,b)·g(b,a)==1`; exponent
  additivity `g(a,c)==g(a,b)·g(b,c)`; `n`-step == `ratio^n`;
  anchor-unity + per-entry agreement of the ladder; ladder strict
  monotonic decrease; `apply_scf_relative` per-sample scaling,
  identity no-op, and inverse round trip. Crate test count
  `199 → 212` (lib).

- **Round 260** — `sv8_huffman` module wiring the 21 staged SV8
  canonical Huffman length-tables and 21 paired int8 symbol maps
  from `docs/audio/musepack/tables/` into typed Rust statics,
  reading only the staged spec
  (`musepack-sv7-sv8-spec.md` §3.4 / §4) and provenance
  (`provenance/01-musepack-table-extraction.md` §6):
  - `Sv8CanonicalEntry { code: u16, length: u8, cum_index: i16 }`
    row of an SV8 canonical length table. `cum_index` widens to
    `i16` to accommodate the `q9up` large-coefficient escape
    map's signed-int8 cumulative-index wrap.
  - `Sv8CanonicalTable { lengths, symbols, name }` paired
    (length-table, symbol-map) view carrying the staged CSV stem
    as a diagnostic `name` field.
  - 21 catalogue tables wired as `pub static` arrays under
    identifier prefixes `SV8_BANDS`, `SV8_RES_{1,2}`,
    `SV8_SCFI_{1,2}`, `SV8_DSCF_{1,2}`, `SV8_Q1`,
    `SV8_Q2_{1,2}`, `SV8_Q3`, `SV8_Q4`, `SV8_Q5_{1,2}` ..
    `SV8_Q8_{1,2}`, and `SV8_Q9UP`. `SV8_CANONICAL_CATALOGUE`
    exposes the 21 tables as a single ordered slice.
  - `Sv8TableRole` enum + `table_for_role(role, ctx) ->
    Option<&Sv8CanonicalTable>` dispatcher mapping a §3.4 / §3.5
    spec role plus a first-order context bit (0 or 1) into the
    matching physical table. Context-pair roles return `None`
    for `ctx >= 2`; non-pair roles ignore `ctx`.
- **Round 260** — `build.rs` grows a third emitter
  (`emit_sv8_canonical_tables`) parsing each
  `sv8-canonical-<stem>.csv` length-table and its companion
  `sv8-symbols-<stem>.csv` symbol map, emitting a
  `Sv8CanonicalEntry` array + paired `[i8; N]` symbol array per
  pair plus a `Sv8CanonicalTable` paired-view static.
- **Round 260** — vendored snapshot of the 21 staged
  `sv8-canonical-*.csv` + `sv8-symbols-*.csv` pairs (with `.meta`
  sidecars, 84 files total) committed under `<crate>/tables/`
  so the crate stays buildable standalone for crates.io / CI
  consumers.
- **Round 260** — 24 new unit tests in `sv8_huffman::tests`
  covering: catalogue shape (21 entries, unique
  `sv8-canonical-`-prefixed names); per-table row counts against
  every staged `.meta` `resolved_dims` value; `bands` first/last
  row equality + symbol-map endpoints; data-row code descending +
  length non-decreasing + length within `1..=16` + zero-low-bits
  left-justification invariant + terminating at code `0x0000`;
  cumulative-index progression (strict increase for the 20
  unsigned-cum tables, modular-256 progression for `q9up`); the
  `q4` length-0 sentinel row pin (only catalogue entry with such
  a sentinel); `min_length` / `max_length` helpers;
  `table_for_role` context-ignore for non-pair tables and
  context-pair dispatch for all 8 pair roles; `table_for_role`
  rejecting `ctx >= 2`; the catalogue `name` field carrying the
  CSV stem; `Sv8CanonicalTable::{len_table_rows, sym_table_rows}`
  helpers; `bands` symbol map spanning `0..=32`; `q9up`
  symbol-map endpoints (`-128, ..., -2`). Total crate test count
  `176 → 200`.

### DOCS-GAP — still standing

- **§3.4 SV8 canonical-Huffman cumulative-index decoder walk**.
  The structural spec names the layer and pins the row layout
  but does not pin the arithmetic that maps a peeked 16-bit code
  window to a symbol index against the `cum_index` column.
  Kraft-McMillan rules out the naive "one row covers
  `2^(16 - Length)` peek bins" reading: staged tables routinely
  skip intermediate lengths (e.g. `q1` rows go 3, 4, 5, 6, … with
  the length-3 row's `cum_index = 7` exceeding the 5 length-3
  peek-bins). Two plausible sub-index interpretations
  (forward-ascending vs descending-from-cum) give incompatible
  symbol mappings; the choice is not derivable from values alone.
  Recommend a §3.4 docs patch that pins the cumulative-index
  walk arithmetic; the typed-table surface this round wires is
  ready to consume it.

- **Round 245** — `sv8_band_decode` module wiring the SV8 §3.4
  per-band sample-decode case classifier, reading only
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.4:
  - `Sv8BandDecodeCase` enum routing each `band_type` to its §3.4
    `switch` arm (`Cns`, `Empty`, `SparseBand`, `Grouped3`,
    `Grouped2`, `ContextHuffmanPerSample`, `LargeCoeffEscape`,
    `OutOfRange`). Mirrors the SV7 sibling
    [`sv7_band_decode::BandDecodeCase`] shape with the SV8-specific
    ladder differences (sparse-band insertion at `case 1`, grouped
    cases shifted up by one, first-order context arm at
    `case 5..=8`, large-coefficient-escape `default` arm at
    `band_type >= 9`).
  - `sv8_band_type_case(band_type: i8) -> Sv8BandDecodeCase` —
    pure `const fn` dispatch, total over the full `i8` range.
  - `case_emits_samples(case)` predicate isolating the §3.4 outer
    loop's "for each non-zero band" arms (every variant except
    `Empty` / `OutOfRange`).
  - `case_uses_first_order_context(case)` predicate isolating the
    SV8-specific `case 5..=8` first-order context-modelled per-
    sample Huffman path — the "table chosen by the previously
    decoded sample" highlight per §3.4 prose.
- **Round 245** — 16 new unit tests across `sv8_band_decode::tests`
  covering: classification of `band_type == -1` (Cns), `0` (Empty),
  `1` (SparseBand), `2` (Grouped3), `3` / `4` (Grouped2), `5..=8`
  (ContextHuffmanPerSample), `9..=64` and `i8::MAX`
  (LargeCoeffEscape), and `-2 / -10 / -100 / i8::MIN` (OutOfRange);
  full-`i8`-range totality of the classifier; the
  `case_emits_samples` truth table per §3.4 arm; the
  `case_uses_first_order_context` truth table; the band_type-range
  cross-check of `case_uses_first_order_context` against
  `5..=8`; const-evaluation sanity at five representative band
  types; the SV7-vs-SV8 ladder divergence on the grouped-case
  indices (SV7 case 1 = Grouped3, SV8 case 1 = SparseBand; SV7
  case 2 = Grouped2, SV8 case 2 = Grouped3; SV7 case 3 =
  HuffmanPerSample, SV8 case 3 = Grouped2; SV7 case 5 =
  HuffmanPerSample, SV8 case 5 = ContextHuffmanPerSample); the
  shared-arm agreement on `case -1` (Cns) and `case 0` (Empty);
  and the `Copy` / `Eq` / `Debug` invariants. Total crate test
  count `160 → 176`.

- **Round 239** — `stream_shape` module wiring an SV8 stream-shape
  observer on top of the round-228 `PacketStream` walker and the
  round-232 `TypedPacket` classifier, reading only
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 + §3.2:
  - `StreamShape` — structural summary carrying per-§3.2-kind
    counters (`sh_count` / `rg_count` / `ei_count` / `so_count` /
    `st_count` / `ap_count` / `se_count` / `unknown_count`),
    cumulative opaque payload bytes (`total_payload_bytes`), and
    `first_kind` / `last_kind` classified packet keys.
  - `scan_sv8_stream(input, convention) -> Result<StreamShape>` —
    pure observer entry point. Validates `MPCK` magic, drives a
    `PacketStream` with the caller-supplied `PacketSizeConvention`
    pick (still-GAP varint convention), classifies each emitted
    `PacketRef` via `TypedPacket::classify`, and accumulates the
    shape. No payload field interpretation; no ordering check.
  - `StreamShape::total_packets()` / `is_empty()` /
    `saw_stream_end()` / `count_for(PacketKey)` accessors plus
    `Default` / `Copy` / `Eq`.
- **Round 239** — 15 new unit tests in `stream_shape::tests`
  covering: rejection of non-`MPCK` magic + short input; empty
  post-magic slice yielding the all-zero shape; single-`SE`
  terminator path; full §3.2 vocabulary walk with correct first /
  last kinds and payload-byte tally; repeated-`AP` aggregation;
  multiple unknown 2-byte keys aggregated into `unknown_count`
  while `first_kind` / `last_kind` preserve the raw bytes / known
  kind; `count_for` routing for every §3.2 key and the `Unknown`
  catch-all; truncated-payload `UnexpectedEof` propagation;
  trailing-garbage-after-`SE` ignored by the walker; `SE`-less
  stream still reporting `first_kind` / `last_kind` for what was
  seen; `total_payload_bytes` payload-only (header bytes excluded);
  inclusive-convention scan with `raw_size = header_len +
  payload_len`; default shape all-zero / empty; `Copy` / `Eq`
  invariants on `StreamShape`; and `first_kind` first-wins lock-in
  (a later packet must not overwrite it). Total crate test count
  `145 → 160`.

- **Round 232** — `typed_packet` module wiring a typed §3.2 packet
  surface on top of the round-228 `PacketStream` walker, reading
  only `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 + §3.2:
  - Per-kind borrowed newtypes covering the full §3.2 vocabulary —
    `StreamHeaderPacket<'a>` (`SH`), `ReplayGainPacket<'a>` (`RG`),
    `EncoderInfoPacket<'a>` (`EI`),
    `SeekTableOffsetPacket<'a>` (`SO`), `SeekTablePacket<'a>` (`ST`),
    `AudioPacket<'a>` (`AP`), `StreamEndPacket<'a>` (`SE`). Each
    newtype carries the opaque payload slice the upstream walker
    emitted and exposes a single `payload_bytes() -> &'a [u8]`
    accessor; per-field maps remain GAP per the §3.2 "Field layout"
    column.
  - `TypedPacket<'a>` sum routing each known key to its per-kind
    newtype plus an `Unknown { key: [u8; 2], payload: &'a [u8] }`
    catch-all that preserves the raw bytes of any 2-byte key
    outside the §3.2 vocabulary (forward-compat for the pending
    observer-trace round).
  - `TypedPacket::classify(PacketRef<'a>) -> TypedPacket<'a>` — pure
    infallible classification of one walker-surfaced packet.
  - `TypedPacket::key()` / `payload_bytes()` accessors plus
    `is_stream_end()` / `is_audio()` / `is_metadata()` predicates
    for log / filter loops without re-matching every variant. The
    three predicates are mutually exclusive (and all `false` for
    `Unknown`).
- **Round 232** — 10 new unit tests across `typed_packet::tests`
  covering: routing of every known §3.2 key into the matching
  typed variant + the three-predicate truth table; `Unknown`
  preservation of raw key bytes and payload; a seven-packet
  end-to-end walk (`SH` + `RG` + `EI` + `SO` + `ST` + `AP` + `SE`)
  through `PacketStream::next_packet` + `TypedPacket::classify`;
  metadata-only filter counting on a mixed `SH` / `AP` / `RG` /
  `AP` / `SE` stream; the `payload_bytes()` accessor agreeing
  with the inner newtype's accessor across every variant
  (including `Unknown`); empty-payload round-trip across every
  variant; an unknown 2-byte key (`ZQ`) traversing the walker into
  `TypedPacket::Unknown` without an error; the `Copy` / `Eq`
  invariants on both `TypedPacket` and its inner newtypes;
  classification independence from `PacketHeader::raw_size` and
  `header_len` (only key + payload are consulted); and the
  mutual-exclusion property of `is_metadata` / `is_audio` /
  `is_stream_end`. Total crate test count `135 → 145`.

- **Round 228** — `packet_stream` module wiring an SV8 packet-stream
  walker on top of the round-194 outer-frame parser, reading only
  `docs/audio/musepack/musepack-sv7-sv8-spec.md` §3.1 + §3.2:
  - `PacketSizeConvention { Inclusive, Exclusive }` — explicit pick
    for the still-GAP §3.1 varint convention. `Inclusive` reads
    the literal "total packet length (key + size + payload)"
    sentence; `Exclusive` treats `raw_size` as the payload byte
    count alone. The pending observer-trace round (#1263) is
    expected to pin one as correct.
  - `PacketRef<'a> { key, header, payload: &'a [u8] }` — one
    decoded packet. `payload` is borrowed from the underlying
    slice; no per-packet allocation.
  - `PacketStream<'a>` — walker built from the post-`MPCK`-magic
    slice plus a `PacketSizeConvention`. `next_packet() ->
    Result<Option<PacketRef<'a>>>` yields one packet per call,
    returns `Ok(None)` after the §3.2 `SE` terminator (or an empty
    input), propagates `UnexpectedEof` / `VarintTooLong` errors
    on malformed input, and locks into a stopped state so
    post-error / post-`SE` calls quietly return `Ok(None)`.
  - `remaining_bytes()` / `is_stopped()` / `convention()`
    accessors.
- **Round 228** — crate-root `SAMPLES_PER_FRAME_PER_CHANNEL: usize
  = 1152` constant pinning the Layer-II 32-subband × 36-sample
  frame geometry per spec §1 lines 65-71. The constant is
  cross-checked at the lib-tests layer against
  `SV7_SUBBAND_COUNT * SAMPLES_PER_BAND`.
- **Round 228** — 15 new unit tests in `packet_stream::tests`
  covering: `Inclusive` / `Exclusive` accessor round-trip; empty
  input yielding `Ok(None)` and stopping; single-`SE` terminator
  path; a three-packet stereo walk (`SH` + `AP` + `SE`); stop-at-
  `SE` with trailing garbage in the buffer (the walker leaves the
  stream exhausted without erroring on the leftover bytes); the
  inclusive convention with `raw_size` covering the 3-byte header;
  inclusive-mode rejection of a sub-header `raw_size` with
  `Error::VarintTooLong`; `UnexpectedEof` propagation on a
  declared-but-truncated payload; malformed-header
  `UnexpectedEof`; the `remaining_bytes()` cursor shrinking on
  each successful read; forward-compat surfacing of unknown 2-byte
  keys via `PacketKey::Unknown`; full-walk count of a five-packet
  synthetic stream; the post-error stopped-state invariant; and
  the lifetime guarantee that `PacketRef::payload` borrows from
  the input slice rather than copying. Plus one new crate-root
  test pinning `SAMPLES_PER_FRAME_PER_CHANNEL` against the
  `SV7_SUBBAND_COUNT * SAMPLES_PER_BAND` invariant. Total crate
  test count `120 → 135`.

- **Round 223** — `sv7_band_header` module wiring the SV7 §2.3
  per-band header loop on top of the round-197
  `SV7_BANDTYPE_HEADER_TABLE` Huffman table and the `Sv7BitReader`,
  reading only `docs/audio/musepack/`:
  - `SV7_SUBBAND_COUNT = 32` + `SV7_MAX_BAND_INCLUSIVE = 31`
    constants pinning the Layer-II 32-subband geometry inherited
    per spec §1 lines 53-71.
  - `RawBandTypeVlc(i8)` typed wrapper around the raw `i8` value
    produced by one invocation of the `sv7-huffman-bandtype-header`
    VLC. `from_raw(i8)` / `as_i8()` / `is_nonzero()` are the only
    accessors; the type intentionally does not expose arithmetic
    so the still-DOCS-GAP §2.3-VLC-symbol → §2.5-dispatcher-case
    remap cannot be implicitly composed with the
    `sv7_band_decode` dispatchers.
  - `BandHeader { band_type: [RawBandTypeVlc; 2], ms_flag:
    Option<bool> }` — one entry per band. `ms_flag` is `None`
    when §2.3's conditional suppressed the flag (both channels'
    `band_type == 0`) and `Some(true|false)` otherwise (true =
    M/S, false = L/R). `has_samples()` short-cuts the §2.5 inner
    loop's "for non-zero bands" predicate.
  - `decode_band_header(reader, nch) -> Result<BandHeader>` —
    one band's read: `nch` (1 or 2) bandtype-header VLCs followed
    by the conditional 1-bit `msflag`. Mono is treated as the
    single channel's VLC occupying both slots so the predicate
    fires the same way.
  - `decode_header_loop(reader, max_band, nch) ->
    Result<Vec<BandHeader>>` — the full §2.3 outer loop walking
    `i = 0..=max_band`, returning `max_band as usize + 1`
    entries.
  - New crate-level `Error::MaxBandOutOfRange(u8)` variant
    rejecting `max_band > 31`, and
    `Error::ChannelCountInvalid(u8)` variant rejecting `nch`
    outside `{1, 2}`.
- **Round 223** — 19 new unit tests across `sv7_band_header::tests`
  covering: the Layer-II 32-subband geometry constants;
  `RawBandTypeVlc` round-trip through `from_raw` + `is_nonzero`
  across `-5..=4`; `BandHeader::has_samples` across the four
  channel-zero-pattern combinations; `decode_band_header` for
  stereo both-zero (no msflag), stereo left-non-zero (msflag=1
  read), stereo right-non-zero (msflag=0 read), mono both-zero
  (no msflag), and mono non-zero (msflag consumed); rejection of
  `nch` outside `{1, 2}` (`0`, `3`, `8`, `255`);
  `UnexpectedEof` propagation in the left-VLC phase and the
  msflag phase; `decode_header_loop` rejection of `max_band > 31`
  (`32`, `200`); rejection of `nch` outside `{1, 2}` in the loop
  entry point; `max_band == 0` returning a single-band vector;
  a three-band stereo walk covering all three msflag outcomes;
  the maximally-wide stereo frame (`max_band == 31` → 32 bands,
  64 bits all-zero); and mid-loop `UnexpectedEof` propagation.
  Total crate test count `101 → 120`.

- **Round 214** — `scf` module wiring the SV7 §2.4 SCF
  coding-method decoder on top of the round-197 SCFI / DSCF
  Huffman tables, reading only `docs/audio/musepack/`:
  - `SCF_GRANULES_PER_BAND = 3` and `SCF_MAX_DISTINCT = 3`
    constants pinning the Layer-II-inherited band geometry
    (1152 samples / 32 bands / 12-sample granules; spec §1
    lines 65-71).
  - `ScfCodingMethod` typed wrapper around the `0..=3` SCFI
    selector value, with `from_raw(i8)` validating that the
    decoded SCFI VLC value is in range and
    `Error::InvalidScfCodingMethod(i8)` carrying the offending
    value on rejection.
  - `GranuleSchedule` exposing `deltas_to_read()` (number of
    distinct DSCFs the band transmits, `1..=3`) and
    `granule_to_slot()` (the granule → stored-delta-index
    mapping). The four schedules are transcribed verbatim from
    §1 lines 79-82 (the Layer-II SCFSI convention restated in
    the Musepack structural spec):
    - method 0: 3 SCFs, mapping `[0, 1, 2]`.
    - method 1: 2 SCFs, mapping `[0, 0, 1]`.
    - method 2: 1 SCF, mapping `[0, 0, 0]`.
    - method 3: 2 SCFs, mapping `[0, 1, 1]`.
  - `reconstruct_scf_from_deltas(reader, base, schedule) ->
    Result<[i32; 3]>` — reads `schedule.deltas_to_read()` DSCF
    VLCs, runs the §2.4 "delta-coded against the previous one"
    accumulation against the caller-supplied `base` anchor,
    and projects the running transmitted values through the
    granule mapping into the three per-granule SCF indices.
  - `decode_band_scf(reader, base) -> Result<BandScf>` — full
    per-band entry point: reads the SCFI VLC, classifies it,
    then drives the DSCF loop. Returns the recovered
    `ScfCodingMethod` alongside the three SCF indices.
  - New crate-level `Error::InvalidScfCodingMethod(i8)`
    variant. The base anchor is sourced upstream (SV7
    fixed-header field map, currently GAP per §2.1) and is not
    decided by this module.
- **Round 214** — 16 new unit tests across `scf::tests`
  covering: SCFI `0..=3` round-trip through `from_raw`;
  rejection of `-1`, `-2`, `4`, `5`, `7`, `i8::MAX`, `i8::MIN`;
  the four granule-schedule shapes against their §1 lines
  79-82 source rows; delta reconstruction for methods 0, 1,
  2, 3 driven through the real `Sv7BitReader` against
  hand-packed DSCF bit streams (covering single-shared,
  three-distinct running-sum, two-pair-mappings); end-to-end
  `decode_band_scf` for method 2 against a 16-bit packed
  stream; `UnexpectedEof` propagation in both the SCFI and
  the DSCF phase; the geometric invariant that every
  schedule's mapping references only valid transmitted slots;
  the constant `SCF_GRANULES_PER_BAND == 3`; and a zero-base
  reconstruction reducing the function to a pure running-sum
  walker. Total crate test count `85 → 101`.
- **Round 206** — `reconstruct` module wiring the SV7 §2.6
  per-sample reconstruction primitives on top of the round-191
  requantiser constants and the round-201 per-band level decode:
  - `DEQUANT_DIVISOR: f64 = 65536.0` constant tied to the
    requantiser relation `C = 65536 / (2D + 1)`.
  - `centre_pcm_level(band_type, raw_unsigned) -> Result<i32>` and
    `centre_pcm_band(band_type, &mut [i32; 36]) -> Result<()>` —
    PCM-escape centring (subtract `D = QUANTIZER_OFFSET_D[band_type
    + 1]`) for band_types 8..=17.
  - `dequantise_sample(band_type, centred_level) -> Result<f64>` —
    single-sample dequant via `centred_level * C / 65536`, covering
    both the CNS / noise band (`-1`) and the normal `0..=17`
    range.
  - `dequantise_band(band_type, &centred, &mut out) -> Result<()>` —
    whole-band variant for `0..=17`.
  - `dequantise_huffman_band(band_type, &huffman_i8, &mut out) ->
    Result<()>` — convenience wrapper accepting the `[i8; 36]`
    shape returned by `decode_huffman_band` for band_types 3..=7.
  - `dequantise_cns_band(&cns_levels, &mut out)` — CNS-specific
    wrapper keyed off `DEQUANT_COEFFICIENT_C[0]` (the
    `32768/2/255*sqrt(3)` anchor per the
    `cns-prng-params.meta` notes line).
  - `pcm_escape_d(band_type) -> Option<i32>` helper.
- **Round 206** — 18 new unit tests across `reconstruct::tests`
  covering: single-sample PCM-centring at band_types 8 and 17
  (boundary inputs `0`, `D`, `2D`), full-band in-place centring,
  out-of-range rejection for both centring functions;
  single-sample dequant for band_type 0 (identity scaling),
  band_type 3 (`d / (2d + 1)`), band_type 17 (`d / (2d + 1)`,
  large-`D` floating-point sanity), and the CNS band (`-1`); whole-
  band dequant against the single-sample path; Huffman-band
  dequant for band_type 3 with a signed i8 ramp; CNS dequant
  magnitude bound check against the PRNG's `-510..=510` range; the
  `pcm_escape_d` helper across the PCM-escape range; out-of-range
  rejection for whole-band paths; and a cross-module integration
  test that wires the PCM-escape Sv7BitReader, the round-201
  PCM-escape decoder, the round-206 centring step, and the
  round-206 dequant multiply end-to-end against a known synthetic
  input. Total crate test count `67 → 85`.

- **Round 201** — `sv7_band_decode` module wiring the SV7 §2.5
  per-band sample-decode `switch (band_type)` dispatch on top of the
  already-staged Huffman / CNS / requant tables:
  - `BandDecodeCase` enum classifies every spec case (`Cns`,
    `Empty`, `Grouped3`, `Grouped2`, `HuffmanPerSample`, `PcmEscape`,
    `OutOfRange`); `band_type_case(i8) -> BandDecodeCase` is a pure
    `const fn` dispatch.
  - `fill_zero_band(out)` — case 0, fills 36 zero samples.
  - `fill_cns_band(prng, out)` — case -1, passes through to the
    already-wired `CnsPrng::fill_samples`.
  - `decode_huffman_band(reader, band_type, ctx, out)` — cases
    3..=7, one Q`band_type` Huffman codeword per sample,
    context-selected via `sv7_q{3..=7}_ctx(ctx)`.
  - `decode_linear_pcm_band(reader, band_type, out)` — cases
    8..=17, `band_type - 1` unsigned bits per sample read MSB-first
    into an `[i32; 36]` raw-level buffer.
  - `SAMPLES_PER_BAND = 36` shared by all four decoders (Layer-II
    heritage per spec §1).
- **Round 201** — `Error::UnsupportedBandType(i8)` variant for the
  per-band-decode dispatcher's fail-loud channel: triggered by the
  structurally-documented-but-unimplemented grouped cases (1, 2)
  and by any out-of-range `band_type` or `ctx`.
- **Round 201** — 11 new unit tests across `sv7_band_decode::tests`
  covering: the classifier across `-2..=18` plus `i8` extremes; the
  zero / CNS fill paths (CNS round-trip vs a directly-driven PRNG
  walk with matching state); the Huffman path on band_type 3 (ctx
  0, shortest-code 36×) and band_type 7 (both contexts, signed-level
  range `-31..=31` invariant); the PCM-escape path on band_type 8
  (7 bits/sample, ramp round-trip) and band_type 17 (16 bits/sample,
  distinct-pattern round-trip); explicit `UnsupportedBandType`
  rejection edges for every dispatcher; EOF propagation through the
  PCM-escape reader. Total crate test count `56 → 67`.

- **Round 197** — `huffman` module wiring the SV7
  `mpc_huffman`-shape entropy tables staged under
  `docs/audio/musepack/tables/sv7-huffman-*.csv` through a
  `build.rs`-driven CSV-to-Rust generator. The script reads only
  the `.csv` numeric initialisers (the Feist facts of the format)
  and emits typed `Sv7Entry` arrays into `OUT_DIR`. Generated /
  exposed tables, all keyed by the staged `.meta`
  `resolved_dims`:
  - `SV7_BANDTYPE_HEADER_TABLE` (10 entries) — band-type / header
    VLC per spec §2.3.
  - `SV7_SCFI_TABLE` (4 entries) — SCF coding-method selector per
    spec §2.4.
  - `SV7_DSCF_TABLE` (16 entries) — delta-scalefactor VLC per spec
    §2.4.
  - `SV7_Q1_TABLE` .. `SV7_Q7_TABLE` (54 / 50 / 14 / 18 / 30 / 62
    / 126 entries) — per-quantiser sample VLCs per spec §2.5,
    each a `[2][N]` context-pair with the two contexts
    concatenated. A `sv7_q{1..=7}_ctx(ctx)` accessor returns the
    requested half-slice per the `.meta` notes line.
  - A `Sv7Entry { code, length, value }` struct mirroring the
    sidecar's `mpc_huffman = {Code:uint16 left-adjusted,
    Length:uint8, Value:int8}` shape, with the canonical-code
    `code <= peek16()` decode loop driving `huffman::decode`.
  - A standalone `Sv7BitReader<'a>` over an in-memory `&[u8]`
    slice, MSB-first, with `peek16`, `consume_bits(1..=32)`, and
    `read_bits(1..=16)` for the spec §2.5 case 8..=17 linear-PCM
    escape ladder. The SV7 per-frame 20-bit length prefix and the
    "read in 32-LSB units" word packing per spec §2.2 are *not*
    handled here — they belong to a later frame-driver round.
- **Round 197** — `cns` module wiring the noise-substitution
  generator from `docs/audio/musepack/tables/cns-prng-{parity,
  params}.csv` via the same `build.rs`. The 256-byte `PARITY`
  table is generated from `cns-prng-parity.csv`; the six scalar
  parameters (`R1_SEED` / `R2_SEED` / `R1_TAP_MASK` /
  `R2_TAP_MASK` / `R2_SHIFT` / `NOISE_SAMPLE_BYTE_SUM_BIAS`) are
  generated from `cns-prng-params.csv`. The `CnsPrng` two-LFSR
  state machine implements the generator step transcribed from
  the staged `.meta` `notes:` line: `r1 = (r1 >> 1) | (Parity[r1
  & 0xF5] << 31); r2 = (r2 << 1) | Parity[(r2 >> 25) & 0x63];
  word = r1 ^ r2; q = byte_sum(word) - 510`. The first step from
  the reset state is verified against a hand-cranked walk
  (`r1' = 0x8000_0000`, `r2' = 2`, `word = 0x8000_0002`, first
  sample = `-380`).
- **Round 197** — `Error::HuffmanNoMatch` variant covering the
  case where no row of the supplied SV7 Huffman table matches the
  peeked 16-bit code window.
- **Round 197** — 22 new unit tests across `huffman::tests` (10)
  and `cns::tests` (6) plus shape assertions on each of the 10
  Huffman tables and the parity table (each test pins the entry
  count from the `.meta` `resolved_dims` line plus the last
  entry of the CSV). Total crate test count `34 → 56`; `cargo
  build`, `cargo test`, `cargo clippy -- -D warnings` and `cargo
  fmt --check` all clean.

### Notes

- The SV8 canonical-Huffman entropy tables
  (`sv8-canonical-*` length tables + `sv8-symbols-*` symbol
  maps) and the SV7/SV8 quantiser/CNS-related constants beyond
  the four wired in round 191 (`requant-*`) remain staged but
  unwired this round — they need a length-table-to-code-table
  builder (canonical-Huffman) that's a different decoder shape
  from the SV7 `mpc_huffman` linear walk and is queued for a
  follow-up round.
- The `build.rs` accepts an `OXIDEAV_MUSEPACK_DOCS_DIR` env-var
  override pointing at the `docs/` root, so the crate can be
  built outside the umbrella checkout. For standalone / CI /
  crates.io builds the script falls back to a vendored snapshot
  of the consumed CSV+`.meta` subset shipped at
  `<crate>/tables/`; the snapshot must stay byte-equal with the
  umbrella's `docs/audio/musepack/tables/` and is refreshed
  manually whenever the docs collaborator restages.

- **Round 194** — `framing` module covering the parts of the
  Musepack container that are *structurally* specified by
  independent sources in `docs/audio/musepack/musepack-sv7-sv8-spec.md`:
  - `SV7_MAGIC = b"MP+"` + `SV7_VERSION_NIBBLE = 7` constants and
    a `SV7Header::parse_magic` recogniser (§2.1) that returns the
    version byte and a slice over the still-GAP rest of the fixed
    header without decoding it.
  - `SV8_MAGIC = b"MPCK"` constant + `parse_sv8_magic` recogniser
    (§3.1).
  - `PacketKey` enum covering the full §3.2 vocabulary
    (`SH` / `RG` / `EI` / `SO` / `ST` / `AP` / `SE`) plus an
    `Unknown([u8; 2])` catch-all, with `from_bytes` /
    `as_bytes` / `is_known` helpers.
  - `PacketHeader` + `parse_packet_header` walking the
    `[2-byte key][varint size][payload]` outer frame per §3.1,
    plus a `parse_varint` decoder for the continuation-bit
    big-endian shape described there. Because §3.1 flags the
    "varint inclusive of header bytes?" convention as GAP, the
    decoded header exposes both `payload_len_inclusive()` (returns
    `None` if the size is too small to cover the header) and
    `payload_len_exclusive()` so the caller can pick the right one
    once the observer trace lands.
  - `StreamKind` + `identify_stream` for magic-bytes-only
    dispatch between SV7 and SV8.
  - `Error` enum extended with `InvalidMagic`, `UnexpectedEof`,
    `UnsupportedVersion(u8)`, and `VarintTooLong` variants.
- **Round 194** — 22 new unit tests in `framing::tests` covering
  magic acceptance and rejection (both versions), the seven known
  packet keys + unknown round-trip, varint single / two / three-byte
  values + truncated and overlong rejection, packet header parsing
  for `SH` / `AP` / `SE`, and a synthetic SV8 stream walk
  reconstructing the packet sequence end-to-end. Total crate test
  count `12 → 34`, all green; clippy clean with
  `-D warnings`; `cargo fmt --check` clean.

### Notes

- The bit-precise field layouts **inside** the SV7 fixed header
  (sample count, intensity / MS flags, `max_band`, encoder
  profile / quality, gapless trailing-sample count, ReplayGain
  title / album gain + peak) and **inside** each SV8 SH / RG / EI
  / SO / ST payload are GAP per the structural spec — they live
  only in the project's walled Trac `SV7Specification` /
  `SV8Specification` pages. They are intentionally **not**
  implemented this round and are the target of the pending
  Musepack observer-trace round (workspace task #1263).

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
