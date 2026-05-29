# oxideav-musepack

Pure-Rust Musepack audio codec for the
[oxideav](https://github.com/OxideAV/oxideav-workspace) framework.

## Status

**Round 0 — clean-room rebuild scaffold.** This `master` branch is a
fresh orphan. The previous implementation was retired alongside the
docs audit dated 2026-05-06 (see
[`AUDIT-2026-05-06.md`](https://github.com/OxideAV/docs/blob/master/AUDIT-2026-05-06.md)),
which found that the source-of-record trace document for this codec
was authored with a methodology that did not satisfy clean-room
separation. The prior history is preserved on the `old` branch for
archival but is forbidden input for the rebuild.

A strict-isolation clean-room workspace at `docs/` is being assembled
before the rebuild's Implementer rounds can run; this orphan `master`
is a placeholder pending that workspace.

The `oxideav_core::CodecResolver` registration this crate's
`register(ctx)` function provides will be wired up by the
Implementer round; until then the public API surfaces only the
crate-local `Error::NotImplemented` placeholder.

## Docs status (round 186 reassessment)

A docs staging round on 2026-05-25 (workspace docs commit `78e2487`)
added `docs/audio/musepack/wikipedia-musepack.html` (CC-BY-SA
Wikipedia article) alongside the existing `wiki/Musepack.wiki`
(CC-BY-SA multimedia.cx snapshot from 2026-04). The staging round
also published an explicit project-shipped-docs policy decision in
`docs/audio/musepack/README.md`:

- The canonical SV7 / SV8 format documentation lives on
  `trac.musepack.net/musepack/wiki/SV8Specification` and is
  **project-shipped** (authored by the maintainers of the
  libmpcdec / mpcenc reference implementation). Under this repo's
  clean-room policy, project-shipped docs from copyrighted-but-
  permissive licences are **not** mirrored — link-only.
- The Mutagen Specs SV8 archive is verbatim mirrored from the same
  upstream and inherits the same project-shipped status.
- The libmpcdec / mpcenc source itself is BSD-3-Clause but
  off-limits to Implementer agents for the same project-shipped
  reason.
- **Fixed numeric tables** (Huffman codebooks for SV7 / SV8, CNS
  Pascal-grid, `huffq2[125]`, `CC[19]`, `SCF[256]`, psychoacoustic-
  model constants) are facts, not creative expression, and per
  *Feist v. Rural* (1991) are extractable as data to
  `docs/audio/musepack/tables/` from libmpcdec by a future
  docs-collaborator round.

The staged material is sufficient to recognise the high-level
codec shape (32-band polyphase subband filter inherited from
MPEG-1 Layer 2; SV7 packet framing with 20-bit per-frame length
prefix; SV8 KEY / SIZE / PAYLOAD packet taxonomy with `MPCK`
magic; SV7 / SV8 band-type and quantiser branching as sketched in
the multimedia.cx wiki overview) but is **insufficient for a
bit-exact decoder implementation**. Specifically still missing
under the wall:

- **SV7 header field map** (magic, sample-frequency / max-band /
  max-level / title / VBR fields, the 20-bit per-frame length
  prefix encoding).
- **SV8 packet field map** (file magic `MPCK`, SH / RG / EI / SO
  / ST / CT packet taxonomy, varint key/size framing; SH packet's
  sample-count / beginning-silence / sample-freq-index /
  max-used-bands / channel-count / ms-used / audio-block-frames
  field layout).
- **SV7 VLC tables** — SCFI, DSCF, header, the seven quant-VLC
  sets (band types 1, 2, 3–7, 8–17 dispatch).
- **SV8 VLC tables** — band, scfi, dscf, res, q1 / q2 / q3 / q4 /
  q5..q8 / q9up plus the CNS Pascal-grid / `huffq2[125]` / `CC[19]`
  / `SCF[256]` constants.

Implementer code for SV7 or SV8 cannot land without violating the
clean-room wall until either (a) a clean-room observer-trace
session (per `docs/CLEANROOM-MANUAL.md` §6 + §10) produces
`docs/audio/musepack/musepack-observer-spec.md`, or (b) a
docs-collaborator round transcribes the libmpcdec numeric tables
to `docs/audio/musepack/tables/` under the *Feist v. Rural*
data-extraction exception (mirroring `docs/audio/g729/tables/`
layout: CSV with spec-role-named filenames + `.meta` provenance
sidecars).

See `CHANGELOG.md` `[Unreleased]` "Blocked" for the round-by-round
gap tracker.

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
