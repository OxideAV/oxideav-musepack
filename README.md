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

A strict-isolation clean-room workspace at `docs/` will be stood up before the rebuild's Implementer round can run; this orphan `master` is a placeholder pending that workspace.

The `oxideav_core::CodecResolver` registration this crate's
`register(ctx)` function provides will be wired up by the
Implementer round; until then the public API surfaces only the
crate-local `Error::NotImplemented` placeholder.

## Docs blocker (round 84)

Round 84 attempted round 1 of the rebuild (foundational SV8
stream-header parse) and confirmed that `docs/audio/musepack/`
currently contains only `wiki/Musepack.wiki` — a 72-line
multimedia.cx overview that links outward (to `trac.musepack.net`)
for the SV7 and SV8 specs but carries **no** byte-level field
layout, magic identifier, packet taxonomy, or table. Implementer
work cannot proceed under the clean-room wall until a docs
collaborator stands up `docs/audio/musepack/spec/` (SV7 + SV8
byte-level field maps) and `docs/audio/musepack/tables/` (the
Huffman / CNS / SCF tables). See `CHANGELOG.md` ("Blocked") for
the full gap list.
