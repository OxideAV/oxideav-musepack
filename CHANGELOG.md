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
- Round 186 README refresh — restated the docs-blocker against the
  2026-05-25 docs staging round (workspace docs commit `78e2487`).
  The staging round added `docs/audio/musepack/wikipedia-musepack.html`
  (CC-BY-SA) and an explicit project-shipped-docs policy decision
  declaring `trac.musepack.net/musepack/wiki/SV8Specification`,
  Mutagen Specs SV8, and libmpcdec source link-only / off-limits for
  Implementer agents; documented the *Feist v. Rural* tables-as-data
  unblock path mirroring `docs/audio/g729/tables/`.

### Blocked

- Round 84 (round 1 of the rebuild) targeted a foundational SV8
  stream-header parser and confirmed `docs/audio/musepack/` then
  contained only the multimedia.cx wiki overview.
- Round 186 reassessment against the 2026-05-25 docs staging
  (workspace `78e2487`):
  - **What was staged:** the Wikipedia article (CC-BY-SA,
    high-level overview), the explicit project-shipped-docs policy
    notice, and the link-only reference list for the Trac wiki +
    Mutagen Specs + libmpcdec source.
  - **What is still missing under the wall:**
    - SV7 header field map (magic, sample-frequency / max-band /
      max-level / title / VBR fields, 20-bit per-frame length prefix
      encoding).
    - SV8 packet field map (`MPCK` magic, SH / RG / EI / SO / ST /
      CT packet taxonomy, varint key/size framing; SH packet's
      sample-count / beginning-silence / sample-freq-index /
      max-used-bands / channel-count / ms-used / audio-block-frames
      field layout).
    - SV7 VLC tables — SCFI, DSCF, header, the seven quant-VLC sets
      (band-types 1, 2, 3–7, 8–17 dispatch).
    - SV8 VLC tables — band, scfi, dscf, res, q1 / q2 / q3 / q4 /
      q5..q8 / q9up plus the CNS Pascal-grid / `huffq2[125]` /
      `CC[19]` / `SCF[256]` constants.
  - **Unblock paths:** either (a) commission an observer-trace
    session per `docs/CLEANROOM-MANUAL.md` §6 + §10 to produce
    `docs/audio/musepack/musepack-observer-spec.md`, or (b)
    docs-collaborator round transcribing libmpcdec's numeric
    tables to `docs/audio/musepack/tables/` under the
    *Feist v. Rural* data-extraction exception (CSV + `.meta`
    sidecars mirroring `docs/audio/g729/tables/`). Implementer
    code for SV7 or SV8 cannot land until at least (b) is in place
    for the table content and either (a) or paraphrased
    structural notes derived from non-project-shipped sources
    cover the field maps.
