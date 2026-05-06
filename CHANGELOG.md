# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.0.3](https://github.com/OxideAV/oxideav-musepack/compare/v0.0.2...v0.0.3) - 2026-05-06

### Other

- prepend retirement notice (docs audit 2026-05-06)
- registry calls: rename make_decoder/make_encoder → first_decoder/first_encoder

## [0.0.2](https://github.com/OxideAV/oxideav-musepack/compare/v0.0.1...v0.0.2) - 2026-05-03

### Other

- align CC[] trailing comments to rustfmt column width
- digit-grouping + unused-mut fixes
- replace never-match regex with semver_check = false

### Added

- Initial scaffold of the Musepack (`MPCK` SV8 + `MP+` SV7) decoder crate.
  SV8 demuxer (chunked container — `SH` / `RG` / `EI` / `SO` / `AP` /
  `ST` / `CT` / `SE`) and audio decoder (per-`res` quantiser dispatch,
  CNS-coded MS-stereo bitmask, `band_vlc` / `res_vlc` / `dscf_vlc` /
  `q1_vlc` / `q2_vlc` / `q3_vlc` / `q5..q8_vlc` / `q9up_vlc`) are wired
  into `oxideav_core::Decoder`. SV7 path is sketch-stage. The 32-band
  PQF synthesis filter bank is shared with `oxideav-mp2`. No third-party
  source consulted; tables transcribed from the in-tree
  `docs/audio/musepack/` clean-room writeup.
