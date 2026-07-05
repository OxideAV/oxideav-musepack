# SV7 conformance fixture corpus

Black-box-produced Musepack SV7 (`MP+`) streams with decode oracles,
imported from the project's staged corpus at
`docs/audio/musepack/fixtures/` (docs commit `af1b888`). Full
provenance — encoder build recipe, source-WAV commands, SHA-256s —
lives there; this copy exists so the crate's CI can run the
conformance gates without the private docs submodule.

Per fixture directory:

- `input.mpc` — a genuine SV7 stream produced by the independent
  reference encoder `mppenc 1.16` (black-box fixture producer; no
  source read).
- `expected.pcm` — the decode oracle: interleaved little-endian
  `s16le` PCM, 44100 Hz, 2 channels, produced by FFmpeg's `mpc7`
  decoder used as a black-box validator. The oracle emits full
  1152-sample frames and does **not** apply gapless trimming, so it
  carries `frames × 1152` samples per channel.

| Fixture | Frames | maxband | profile | last_frame_samples |
|---|--:|--:|--:|--:|
| `stereo-sine-partial-last-frame` | 20 | 28 | 10 | 162 |
| `exact-multiple-16-frames` | 16 | 28 | 10 | 1152 |
| `silence-then-tone-partial` | 16 | 28 | 10 | 360 |
| `stereo-sine-xtreme-quality` | 20 | 31 | 11 | 162 |

All streams: 44100 Hz, stereo, stream M/S on, true-gapless on,
fast-seek on, encoder-version byte 116.
