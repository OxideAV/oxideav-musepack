# SV8 conformance fixture corpus (round 419)

Black-box-produced Musepack **SV8** (`MPCK`) streams with decode
oracles — the first real-stream SV8 corpus for this crate. Two fixture
families:

## 1. Lossless transcodes of the staged SV7 corpus

Five fixtures whose `input.mpc` is the **lossless SV7→SV8 transcode**
(`mpc2sv8`, Musepack tools r475, used as a black-box converter) of the
project's staged SV7 corpus at `docs/audio/musepack/fixtures/` (docs
commit `af1b888` / `0f1b6a2`; the same streams imported under
`../sv7/`). Per the structural spec §3.6 the quantised-coefficient
payload is numerically identical between the generations — which makes
these transcodes a **ground-truth corpus**: every SV8 frame must decode
to exactly the `Res` / M/S / SCF / sample-level structure the (already
corpus-pinned) SV7 decode of the sibling stream produces. That equality
is what pinned the r419 SV8 frame-body layout, and
`tests/sv8_corpus.rs` gates it end-to-end at the PCM level.

| Fixture | Frames | maxband | ch | block_power | sample_count |
|---|--:|--:|--:|--:|--:|
| `stereo-sine-partial-last-frame` | 20 | 28 | 2 | 3 | 22050 |
| `exact-multiple-16-frames` | 16 | 28 | 2 | 3 | 18432 |
| `silence-then-tone-partial` | 16 | 28 | 2 | 3 | 18432 |
| `stereo-sine-xtreme-quality` | 20 | 31 | 2 | 3 | 22050 |
| `cns-pns` | 20 | 28 | 2 | 3 | 22050 |

Transcode command (per fixture, from the docs-staged SV7 stream):

```
mpc2sv8 <docs>/fixtures/<name>/input.mpc input.mpc
```

## 2. Fresh reference-encoder streams

Two fixtures encoded directly to SV8 with `mpcenc` (Musepack tools
r475, black-box fixture producer) from deterministic synthetic WAVs:

- `mono-sine-standard` — 0.5 s, 44100 Hz **mono** 525 Hz sine,
  `--standard`. 20 frames, one `AP`. Pins the mono-stream shape: the
  `SH` declares 1 channel but every frame body still codes two
  channels (the r419 two-channel-body finding).
- `stereo-sine-two-packets` — 2 s stereo 440/660 Hz sine pair,
  `--standard`. 77 frames = 88200 samples across **two `AP` packets**
  (64 + 13 frames, `block_power` 3). Pins the multi-packet path: each
  `AP` opens with a key frame and the SCF memory resets at the packet
  boundary.

```
ffmpeg -f lavfi -i "sine=frequency=525:sample_rate=44100:duration=0.5" \
       -c:a pcm_s16le A_mono_sine.wav
mpcenc --overwrite --standard A_mono_sine.wav mono-sine-standard/input.mpc

ffmpeg -f lavfi -i "sine=frequency=440:sample_rate=44100:duration=2" \
       -f lavfi -i "sine=frequency=660:sample_rate=44100:duration=2" \
       -filter_complex "[0:a][1:a]join=inputs=2:channel_layout=stereo[a]" \
       -map "[a]" -c:a pcm_s16le A_stereo_long.wav
mpcenc --overwrite --standard A_stereo_long.wav stereo-sine-two-packets/input.mpc
```

## Oracles

Each `expected.pcm` is FFmpeg's `mpc8` decoder output used as a
black-box validator: raw interleaved little-endian `s16le`, 44100 Hz,
at the stream's channel count.

```
ffmpeg -i input.mpc -f s16le -acodec pcm_s16le expected.pcm
```

The oracle emits full 1152-sample frames **without** gapless trimming
(and for `exact-multiple-16-frames` appends one extra flush frame), so
the crate's trimmed output is compared as a prefix. Notably, for 4 of
the 5 transcoded fixtures the `mpc8` oracle bytes are **identical** to
the staged SV7 fixtures' `mpc7` oracle bytes (`exact-multiple`'s
differs only by that appended flush frame) — independent black-box
confirmation of the §3.6 lossless SV7↔SV8 relationship.

The `cns-pns` transcode inherits the SV7 corpus's known limitation:
the oracle's Clear-Noise-Substitution noise **waveform** is not
reproducible from the staged generator facts (a filed docs gap — see
`tests/sv7_cns_corpus.rs`), so that fixture is gated on its CNS-free
first frame plus a statistical correlation bound, exactly like its SV7
sibling.

## Tools (black-box only)

| Tool | Version | Role |
|---|---|---|
| `mpc2sv8`, `mpcenc` | Musepack tools r475 (Homebrew `musepack`) | fixture producers (no source read) |
| FFmpeg `mpc8` decoder | 8.x | decode oracle (no source read) |

## SHA-256

```
5c6e57c87ff013631cda0044c25ae3baca837a130cd54e250a0fdf9836a86842  stereo-sine-partial-last-frame/input.mpc
b1c2ff5560946905ad3d585acfcd484f7f7ffd105e6fb1e77e2cdad29c541b4f  stereo-sine-partial-last-frame/expected.pcm
27bc2a033ce195e07e65ac90f86167fb207250de957a95fcbf6e9cf5313e5046  exact-multiple-16-frames/input.mpc
e6dc84bb91a7a62a84f2d6a34104d3df47c5a9fe668b875f9f4000239deded8e  exact-multiple-16-frames/expected.pcm
bcb062036fc35d11581ffc6a0e45140ad3e9d79f798cc6970b2eb4d06e1422c6  silence-then-tone-partial/input.mpc
42e5ebc3f8a97f85a79b51dc44ef3026405c1bee1490d02f1fc88fb35c8b12bb  silence-then-tone-partial/expected.pcm
0c37365d9230fc79dd3385c31a0bd7b17d8dbdb7f91cb470510ca606b8ddf12e  stereo-sine-xtreme-quality/input.mpc
69b2c1f9e5c1d0c861ac1bb568e565afad9a80aade42ae4d2775a67fa0cbdf83  stereo-sine-xtreme-quality/expected.pcm
6040b52f76963c083096a064f227da9252116f2372af0307c02098bc15b1056c  cns-pns/input.mpc
cfc9e5605d05909d140b8ef2e5ef5abaa5cbe01dfe3f77cbcc31d2c61bdf8f07  cns-pns/expected.pcm
7d8f72c7fdd8c12b888acb41c811a18ea3936c20219f014ecf124b31a28e6c26  mono-sine-standard/input.mpc
ba0dafffd9d505577b932cf722cd933586d29df8820677ea22fd7966270ecee6  mono-sine-standard/expected.pcm
8f745751f74c1a186b624437ab3b275acd271b470bc45aeaf33a53163dedcfa0  stereo-sine-two-packets/input.mpc
0923e07ee4f88b769e15bcc3afee73def2cd886368f0a6558321aba54d4c6d13  stereo-sine-two-packets/expected.pcm
```

Deterministic: re-running the transcode / encode commands reproduces
each `input.mpc` byte-for-byte. This corpus should be mirrored into the
project's docs staging (`docs/audio/musepack/fixtures/`) by a docs
round; this copy exists so the crate's CI gates run self-contained.
