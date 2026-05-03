# oxideav-musepack

Pure-Rust **Musepack** audio decoder — both the legacy SV7 (`MP+`)
container and the modern SV8 (`MPCK`) chunked container. Zero C
dependencies; clean-room from the public `musepack.net` SV7/SV8
specifications and an internal trace-based reverse-engineering writeup
(under `docs/audio/musepack/` of the workspace).

Part of the [oxideav](https://github.com/OxideAV/oxideav-workspace)
framework but usable standalone.

## Status

| Variant | Demuxer | Decoder | Notes                                                    |
|---------|---------|---------|----------------------------------------------------------|
| SV8     | yes     | yes     | Mono / stereo, all four sample rates, all `res` paths    |
| SV7     | yes     | partial | Demuxer reads the `MP+` 16-byte fixed header + 20-bit per-frame length prefixes; decoder shares synthesis but the SV7 entropy path is still being wired in |

SV8 is shipped first because the chunked container (`SH` / `AP` / `SE`
+ explicit `total_samples` / `block_power`) is structurally simpler
than SV7's running 20-bit-prefix bit cursor.

## Installation

```toml
[dependencies]
oxideav-core = "0.1"
oxideav-musepack = "0.0"
```

## Decoder

Accepts `MPCK`-prefixed Musepack SV8 streams (mono or stereo, sample
rates `{44 100, 48 000, 37 800, 32 000}` Hz, all `res ∈ [-1..17]`
quantiser branches). Output frames are interleaved
`SampleFormat::S16` at 1152 samples per channel per sub-frame; one
`AP` packet expands to `1 << (2 * block_power)` such sub-frames.

```rust
use oxideav_core::registry::CodecRegistry;
use oxideav_core::{CodecId, CodecParameters};

let mut codecs = CodecRegistry::new();
oxideav_musepack::register(&mut codecs);

let params = CodecParameters::audio(CodecId::new("musepack8"));
let mut dec = codecs.make_decoder(&params)?;
# Ok::<(), oxideav_core::Error>(())
```

## Encoder

Not provided. Musepack's reference encoder (`mpcenc`) is the only
production-grade encoder in existence and our workspace policy bans
any third-party source (including a clean-room port). We focus on
decode + container parse parity.

## License

MIT — see `LICENSE`.
