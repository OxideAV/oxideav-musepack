//! Fuzz the SV8 whole-stream decoder: arbitrary bytes through
//! `decode_sv8_stream` — the `MPCK` packet walker, the `SH`/`RG`/`EI`
//! field maps, the multi-frame `AP` entropy decode, and the
//! totals-bounded drain.
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_musepack::sv8_decode::decode_sv8_stream(data);
});
