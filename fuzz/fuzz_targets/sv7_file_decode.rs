//! Fuzz the SV7 whole-file decoder: arbitrary bytes through
//! `decode_sv7_file` must return (Ok or a structured error) without
//! panicking, hanging, or unbounded allocation — the §1 header, the
//! word-swapped non-byte-aligned frame run, the 20-bit prefixes, the
//! four band-major passes, and the flush/drain tail all sit behind
//! this entry.
#![no_main]

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    let _ = oxideav_musepack::sv7_file_decode::decode_sv7_file(data);
});
