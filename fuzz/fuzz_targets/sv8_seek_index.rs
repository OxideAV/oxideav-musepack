//! Fuzz the SV8 seek layer (headers-and-coding §9): the `SO`/`ST`
//! payload parsers, the index resolvers, and a random-access decode
//! from each resolved entry.
#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_musepack::sv8_seek::{decode_sv8_from_entry, SeekTableFields, Sv8SeekIndex};

fuzz_target!(|data: &[u8]| {
    // The raw §9.2 payload parser on the naked input.
    let _ = SeekTableFields::parse(data);
    // The stream-level resolvers + a bounded random access.
    if let Ok(Some(index)) = Sv8SeekIndex::from_seek_packets(data) {
        for entry in 0..index.positions.len().min(4) {
            let _ = decode_sv8_from_entry(data, &index, entry);
        }
    }
    let _ = Sv8SeekIndex::from_packet_walk(data);
});
