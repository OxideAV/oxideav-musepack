//! The SV8 `SH`-packet CRC-32.
//!
//! `spec/musepack-headers-and-coding.md` §2 (field 1) states the `SH`
//! payload opens with a "CRC-32 over the `SH` payload from the byte
//! after the CRC to the packet end" but does **not** pin the
//! polynomial/reflection/init parameters, so the decode side
//! ([`crate::sh_header`]) surfaces the value without validating it.
//!
//! For the **encode** side the parameters are pinned **empirically
//! against the staged fixture corpus** (the same black-box method as
//! the r390/r419 wire pinning): recomputing the checksum of every
//! `tests/fixtures/sv8/*/input.mpc` `SH` payload with the standard
//! reflected CRC-32 (polynomial `0xEDB88320`, init `0xFFFFFFFF`,
//! final XOR `0xFFFFFFFF` — the ubiquitous IEEE 802.3 / zlib variant)
//! reproduces all seven streams' stored values exactly, while the
//! common alternates (unreflected `0x04C11DB7` with either finaliser,
//! POSIX cksum) match none. That gate lives in `tests/sv8_corpus.rs`
//! so the pin re-verifies against the corpus on every run.
//!
//! Bit-shift implementation (no lookup table): the byte enters the
//! low end of the register (reflected form) and eight conditional
//! `>> 1` / XOR steps follow.

/// The reflected CRC-32 polynomial (IEEE 802.3), bit-reversed form.
pub const CRC32_POLY_REFLECTED: u32 = 0xEDB8_8320;

/// Compute the SV8 `SH` checksum of `data`: standard reflected CRC-32
/// (init `0xFFFFFFFF`, final XOR `0xFFFFFFFF`).
#[must_use]
pub fn sv8_crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &byte in data {
        crc ^= u32::from(byte);
        for _ in 0..8 {
            let lsb = crc & 1;
            crc >>= 1;
            if lsb != 0 {
                crc ^= CRC32_POLY_REFLECTED;
            }
        }
    }
    crc ^ 0xFFFF_FFFF
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The classic check value: CRC-32("123456789") = 0xCBF43926 for
    /// the reflected IEEE variant.
    #[test]
    fn matches_the_standard_check_value() {
        assert_eq!(sv8_crc32(b"123456789"), 0xCBF4_3926);
    }

    /// Empty input is the identity (0x00000000 after the double
    /// complement).
    #[test]
    fn empty_input() {
        assert_eq!(sv8_crc32(b""), 0);
    }

    /// A single flipped bit changes the checksum (basic error
    /// detection sanity).
    #[test]
    fn detects_single_bit_flip() {
        let a = sv8_crc32(&[0x00, 0x01, 0x02, 0x03]);
        let b = sv8_crc32(&[0x00, 0x01, 0x02, 0x07]);
        assert_ne!(a, b);
    }

    /// Recompute the SH checksum of every staged SV8 fixture: the
    /// stored field must reproduce exactly (the empirical parameter
    /// pin — see the module docs).
    #[test]
    fn fixture_sh_checksums_reproduce() {
        use crate::packet_stream::{PacketSizeConvention, PacketStream};
        use crate::sh_header::StreamHeaderFields;
        use crate::typed_packet::TypedPacket;

        for name in [
            "stereo-sine-partial-last-frame",
            "exact-multiple-16-frames",
            "silence-then-tone-partial",
            "stereo-sine-xtreme-quality",
            "cns-pns",
            "mono-sine-standard",
            "stereo-sine-two-packets",
        ] {
            let path = format!(
                "{}/tests/fixtures/sv8/{name}/input.mpc",
                env!("CARGO_MANIFEST_DIR")
            );
            let bytes = std::fs::read(&path).expect("fixture");
            let mut stream = PacketStream::new(&bytes[4..], PacketSizeConvention::Inclusive);
            let mut seen = false;
            while let Some(p) = stream.next_packet().unwrap() {
                if let TypedPacket::StreamHeader(sh) = TypedPacket::classify(p) {
                    let payload = sh.payload_bytes();
                    let fields = StreamHeaderFields::parse(payload).unwrap();
                    assert_eq!(
                        sv8_crc32(&payload[4..]),
                        fields.crc,
                        "{name}: SH checksum must reproduce"
                    );
                    seen = true;
                    break;
                }
            }
            assert!(seen, "{name}: no SH packet found");
        }
    }
}
