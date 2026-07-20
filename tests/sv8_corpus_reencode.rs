//! SV8 corpus re-encode gates (round 419): wire symmetry with the
//! reference producers.
//!
//! For every `AP` packet of every corpus fixture
//! (`tests/fixtures/sv8/`): decode each frame to its structured form
//! ([`oxideav_musepack::sv8_stereo_frame::Sv8StereoFrameDecode`]),
//! re-encode the whole packet with
//! [`oxideav_musepack::sv8_stereo_frame_encode::encode_sv8_stereo_frame`],
//! and require the emitted bytes to equal the original payload
//! **exactly** — data bits and byte-boundary zero padding alike.
//!
//! Passing this over the transcoded fixtures and the fresh
//! reference-encoder streams proves the encoder is wire-symmetric with
//! the reference tools: on this corpus the frame-body coding leaves the
//! encoder no free choices our inverse resolves differently (SCFI
//! selectors are carried through the structured decode; DSCF
//! escape-vs-direct forms are forced by the delta value; the non-key
//! `Bands` delta ring resolves to the in-alphabet form).

use oxideav_musepack::cns::CnsPrng;
use oxideav_musepack::huffman::Sv7BitReader;
use oxideav_musepack::packet_stream::{PacketSizeConvention, PacketStream};
use oxideav_musepack::sh_header::StreamHeaderFields;
use oxideav_musepack::sv7_bitwriter::Sv7BitWriter;
use oxideav_musepack::sv8_stereo_frame::{decode_sv8_stereo_frame, Sv8FrameState};
use oxideav_musepack::sv8_stereo_frame_encode::encode_sv8_stereo_frame;
use oxideav_musepack::SAMPLES_PER_FRAME_PER_CHANNEL;

const CORPUS: [&str; 7] = [
    "stereo-sine-partial-last-frame",
    "exact-multiple-16-frames",
    "silence-then-tone-partial",
    "stereo-sine-xtreme-quality",
    "cns-pns",
    "mono-sine-standard",
    "stereo-sine-two-packets",
];

fn fixture(name: &str) -> Vec<u8> {
    let path = format!(
        "{}/tests/fixtures/sv8/{name}/input.mpc",
        env!("CARGO_MANIFEST_DIR")
    );
    std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"))
}

#[test]
fn every_corpus_audio_packet_reencodes_byte_identically() {
    for name in CORPUS {
        let bytes = fixture(name);
        assert_eq!(&bytes[..4], b"MPCK", "{name}");
        let mut stream = PacketStream::new(&bytes[4..], PacketSizeConvention::Inclusive);
        let mut sh: Option<StreamHeaderFields> = None;
        let mut frames_remaining: u64 = 0;
        let mut frames_per_packet: u64 = 0;
        // One CNS PRNG for the whole stream (decode order), mirroring
        // the stream decoder.
        let mut cns = CnsPrng::new();
        let mut ap_index = 0usize;
        while let Some(p) = stream.next_packet().unwrap() {
            let key = format!("{:?}", p.key);
            if key == "StreamHeader" {
                let fields = StreamHeaderFields::parse(p.payload).unwrap();
                frames_per_packet = fields.frames_per_audio_packet();
                frames_remaining = (fields.sample_count + fields.beginning_silence)
                    .div_ceil(SAMPLES_PER_FRAME_PER_CHANNEL as u64);
                sh = Some(fields);
            } else if key == "AudioPacket" {
                let sh = sh.as_ref().expect("SH before AP");
                let min_frames = frames_remaining.min(frames_per_packet);

                // Decode every frame the packet physically carries: at
                // least the totals-implied count, and any further coded
                // frames up to the byte-padding tail (a reference
                // encoder may flush one frame past the sample totals —
                // the `exact-multiple` transcode does; its oracle
                // decodes that extra frame too).
                let payload_bits = (p.payload.len() as u64) * 8;
                let mut padded = p.payload.to_vec();
                padded.extend_from_slice(&[0, 0]);
                let mut reader = Sv7BitReader::new(&padded);
                let total_bits = reader.bits_remaining();
                let mut dstate = Sv8FrameState::new();
                let mut decoded = Vec::new();
                let mut f: u64 = 0;
                while f < frames_per_packet {
                    let consumed = total_bits - reader.bits_remaining();
                    if f >= min_frames && payload_bits - consumed < 8 {
                        break; // only byte-boundary zero padding remains
                    }
                    let frame = decode_sv8_stereo_frame(
                        &mut reader,
                        sh.max_band,
                        f == 0,
                        sh.mid_side,
                        &mut dstate,
                        &mut cns,
                    )
                    .unwrap_or_else(|e| panic!("{name} AP{ap_index} frame {f}: {e:?}"));
                    decoded.push(frame);
                    f += 1;
                }
                frames_remaining -= min_frames;

                // Re-encode the whole packet.
                let mut estate = Sv8FrameState::new();
                let mut w = Sv7BitWriter::new();
                for (f, frame) in decoded.iter().enumerate() {
                    encode_sv8_stereo_frame(
                        &mut w,
                        frame,
                        sh.max_band,
                        f == 0,
                        sh.mid_side,
                        &mut estate,
                    )
                    .unwrap_or_else(|e| panic!("{name} AP{ap_index} frame {f} encode: {e:?}"));
                }
                let emitted = w.finish();
                assert_eq!(
                    emitted.len(),
                    p.payload.len(),
                    "{name} AP{ap_index}: payload length"
                );
                assert_eq!(
                    emitted, p.payload,
                    "{name} AP{ap_index}: byte-identical re-encode"
                );
                ap_index += 1;
            }
        }
        assert!(ap_index > 0, "{name}: no AP packets exercised");
    }
}
