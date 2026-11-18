//! Stable on-disk identifiers and framing constants.
//!
//! The archive-family marker, layout revision, and codec selector have
//! separate roles: the marker identifies this project's container, the
//! revision describes its header/section layout, and the selector chooses
//! the sensor encoding.

/// Four-byte archive-family marker. This is distinct from the format version.
pub const MAGIC: &[u8; 4] = b"AAA1";
/// Version of the archive header/framing layout currently emitted and decoded.
pub const FORMAT_VERSION: u8 = 2;
/// Codec selector stored in the current archive header.
pub const CODEC_ID: u16 = 3700;
/// Fixed prefix length through both SHA-256 digests.
pub const FIXED_HEADER_LEN: usize = 127;
