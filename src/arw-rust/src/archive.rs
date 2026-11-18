//! Construction and restoration of complete versioned ARW archives.
//!
//! Packing locates a supported Sony ARW2 strip, entropy-codes its existing
//! blocks, XZ-compresses the helper tables and untouched file regions, and
//! stores integrity digests in the header. Unpacking validates those sections,
//! reconstructs the original byte sequence, and verifies its SHA-256 digest.
use std::{io, time::Instant};

use crate::{
    decode_sensor, encode_sensor, fit_cross_profile, format, log_to, sensor_maps, Level, PhaseTimer,
};

const MAP_SIZE: usize = 4 * 6 * 256;
const MAX_MEMBER: usize = 1 << 30;
const MAX_PAYLOAD: usize = 1 << 30;

/// Compress one independent archive section with the fixed XZ preset.
fn xz_encode(input: &[u8]) -> Result<Vec<u8>, String> {
    use std::io::Write;
    let options = lzma_rust2::XzOptions::with_preset(6);
    let mut writer = lzma_rust2::XzWriter::new(Vec::new(), options)
        .map_err(|e| format!("XZ compressor initialization failed: {e}"))?;
    writer
        .write_all(input)
        .map_err(|e| format!("XZ compression failed: {e}"))?;
    writer
        .finish()
        .map_err(|e| format!("XZ compression finalization failed: {e}"))
}

/// Decode an XZ section and reject it unless it has the expected exact size.
fn xz_decode(input: &[u8], expected: usize) -> Result<Vec<u8>, String> {
    let mut src = input;
    let mut out = Vec::with_capacity(expected);
    lzma_rs::xz_decompress(&mut src, &mut out).map_err(|e| format!("XZ decode failed: {e}"))?;
    if out.len() != expected {
        return Err(format!(
            "invalid XZ decoded length ({}/{expected})",
            out.len()
        ));
    }
    Ok(out)
}

/// Compute the SHA-256 digest embedded in the archive header.
fn sha256(data: &[u8]) -> [u8; 32] {
    const K: [u32; 64] = [
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4,
        0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe,
        0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f,
        0x4a7484aa, 0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
        0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85, 0x2e1b2138, 0x4d2c6dfc,
        0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b,
        0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116,
        0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
        0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7,
        0xc67178f2,
    ];
    let mut msg = data.to_vec();
    let bit_len = (msg.len() as u64).wrapping_mul(8);
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bit_len.to_be_bytes());
    let mut h = [
        0x6a09e667u32,
        0xbb67ae85,
        0x3c6ef372,
        0xa54ff53a,
        0x510e527f,
        0x9b05688c,
        0x1f83d9ab,
        0x5be0cd19,
    ];
    for chunk in msg.chunks_exact(64) {
        let mut w = [0u32; 64];
        for (i, b) in chunk.chunks_exact(4).enumerate() {
            w[i] = u32::from_be_bytes(b.try_into().unwrap());
        }
        for i in 16..64 {
            let (a, b) = (w[i - 15], w[i - 2]);
            let s0 = a.rotate_right(7) ^ a.rotate_right(18) ^ (a >> 3);
            let s1 = b.rotate_right(17) ^ b.rotate_right(19) ^ (b >> 10);
            w[i] = w[i - 16]
                .wrapping_add(s0)
                .wrapping_add(w[i - 7])
                .wrapping_add(s1);
        }
        let (mut a, mut b, mut c, mut d, mut e, mut f, mut g, mut z) =
            (h[0], h[1], h[2], h[3], h[4], h[5], h[6], h[7]);
        for i in 0..64 {
            let s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
            let ch = (e & f) ^ (!e & g);
            let t1 = z
                .wrapping_add(s1)
                .wrapping_add(ch)
                .wrapping_add(K[i])
                .wrapping_add(w[i]);
            let s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
            let maj = (a & b) ^ (a & c) ^ (b & c);
            let t2 = s0.wrapping_add(maj);
            (z, g, f, e, d, c, b, a) = (g, f, e, d.wrapping_add(t1), c, b, a, t1.wrapping_add(t2));
        }
        for (v, x) in h.iter_mut().zip([a, b, c, d, e, f, g, z]) {
            *v = v.wrapping_add(x);
        }
    }
    let mut out = [0u8; 32];
    for (b, x) in out.chunks_exact_mut(4).zip(h) {
        b.copy_from_slice(&x.to_be_bytes());
    }
    out
}

fn get16(b: &[u8], p: usize, le: bool) -> Result<u16, String> {
    let x: [u8; 2] = b.get(p..p + 2).ok_or("truncated TIFF")?.try_into().unwrap();
    Ok(if le {
        u16::from_le_bytes(x)
    } else {
        u16::from_be_bytes(x)
    })
}
fn get32(b: &[u8], p: usize, le: bool) -> Result<u32, String> {
    let x: [u8; 4] = b.get(p..p + 4).ok_or("truncated TIFF")?.try_into().unwrap();
    Ok(if le {
        u32::from_le_bytes(x)
    } else {
        u32::from_be_bytes(x)
    })
}
/// Find a supported single-strip Sony ARW2 image in a classic TIFF.
///
/// Returns `(strip_offset, strip_size, rows, encoded_row_width)`. The encoded
/// width is derived from strip size divided by row count; it is not the
/// visible image crop width.
fn raw_geometry(raw: &[u8]) -> Result<(usize, usize, usize, usize), String> {
    let le = match raw.get(0..2) {
        Some(b"II") => true,
        Some(b"MM") => false,
        _ => return Err("unsupported TIFF byte order".into()),
    };
    if get16(raw, 2, le)? != 42 {
        return Err("expected classic TIFF".into());
    }
    // TIFF may point to raw data through a SubIFD, a next-IFD link, or both.
    let mut pending = vec![get32(raw, 4, le)? as usize];
    let mut seen = 0;
    while let Some(pos) = pending.pop() {
        if pos == 0 {
            continue;
        }
        seen += 1;
        if seen > 128 {
            return Err("too many TIFF IFDs".into());
        }
        let count = get16(raw, pos, le)? as usize;
        if count > 4096 || pos + 2 + 12 * count + 4 > raw.len() {
            return Err("invalid TIFF IFD".into());
        }
        // Ignore unrelated TIFF metadata; retain only strip and IFD-link tags.
        let mut tags = std::collections::HashMap::<u16, Vec<u32>>::new();
        for i in 0..count {
            let p = pos + 2 + 12 * i;
            let tag = get16(raw, p, le)?;
            if ![259, 273, 257, 279, 278, 330].contains(&tag) {
                continue;
            }
            let typ = get16(raw, p + 2, le)?;
            let cnt = get32(raw, p + 4, le)? as usize;
            let unit = match typ {
                1 => 1,
                3 => 2,
                4 | 13 => 4,
                _ => continue,
            };
            if cnt == 0 || cnt > 64 {
                continue;
            }
            let size = unit * cnt;
            let start = if size <= 4 {
                p + 8
            } else {
                get32(raw, p + 8, le)? as usize
            };
            if start + size > raw.len() {
                return Err("TIFF tag outside file".into());
            }
            let mut vals = Vec::with_capacity(cnt);
            for j in 0..cnt {
                vals.push(match typ {
                    1 => raw[start + j] as u32,
                    3 => get16(raw, start + 2 * j, le)? as u32,
                    _ => get32(raw, start + 4 * j, le)?,
                });
            }
            tags.insert(tag, vals);
        }
        let next = get32(raw, pos + 2 + 12 * count, le)? as usize;
        pending.push(next);
        if let Some(v) = tags.get(&330) {
            pending.extend(v.iter().map(|x| *x as usize));
        }
        // Compression 32767 identifies the Sony block stream this codec
        // understands. Only a single contiguous strip is accepted.
        if tags.get(&259).map(Vec::as_slice) == Some(&[32767][..]) {
            if let (Some(a), Some(b), Some(c)) = (tags.get(&273), tags.get(&257), tags.get(&279)) {
                if a.len() == 1 && b.len() == 1 && c.len() == 1 {
                    let off = a[0] as usize;
                    let h = b[0] as usize;
                    let n = c[0] as usize;
                    if h > 0 && n % h == 0 {
                        let w = n / h;
                        if h % 2 == 0 && w >= 32 && w % 32 == 0 && off + n <= raw.len() {
                            return Ok((off, n, h, w));
                        }
                    }
                }
            }
        }
    }
    Err("no supported ARW2 strip found".into())
}

fn put_u32(v: &mut Vec<u8>, x: u32) {
    v.extend_from_slice(&x.to_le_bytes())
}
fn put_u64(v: &mut Vec<u8>, x: u64) {
    v.extend_from_slice(&x.to_le_bytes())
}
/// Pack one supported ARW byte slice, optionally reporting phase timings.
///
/// The original strip bytes are preserved by the sensor codec; all bytes
/// outside the strip are stored in an independent XZ-compressed section.
pub fn pack_with_level(raw: &[u8], level: Level) -> Result<Vec<u8>, String> {
    if raw.len() > MAX_MEMBER {
        return Err("input exceeds size limit".into());
    }
    let timer = Instant::now();
    let (off, n, h, w) = raw_geometry(raw)?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!("geometry: {w}x{h}, strip {n} bytes at {off}"),
    )
    .map_err(|e| e.to_string())?;
    let map_t = PhaseTimer::start();
    let blocks = &raw[off..off + n];
    let maps = sensor_maps(blocks, h, w)?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Debug,
        &format!("sensor maps: {:.3}s", map_t.elapsed().as_secs_f64()),
    )
    .map_err(|e| e.to_string())?;
    let fit_t = PhaseTimer::start();
    let reg = fit_cross_profile(blocks, h, w).map_err(str::to_string)?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!("regression fit: {:.3}s", fit_t.elapsed().as_secs_f64()),
    )
    .map_err(|e| e.to_string())?;
    let enc_t = PhaseTimer::start();
    let encoded = encode_sensor(blocks, h, w, &maps, &reg).map_err(str::to_string)?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!(
            "sensor encode (codec ID {}): {:.3}s",
            format::CODEC_ID,
            enc_t.elapsed().as_secs_f64()
        ),
    )
    .map_err(|e| e.to_string())?;
    let mapraw = [maps.as_slice(), reg.as_slice()].concat();
    let side_raw = [&raw[..off], &raw[off + n..]].concat();
    let xzt = PhaseTimer::start();
    let map = xz_encode(&mapraw)?;
    let side = xz_encode(&side_raw)?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!("XZ map+metadata: {:.3}s", xzt.elapsed().as_secs_f64()),
    )
    .map_err(|e| e.to_string())?;
    let mut out =
        Vec::with_capacity(format::FIXED_HEADER_LEN + map.len() + side.len() + encoded.len());
    out.extend_from_slice(format::MAGIC);
    out.push(format::FORMAT_VERSION);
    out.extend_from_slice(&format::CODEC_ID.to_le_bytes());
    put_u32(&mut out, w as u32);
    put_u32(&mut out, h as u32);
    for v in [off, n, raw.len(), map.len(), side.len(), encoded.len()] {
        put_u64(&mut out, v as u64);
    }
    out.extend_from_slice(&sha256(raw));
    out.extend_from_slice(&sha256(&mapraw));
    out.extend_from_slice(&map);
    out.extend_from_slice(&side);
    out.extend_from_slice(&encoded);
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!(
            "pack total: {:.3}s, {} -> {} bytes",
            timer.elapsed().as_secs_f64(),
            raw.len(),
            out.len()
        ),
    )
    .map_err(|e| e.to_string())?;
    Ok(out)
}

/// Restore and integrity-check one archive before returning any file bytes.
///
/// Header identifiers, section lengths, helper-table digest, sensor-stream
/// termination, and final original-file digest are all checked.
pub fn unpack_with_level(archive: &[u8], level: Level) -> Result<Vec<u8>, String> {
    let timer = Instant::now();
    if archive.len() < format::FIXED_HEADER_LEN
        || archive.len() > MAX_PAYLOAD
        || &archive[..4] != format::MAGIC
    {
        return Err("invalid AAA1 archive".into());
    }
    let version = archive[4];
    let flags = u16::from_le_bytes(archive[5..7].try_into().unwrap());
    if version != format::FORMAT_VERSION || flags != format::CODEC_ID {
        return Err(format!(
            "decoder supports format version {} and codec ID {} only",
            format::FORMAT_VERSION,
            format::CODEC_ID
        ));
    }
    let w = u32::from_le_bytes(archive[7..11].try_into().unwrap()) as usize;
    let h = u32::from_le_bytes(archive[11..15].try_into().unwrap()) as usize;
    let mut lengths = [0usize; 6];
    for (i, v) in lengths.iter_mut().enumerate() {
        let p = 15 + 8 * i;
        *v = u64::from_le_bytes(archive[p..p + 8].try_into().unwrap()) as usize;
    }
    let [off, n, raw_len, map_len, side_len, stream_len] = lengths;
    if w < 32
        || w % 32 != 0
        || h < 2
        || h % 2 != 0
        || n != w.checked_mul(h).ok_or("geometry overflow")?
        || off + n > raw_len
        || raw_len > MAX_MEMBER
        || format::FIXED_HEADER_LEN
            .checked_add(map_len)
            .and_then(|x| x.checked_add(side_len))
            .and_then(|x| x.checked_add(stream_len))
            != Some(archive.len())
    {
        return Err("invalid AAA1 lengths".into());
    }
    let cut = format::FIXED_HEADER_LEN + map_len;
    let mapraw = xz_decode(&archive[format::FIXED_HEADER_LEN..cut], MAP_SIZE + 304)?;
    if sha256(&mapraw) != archive[95..127] {
        return Err("map digest mismatch".into());
    }
    if &mapraw[MAP_SIZE..MAP_SIZE + 4] != b"RG2\0" {
        return Err("unsupported regression table".into());
    }
    let maps = &mapraw[..MAP_SIZE];
    let reg = &mapraw[MAP_SIZE..];
    let side = xz_decode(&archive[cut..cut + side_len], raw_len - n)?;
    let blocks = &archive[cut + side_len..];
    let dec_t = PhaseTimer::start();
    let decoded = decode_sensor(blocks, h, w, maps, reg).map_err(str::to_string)?;
    let mut raw = Vec::with_capacity(raw_len);
    raw.extend_from_slice(&side[..off]);
    raw.extend_from_slice(&decoded);
    raw.extend_from_slice(&side[off..]);
    if sha256(&raw) != archive[63..95] {
        return Err("original SHA-256 mismatch".into());
    }
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!("sensor decode: {:.3}s", dec_t.elapsed().as_secs_f64()),
    )
    .map_err(|e| e.to_string())?;
    log_to(
        &mut io::stdout(),
        level,
        Level::Info,
        &format!(
            "unpack total: {:.3}s, {} bytes SHA-256 verified",
            timer.elapsed().as_secs_f64(),
            raw.len()
        ),
    )
    .map_err(|e| e.to_string())?;
    Ok(raw)
}

/// Compress a supported Sony ARW2 byte slice into an archive.
///
/// This quiet convenience wrapper performs the same validation as
/// [`pack_with_level`] but suppresses progress output.
pub fn pack_archive(raw: &[u8]) -> Result<Vec<u8>, String> {
    pack_with_level(raw, Level::Quiet)
}

/// Restore an archive and validate its embedded integrity digests.
///
/// This quiet convenience wrapper performs the same checks as
/// [`unpack_with_level`] but suppresses progress output.
pub fn unpack_archive(archive: &[u8]) -> Result<Vec<u8>, String> {
    unpack_with_level(archive, Level::Quiet)
}

#[cfg(test)]
mod tests {
    use super::{pack_archive, sha256, unpack_archive};
    use crate::format;

    #[test]
    fn sha256_uses_standard_digest() {
        assert_eq!(
            sha256(b"abc"),
            [
                0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae,
                0x22, 0x23, 0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61,
                0xf2, 0x00, 0x15, 0xad
            ]
        );
    }

    #[test]
    fn versioned_archive_round_trips_complete_tiff() {
        let rows = 6usize;
        let strip_len = rows * 32;
        let strip_offset = 64usize;
        let entries = [
            (259u16, 3u16, 1u32, 32767u32),
            (273, 4, 1, strip_offset as u32),
            (257, 4, 1, rows as u32),
            (279, 4, 1, strip_len as u32),
        ];
        let mut raw = b"II*\0".to_vec();
        raw.extend_from_slice(&8u32.to_le_bytes());
        raw.extend_from_slice(&(entries.len() as u16).to_le_bytes());
        for (tag, kind, count, value) in entries {
            raw.extend_from_slice(&tag.to_le_bytes());
            raw.extend_from_slice(&kind.to_le_bytes());
            raw.extend_from_slice(&count.to_le_bytes());
            raw.extend_from_slice(&value.to_le_bytes());
        }
        raw.extend_from_slice(&0u32.to_le_bytes());
        raw.resize(strip_offset, 0);
        raw.extend((0..strip_len).map(|i| (i * 29 + 7) as u8));

        let archive = pack_archive(&raw).unwrap();
        assert_eq!(&archive[..4], format::MAGIC);
        assert_eq!(archive[4], format::FORMAT_VERSION);
        assert_eq!(
            u16::from_le_bytes(archive[5..7].try_into().unwrap()),
            format::CODEC_ID
        );
        assert_eq!(unpack_archive(&archive).unwrap(), raw);
    }
}
