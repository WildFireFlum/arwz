//! Lossless archiver for supported Sony ARW2 packed sensor strips.
//!
//! The sensor codec models the camera's existing 16-byte ARW2 blocks and
//! entropy-codes their fields without changing them. The archive preserves the
//! bytes around the strip and verifies the complete restored file. “Lossless”
//! means byte-for-byte restoration of the input ARW; it does not recover
//! information already discarded by lossy camera compression.
//!
//! [`archive`] exposes complete in-memory archive operations. The crate also
//! builds C-compatible libraries; see `include/arw_archive.h`.

pub mod archive;
pub mod ffi;
pub mod format;

use std::io::{self, Write};
use std::time::{Duration, Instant};

/// Two adaptive counts used to estimate the probability of a one bit.
///
/// The counts move toward observed bits at different rates so the estimate
/// responds to recent changes while retaining longer-term history.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Probability {
    pub a: u16,
    pub b: u16,
}
impl Probability {
    /// Start with an even chance for either bit value.
    pub const fn initial() -> Self {
        Self { a: 32768, b: 32768 }
    }
    /// Code or decode one bit and update this context from the observed bit.
    fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        bit: u8,
    ) -> Result<u8, &'static str> {
        let p = ((self.a as u32 + self.b as u32) >> 1).clamp(64, 65535 - 64);
        let decoded = coder.bit(p, bit)?;
        if decoded != 0 {
            self.a = self.a.wrapping_add((65535 - self.a) >> 4);
            self.b = self.b.wrapping_add((65535 - self.b) >> 7);
        } else {
            self.a = self.a.wrapping_sub(self.a >> 4);
            self.b = self.b.wrapping_sub(self.b >> 7);
        }
        Ok(decoded)
    }
}

/// 32-bit arithmetic-style coder shared by the sensor encoder and decoder.
///
/// Each bit partitions the current inclusive interval using a fixed-point
/// probability of one. When the interval's leading byte is known, the encoder
/// emits it and the decoder consumes the corresponding byte. The const
/// parameter selects the direction while keeping both paths in one routine.
pub struct RangeCoder<'a, const ENCODING: bool> {
    low: u32,
    high: u32,
    code: u32,
    input: &'a [u8],
    output: Vec<u8>,
    position: usize,
    #[cfg(feature = "profile-counters")]
    profile_bit_calls: u64,
    #[cfg(feature = "profile-counters")]
    profile_renormalizations: u64,
}
impl RangeCoder<'static, true> {
    pub fn encoder() -> Self {
        Self::encoder_with_capacity(0)
    }
    pub fn encoder_with_capacity(capacity: usize) -> Self {
        Self {
            low: 0,
            high: u32::MAX,
            code: 0,
            input: &[],
            output: Vec::with_capacity(capacity),
            position: 0,
            #[cfg(feature = "profile-counters")]
            profile_bit_calls: 0,
            #[cfg(feature = "profile-counters")]
            profile_renormalizations: 0,
        }
    }
}

impl<'a> RangeCoder<'a, false> {
    pub fn decoder(input: &'a [u8]) -> Self {
        let mut code = 0u32;
        for i in 0..4 {
            code = (code << 8) | input.get(i).copied().unwrap_or(0) as u32;
        }
        Self {
            low: 0,
            high: u32::MAX,
            code,
            input,
            output: Vec::new(),
            position: 4,
            #[cfg(feature = "profile-counters")]
            profile_bit_calls: 0,
            #[cfg(feature = "profile-counters")]
            profile_renormalizations: 0,
        }
    }
}

impl<'a, const ENCODING: bool> RangeCoder<'a, ENCODING> {
    fn for_sensor(source_len: usize, stream: &'a [u8]) -> Self {
        let mut code = 0u32;
        if !ENCODING {
            for i in 0..4 {
                code = (code << 8) | stream.get(i).copied().unwrap_or(0) as u32;
            }
        }
        Self {
            low: 0,
            high: u32::MAX,
            code,
            input: if ENCODING { &[] } else { stream },
            output: if ENCODING {
                Vec::with_capacity(source_len)
            } else {
                Vec::new()
            },
            position: if ENCODING { 0 } else { 4 },
            #[cfg(feature = "profile-counters")]
            profile_bit_calls: 0,
            #[cfg(feature = "profile-counters")]
            profile_renormalizations: 0,
        }
    }

    /// Code one bit whose probability of being one is a 16-bit fixed-point value.
    ///
    /// Both directions calculate the same split. Encoding chooses the half
    /// from `bit`; decoding chooses the half containing `code`. Shared leading
    /// bytes are then shifted out to keep useful precision in the interval.
    fn bit(&mut self, probability_one: u32, bit: u8) -> Result<u8, &'static str> {
        #[cfg(feature = "profile-counters")]
        {
            self.profile_bit_calls += 1;
        }
        let mid = self.low.wrapping_add(
            (((self.high.wrapping_sub(self.low) as u64) * probability_one as u64) >> 16) as u32,
        );
        let bit = if ENCODING {
            bit
        } else {
            u8::from(self.code <= mid)
        };
        if bit != 0 {
            self.high = mid;
        } else {
            self.low = mid.wrapping_add(1);
        }
        while ((self.low ^ self.high) & 0xff00_0000) == 0 {
            #[cfg(feature = "profile-counters")]
            {
                self.profile_renormalizations += 1;
            }
            if ENCODING {
                self.output.push((self.high >> 24) as u8);
            } else {
                self.code =
                    (self.code << 8) | self.input.get(self.position).copied().unwrap_or(0) as u32;
                self.position += 1;
            }
            self.low <<= 8;
            self.high = (self.high << 8) | 255;
        }
        Ok(bit)
    }
    pub fn finish(mut self) -> Vec<u8> {
        if ENCODING {
            for _ in 0..4 {
                self.output.push((self.low >> 24) as u8);
                self.low <<= 8;
            }
            self.output
        } else {
            Vec::new()
        }
    }
    fn decoder_at_end(&self) -> bool {
        !ENCODING && self.position == self.input.len() && self.code == self.low
    }
}

/// Adaptive binary contexts used for signed block-header residuals.
///
/// Non-negative magnitudes use a gamma-style prefix and adaptively coded
/// leading bits; nonzero values then carry a separately modeled sign.
#[derive(Clone)]
pub struct GammaContext {
    unary: [Probability; 20],
    first: [Probability; 21],
    second: [Probability; 42],
    third: [Probability; 84],
    sign: Probability,
}

#[derive(Clone)]
struct HeaderStump {
    // Counts of residual classes on either side of each candidate bit test.
    counts: [[[u16; 7]; 2]; 8],
    seen: u16,
    // Chosen after the first 4096 interior samples; -1 means no useful split.
    split: i8,
    leaves: [GammaContext; 2],
}

#[derive(Clone, Copy, Default)]
struct HeaderProfile {
    // Summary of a 16-sample causal neighborhood, used to select header models.
    mean: i32,
    mad: i32,
    trimmed: i32,
    adjacent: i32,
    minimum: i32,
    second: i32,
}

/// Online correction learner that compares corrected prediction error with
/// the uncorrected prediction before the correction is used.
#[derive(Clone, Default)]
struct HeaderLearner {
    weights: [i32; 6],
    native_error: i64,
    corrected_error: i64,
}

impl HeaderLearner {
    fn candidate(&self, base: i32, features: &[i32; 6]) -> i32 {
        let sum: i64 = self
            .weights
            .iter()
            .zip(features)
            .map(|(&w, &f)| w as i64 * f as i64)
            .sum();
        base + round_div_i64(sum, 4096).clamp(-2, 2) as i32
    }

    fn update(&mut self, base: i32, candidate: i32, actual: i32, features: &[i32; 6]) {
        let denom = 4096i64 + features.iter().map(|&f| f as i64 * f as i64).sum::<i64>();
        let error = actual - candidate;
        for (weight, &feature) in self.weights.iter_mut().zip(features) {
            *weight = (*weight + round_div_i64(128 * error as i64 * feature as i64, denom) as i32)
                .clamp(-8192, 8192);
        }
        self.native_error =
            self.native_error - (self.native_error >> 4) + (actual - base).unsigned_abs() as i64;
        self.corrected_error =
            self.corrected_error - (self.corrected_error >> 4) + error.unsigned_abs() as i64;
    }
}

fn round_div_i64(x: i64, d: i64) -> i64 {
    if x < 0 {
        -((-x + d / 2) / d)
    } else {
        (x + d / 2) / d
    }
}

fn header_profile(row: &[u16], base: usize, lut: &[usize; 2048]) -> HeaderProfile {
    let mut samples = [0i32; 16];
    let mut sorted = [0i32; 16];
    let mut sum = 0;
    let mut differences = 0;
    for i in 0..16 {
        let value = lut[row[base + 2 * i] as usize] as i32;
        samples[i] = value;
        sum += value;
        if i > 0 {
            differences += (value - samples[i - 1]).abs();
        }
        let mut j = i;
        while j > 0 && sorted[j - 1] > value {
            sorted[j] = sorted[j - 1];
            j -= 1;
        }
        sorted[j] = value;
    }
    let mean = (sum + 8) / 16;
    let mad = samples.iter().map(|&v| (v - mean).abs()).sum::<i32>();
    HeaderProfile {
        mean,
        mad: (mad + 8) / 16,
        trimmed: sorted[14] - sorted[1],
        adjacent: (differences + 7) / 15,
        minimum: sorted[0],
        second: sorted[1],
    }
}

impl Default for HeaderStump {
    fn default() -> Self {
        Self {
            counts: [[[0; 7]; 2]; 8],
            seen: 0,
            split: -1,
            leaves: [GammaContext::default(), GammaContext::default()],
        }
    }
}

impl HeaderStump {
    fn class(residual: i32) -> usize {
        if residual == 0 {
            return 0;
        }
        let magnitude = residual.unsigned_abs();
        let bin = if magnitude <= 2 {
            0
        } else if magnitude <= 7 {
            1
        } else {
            2
        };
        1 + 2 * bin + usize::from(residual < 0)
    }

    fn observe(&mut self, base: &GammaContext, mask: u8, residual: i32) {
        if self.seen >= 4096 {
            return;
        }
        let class = Self::class(residual);
        for test in 0..8 {
            self.counts[test][((mask >> test) & 1) as usize][class] += 1;
        }
        self.seen += 1;
        if self.seen != 4096 {
            return;
        }
        // Select a binary split only when it separates the residual classes
        // better than the unsplit model; then clone its trained contexts.
        let mut best = 4u64 * 4096;
        let mut winner = None;
        for test in 0..8 {
            let left: u64 = self.counts[test][0].iter().map(|&v| v as u64).sum();
            let right: u64 = self.counts[test][1].iter().map(|&v| v as u64).sum();
            if left < 128 || right < 128 {
                continue;
            }
            let mut score = 0u64;
            for class in 0..7 {
                let d = left as i64 * self.counts[test][1][class] as i64
                    - right as i64 * self.counts[test][0][class] as i64;
                score += (d * d) as u64 / (left * right);
            }
            if score > best {
                best = score;
                winner = Some(test as i8);
            }
        }
        if let Some(test) = winner {
            self.split = test;
            self.leaves = [base.clone(), base.clone()];
        }
    }
}

/// Adaptive residual contexts for block range and minimum endpoint ranks.
struct HeaderModels {
    range: [GammaContext; 36],
    minimum: [GammaContext; 36],
    stumps: [HeaderStump; 72],
}

impl Default for HeaderModels {
    fn default() -> Self {
        Self {
            range: std::array::from_fn(|_| GammaContext::default()),
            minimum: std::array::from_fn(|_| GammaContext::default()),
            stumps: std::array::from_fn(|_| HeaderStump::default()),
        }
    }
}

impl HeaderModels {
    fn signed<const ENCODING: bool>(
        coder: &mut RangeCoder<'_, ENCODING>,
        context: &mut GammaContext,
        encoded_value: i32,
    ) -> Result<i32, &'static str> {
        if ENCODING {
            context.encode_signed(coder, encoded_value)?;
            Ok(encoded_value)
        } else {
            context.decode_signed(coder)
        }
    }

    fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        field: usize,
        context_index: usize,
        interior: bool,
        mask: u8,
        residual: i32,
    ) -> Result<i32, &'static str> {
        // Interior blocks can use a learned binary split of their causal
        // features. Boundary blocks and unsplit contexts use the shared model.
        let index = field * 36 + context_index;
        let result = if interior && self.stumps[index].split >= 0 {
            let leaf = ((mask >> self.stumps[index].split) & 1) as usize;
            Self::signed(coder, &mut self.stumps[index].leaves[leaf], residual)?
        } else if field == 0 {
            Self::signed(coder, &mut self.range[context_index], residual)?
        } else {
            Self::signed(coder, &mut self.minimum[context_index], residual)?
        };
        let base = if field == 0 {
            &self.range[context_index]
        } else {
            &self.minimum[context_index]
        };
        if interior {
            self.stumps[index].observe(base, mask, result);
        }
        Ok(result)
    }
}

/// Small helpers shared by the predictor and residual models.
pub fn bucket8(x: i32) -> usize {
    if x <= 0 {
        0
    } else {
        (32 - x.leading_zeros() as usize).min(7)
    }
}

#[inline]
fn div_u64_by_u32(numerator: u64, denominator: u32) -> u32 {
    debug_assert_ne!(denominator, 0);
    debug_assert!(numerator <= denominator as u64 * u32::MAX as u64);
    #[cfg(target_arch = "x86_64")]
    {
        let low = numerator as u32;
        let high = (numerator >> 32) as u32;
        let quotient: u32;
        // x86-64's 32-bit DIV consumes the 64-bit numerator in EDX:EAX and
        // avoids a general 64-bit divide when the quotient fits in u32.
        unsafe {
            std::arch::asm!(
                "div {divisor:e}",
                divisor = in(reg) denominator,
                inout("eax") low => quotient,
                in("edx") high,
                options(pure, nomem, nostack)
            );
        }
        quotient
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        (numerator / denominator as u64) as u32
    }
}

#[inline]
fn allowed_count_before(mask: [u64; 2], limit: usize) -> usize {
    debug_assert!(limit <= 128);
    let lower = if limit >= 64 {
        mask[0].count_ones() as usize
    } else {
        (mask[0] & ((1u64 << limit) - 1)).count_ones() as usize
    };
    if limit <= 64 {
        lower
    } else {
        let upper_limit = limit - 64;
        let upper_mask = if upper_limit == 64 {
            u64::MAX
        } else {
            (1u64 << upper_limit) - 1
        };
        lower + (mask[1] & upper_mask).count_ones() as usize
    }
}

pub fn bucket12(mut x: i32) -> usize {
    let mut n = 0;
    while x > 0 && n < 11 {
        x >>= 1;
        n += 1;
    }
    n
}
pub fn round_div(x: i32, d: i32) -> i32 {
    if x < 0 {
        -((-x + d / 2) / d)
    } else {
        (x + d / 2) / d
    }
}
pub fn rank_of(q: i32, p: i32, hi: i32) -> i32 {
    if q == p {
        return 0;
    }
    let d = (q - p).abs();
    let t = d - 1;
    let mut r = 1 + t.min(p) + t.min(hi - p);
    if q < p && p + d <= hi {
        r += 1;
    }
    r
}
pub fn unrank(r: i32, p: i32, hi: i32) -> i32 {
    if r == 0 {
        return p;
    }
    let (mut lo, mut h) = (1, hi);
    while lo < h {
        let d = (lo + h) / 2;
        let n = d.min(p) + d.min(hi - p);
        if n >= r {
            h = d
        } else {
            lo = d + 1
        }
    }
    let d = lo;
    let before = (d - 1).min(p) + (d - 1).min(hi - p);
    if p + d <= hi && r == before + 1 {
        p + d
    } else {
        p - d
    }
}

pub struct Tree4 {
    nodes: [Probability; 31],
}
impl Default for Tree4 {
    fn default() -> Self {
        Self {
            nodes: [Probability::initial(); 31],
        }
    }
}
impl Tree4 {
    pub fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        value: u8,
    ) -> Result<u8, &'static str> {
        if value > 15 {
            return Err("tree4 symbol outside 0..15");
        }
        let mut node = 1usize;
        for bit in (0..4).rev() {
            let input = if ENCODING { (value >> bit) & 1 } else { 0 };
            node = node * 2 + self.nodes[node].code(coder, input)? as usize;
        }
        Ok((node - 16) as u8)
    }
}

pub struct QuantizedModel {
    contexts: [[Probability; 16]; 8],
}

/// Adaptive cumulative-frequency model for bounded sensor differences.
///
/// Contexts combine residual size, block scale and row phase. A symbol is
/// coded by splitting its bounded range around the prediction; the observed
/// frequencies choose the probability of each split. Batched updates and
/// periodic count decay let the model adapt without unbounded history.
pub struct CdfModel {
    hist: [[u16; 255]; 64],
    prefix: [[u32; 256]; 64],
    pending: [u16; 64],
    #[cfg(feature = "profile-counters")]
    profile_calls: u64,
    #[cfg(feature = "profile-counters")]
    profile_search_steps: u64,
    #[cfg(feature = "profile-counters")]
    profile_rebuilds: u64,
    #[cfg(feature = "profile-counters")]
    profile_decay_rebuilds: u64,
    #[cfg(feature = "profile-counters")]
    profile_forced_bits: u64,
}

/// Adaptive model for the maximum and minimum sample positions in each block.
///
/// It codes each position by subdividing the 16 choices. An adaptive tree is
/// always available; when neighboring samples give a useful ranking, its
/// observed frequencies are blended with the tree estimate. The blend adapts
/// to which estimate has been more reliable. The second position excludes the
/// first so the extrema cannot collide.
pub struct PositionModel {
    native: [[[Probability; 16]; 2]; 2],
    same: [Probability; 2],
    rank_count: [[[[u16; 16]; 2]; 4]; 4],
    weight: [[[[u32; 4]; 2]; 4]; 4],
}

impl Default for PositionModel {
    fn default() -> Self {
        Self {
            native: [[[Probability::initial(); 16]; 2]; 2],
            same: [Probability::initial(); 2],
            rank_count: [[[[1; 16]; 2]; 4]; 4],
            weight: [[[[32768; 4]; 2]; 4]; 4],
        }
    }
}

impl PositionModel {
    fn clamp_probability(p: u32) -> u32 {
        p.clamp(64, 65471)
    }

    fn index<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        z: usize,
        phase: usize,
        bucket: Option<usize>,
        kind: usize,
        ranks: &[usize; 16],
        excluded: Option<usize>,
        value: usize,
    ) -> Result<usize, &'static str> {
        let (mut node, mut lo, mut hi) = (1usize, 0usize, 16usize);
        for depth in 0..4 {
            let mid = (lo + hi) / 2;
            let nl = mid - lo - usize::from(excluded.is_some_and(|e| e >= lo && e < mid));
            let nr = hi - mid - usize::from(excluded.is_some_and(|e| e >= mid && e < hi));
            let bit = if nl == 0 || nr == 0 {
                u8::from(nl == 0)
            } else if let Some(bucket) = bucket {
                let (mut left, mut right) = (0u64, 0u64);
                for i in lo..hi {
                    if Some(i) == excluded {
                        continue;
                    }
                    let count = self.rank_count[phase][bucket][kind][ranks[i]] as u64;
                    if i < mid {
                        left += count;
                    } else {
                        right += count;
                    }
                }
                let pe = Self::clamp_probability(((right << 16) / (left + right)) as u32);
                let p = &mut self.native[z][kind][node];
                let pn = Self::clamp_probability((p.a as u32 + p.b as u32) >> 1);
                let w = self.weight[phase][bucket][kind][depth] as u64;
                let prob = Self::clamp_probability(
                    ((w * pe as u64 + (65536 - w) * pn as u64 + 32768) >> 16) as u32,
                );
                let bit = coder.bit(prob, if ENCODING { u8::from(value >= mid) } else { 0 })?;
                if bit != 0 {
                    p.a = p.a.wrapping_add((65535 - p.a) >> 4);
                    p.b = p.b.wrapping_add((65535 - p.b) >> 7);
                } else {
                    p.a = p.a.wrapping_sub(p.a >> 4);
                    p.b = p.b.wrapping_sub(p.b >> 7);
                }
                let a = w * if bit != 0 {
                    pe as u64
                } else {
                    65536 - pe as u64
                };
                let b = (65536 - w)
                    * if bit != 0 {
                        pn as u64
                    } else {
                        65536 - pn as u64
                    };
                let posterior = (a << 16) / (a + b);
                self.weight[phase][bucket][kind][depth] =
                    ((posterior as u32 * 255 + 32768 + 128) >> 8).min(65535);
                bit
            } else {
                let p = &mut self.native[z][kind][node];
                p.code(coder, if ENCODING { u8::from(value >= mid) } else { 0 })?
            };
            node = node * 2 + bit as usize;
            if bit != 0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        Ok(lo)
    }

    pub fn code_pair<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        phase: usize,
        range: i32,
        scores: Option<&[i32; 16]>,
        score_scale: i32,
        input_ix: usize,
        input_iy: usize,
    ) -> Result<(usize, usize), &'static str> {
        if phase > 3 || input_ix >= 16 || input_iy >= 16 {
            return Err("invalid position model input");
        }
        let mut bucket = None;
        let mut ranks = [[0usize; 16]; 2];
        if let Some(scores) = scores {
            let (min, max) = scores.iter().fold((scores[0], scores[0]), |(lo, hi), &v| {
                (lo.min(v), hi.max(v))
            });
            let span = max - min;
            if span != 0 {
                let norm = range.unsigned_abs() as i64 * score_scale as i64 + 1;
                bucket = Some(if span as i64 * 4 < norm {
                    0
                } else if span as i64 * 2 < norm {
                    1
                } else if (span as i64) < norm {
                    2
                } else {
                    3
                });
                let mut order = std::array::from_fn::<_, 16, _>(|i| i);
                for i in 1..16 {
                    let index = order[i];
                    let mut j = i;
                    while j > 0
                        && (scores[order[j - 1]] > scores[index]
                            || (scores[order[j - 1]] == scores[index] && order[j - 1] > index))
                    {
                        order[j] = order[j - 1];
                        j -= 1;
                    }
                    order[j] = index;
                }
                let mut start = 0;
                while start < 16 {
                    let mut end = start + 1;
                    while end < 16 && scores[order[end]] == scores[order[start]] {
                        end += 1;
                    }
                    for rank in start..end {
                        let index = order[rank];
                        let tied_before = rank - start;
                        ranks[0][index] = 16 - end + tied_before;
                        ranks[1][index] = start + tied_before;
                    }
                    start = end;
                }
            }
        }
        let z = usize::from(range <= 0);
        let x = self.index(coder, z, phase, bucket, 0, &ranks[0], None, input_ix)?;
        let same = self.same[z].code(coder, u8::from(ENCODING && input_iy == x))? != 0;
        let y = if same {
            x
        } else {
            self.index(coder, z, phase, bucket, 1, &ranks[1], Some(x), input_iy)?
        };
        if let Some(bucket) = bucket {
            for kind in 0..2 {
                let index = if kind == 0 { x } else { y };
                let hist = &mut self.rank_count[phase][bucket][kind];
                if hist[ranks[kind][index]] >= 4095 {
                    for v in hist.iter_mut() {
                        *v = (*v + 1) >> 1;
                    }
                }
                hist[ranks[kind][index]] += 1;
            }
        }
        Ok((x, y))
    }
}

impl Default for CdfModel {
    fn default() -> Self {
        let mut model = Self {
            hist: [[1; 255]; 64],
            prefix: [[0; 256]; 64],
            pending: [0; 64],
            #[cfg(feature = "profile-counters")]
            profile_calls: 0,
            #[cfg(feature = "profile-counters")]
            profile_search_steps: 0,
            #[cfg(feature = "profile-counters")]
            profile_rebuilds: 0,
            #[cfg(feature = "profile-counters")]
            profile_decay_rebuilds: 0,
            #[cfg(feature = "profile-counters")]
            profile_forced_bits: 0,
        };
        for ctx in 0..64 {
            model.rebuild(ctx, false);
        }
        model
    }
}

impl CdfModel {
    fn rebuild(&mut self, ctx: usize, decay: bool) {
        let mut sum = 0u32;
        self.prefix[ctx][0] = 0;
        let hist = &mut self.hist[ctx];
        let prefix = &mut self.prefix[ctx];
        if decay {
            for i in 0..255 {
                hist[i] = (hist[i] + 1) >> 1;
                sum += hist[i] as u32;
                prefix[i + 1] = sum;
            }
        } else {
            for i in 0..255 {
                sum += hist[i] as u32;
                prefix[i + 1] = sum;
            }
        }
        self.pending[ctx] = 0;
    }

    pub fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        hi: i32,
        prediction_q4: i32,
        error_q8: i32,
        symbol: i32,
        shift: usize,
        phase: usize,
        gradient: i32,
        brightness: usize,
        texture: usize,
    ) -> Result<i32, &'static str> {
        self.code_bounded(
            coder,
            hi,
            prediction_q4,
            error_q8,
            symbol,
            shift,
            phase,
            gradient,
            brightness,
            texture,
            false,
            false,
        )
    }

    pub fn code_bounded<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        hi: i32,
        prediction_q4: i32,
        error_q8: i32,
        symbol: i32,
        shift: usize,
        phase: usize,
        gradient: i32,
        brightness: usize,
        texture: usize,
        drop_low: bool,
        drop_high: bool,
    ) -> Result<i32, &'static str> {
        self.code_bounded_lite(
            coder,
            hi,
            prediction_q4,
            error_q8,
            symbol,
            shift,
            phase,
            gradient,
            brightness,
            texture,
            drop_low,
            drop_high,
        )
    }

    fn code_bounded_lite<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        hi: i32,
        prediction_q4: i32,
        error_q8: i32,
        symbol: i32,
        shift: usize,
        phase: usize,
        _gradient: i32,
        brightness: usize,
        texture: usize,
        drop_low: bool,
        drop_high: bool,
    ) -> Result<i32, &'static str> {
        #[cfg(feature = "profile-counters")]
        {
            self.profile_calls += 1;
        }
        if !(0..=254).contains(&hi)
            || shift > 4
            || phase > 3
            || brightness > 7
            || texture > 3
            || (ENCODING && !(0..=hi).contains(&symbol))
        {
            return Err("invalid CDF residual parameters");
        }
        let center = ((prediction_q4 + 2) / 4).clamp(0, hi);
        let scale = bucket8(if error_q8 > 0 { error_q8 / 8 } else { 0 });
        let ctx = (scale * 4 + shift.min(3)) * 2 + (phase & 1);
        let mut lo = 0i32;
        let mut upper = hi;
        let prefix = &self.prefix[ctx];
        let left = (lo - center + 127) as usize;
        let mut end = (upper - center + 128) as usize;
        let mut total = prefix[end] - prefix[left];

        while lo < upper {
            #[cfg(feature = "profile-counters")]
            {
                self.profile_search_steps += 1;
            }
            let mid = (lo + upper + 1) >> 1;
            let split = (mid - center + 127) as usize;
            if total == 0 {
                return Err("empty adaptive CDF interval");
            }
            let high = prefix[end] - prefix[split];
            debug_assert!(total <= u16::MAX as u32);
            let probability = ((high << 16) / total).clamp(64, 65471);
            let forced = if drop_low && mid <= 1 {
                Some(1)
            } else if drop_high && mid == hi {
                Some(0)
            } else {
                None
            };
            #[cfg(feature = "profile-counters")]
            if forced.is_some() {
                self.profile_forced_bits += 1;
            }
            if let Some(bit) = forced {
                if ENCODING && (symbol >= mid) != (bit != 0) {
                    return Err("symbol violates forced CDF bound");
                }
            }
            let bit = if let Some(bit) = forced {
                bit
            } else {
                coder.bit(
                    probability,
                    if ENCODING { u8::from(symbol >= mid) } else { 0 },
                )?
            };
            if bit != 0 {
                lo = mid;
                total = high;
            } else {
                upper = mid - 1;
                end = split;
                total -= high;
            }
        }

        let bin = (lo - center + 127) as usize;
        self.hist[ctx][bin] += 1;
        self.pending[ctx] += 1;
        if self.pending[ctx] == 64 {
            let decay = self.prefix[ctx][255] >= 4096;
            #[cfg(feature = "profile-counters")]
            {
                self.profile_rebuilds += 1;
                self.profile_decay_rebuilds += u64::from(decay);
            }
            self.rebuild(ctx, decay);
        }
        Ok(lo)
    }
}

impl Default for QuantizedModel {
    fn default() -> Self {
        Self {
            contexts: [[Probability::initial(); 16]; 8],
        }
    }
}
impl QuantizedModel {
    /// Adaptive interval-halving model for a bounded quantized symbol.
    ///
    /// This helper is separate from the cumulative-frequency model used by
    /// the current sensor pass.
    pub fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        hi: i32,
        pq4: i32,
        eq8: i32,
        q: i32,
    ) -> Result<i32, &'static str> {
        let (mut lo, mut upper) = (0, hi);
        while lo < upper {
            let mid = (lo + upper + 1) >> 1;
            let dz = pq4 - 4 * mid + 2;
            let bucket = if dz == 0 {
                0
            } else {
                let az = dz.abs() * 8 / (eq8 + 8);
                let lv = if az < 1 {
                    0
                } else if az < 2 {
                    1
                } else if az < 4 {
                    2
                } else if az < 6 {
                    3
                } else if az < 8 {
                    4
                } else if az < 12 {
                    5
                } else {
                    6
                };
                (if dz > 0 { 1 } else { 8 }) + lv
            };
            let bit = if ENCODING { u8::from(q >= mid) } else { 0 };
            let b =
                self.contexts[bucket8((upper - lo + 1) >> 1)][bucket as usize].code(coder, bit)?;
            if b != 0 {
                lo = mid
            } else {
                upper = mid - 1
            }
        }
        Ok(lo)
    }
}

pub struct RankModel {
    unary: [Probability; 8 * 7],
    bits: [Probability; 8 * 8 * 7 * 4],
}
impl Default for RankModel {
    fn default() -> Self {
        Self {
            unary: [Probability::initial(); 8 * 7],
            bits: [Probability::initial(); 8 * 8 * 7 * 4],
        }
    }
}
impl RankModel {
    pub fn code<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        hi: i32,
        prediction_q4: i32,
        error_q8: i32,
        q: i32,
    ) -> Result<i32, &'static str> {
        let p = ((prediction_q4 + 2) / 4).clamp(0, hi);
        let bucket = bucket8(error_q8 >> 3);
        let rank = if ENCODING { rank_of(q, p, hi) } else { 0 };
        let mut maxk = 0;
        while (1 << (maxk + 1)) <= hi + 1 {
            maxk += 1;
        }
        let mut want = 0;
        while ENCODING && (1 << (want + 1)) <= rank + 1 {
            want += 1;
        }
        let (mut k, mut ui) = (0, 0);
        while k < maxk {
            let b = self.unary[bucket * 7 + ui].code(coder, u8::from(ENCODING && k < want))?;
            if b == 0 {
                break;
            }
            k += 1;
            ui += 1;
        }
        let base = (1 << k) - 1;
        let n = (hi - base + 1).min(1 << k);
        let (mut lo, mut upper, mut depth, mut prefix) = (0, n - 1, 0usize, 0usize);
        while lo < upper {
            let mid = (lo + upper + 1) / 2;
            let bit = if ENCODING {
                u8::from(rank - base >= mid)
            } else {
                0
            };
            let index = (((bucket * 8 + k) * 7 + depth) * 4) + (prefix & 3);
            let b = self.bits[index].code(coder, bit)?;
            if b != 0 {
                lo = mid
            } else {
                upper = mid - 1
            }
            prefix = ((prefix << 1) | b as usize) & 3;
            depth += 1;
        }
        Ok(unrank(base + lo, p, hi))
    }
}

impl Default for GammaContext {
    fn default() -> Self {
        Self {
            unary: [Probability::initial(); 20],
            first: [Probability::initial(); 21],
            second: [Probability::initial(); 42],
            third: [Probability::initial(); 84],
            sign: Probability::initial(),
        }
    }
}
impl GammaContext {
    /// Encode a non-negative magnitude with an adaptive gamma-style code.
    pub fn encode<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        value: u32,
    ) -> Result<(), &'static str> {
        if !ENCODING {
            return Err("encoder required");
        }
        let u = value.checked_add(1).ok_or("gamma value overflow")?;
        let nb = 31 - u.leading_zeros() as usize;
        let mut k = 0;
        while self.unary[k].code(coder, u8::from(k < nb))? != 0 {
            k += 1;
            if k >= 20 {
                break;
            }
        }
        let mut prefix = 1u32;
        for j in (0..k).rev() {
            let bit = ((u >> j) & 1) as u8;
            if j == k - 1 {
                self.first[k].code(coder, bit)?;
            } else if j == k - 2 {
                self.second[k * 2 + (prefix as usize & 1)].code(coder, bit)?;
            } else if j == k - 3 {
                self.third[k * 4 + (prefix as usize & 3)].code(coder, bit)?;
            } else {
                coder.bit(32768, bit)?;
            }
            prefix = (prefix << 1) | bit as u32;
        }
        Ok(())
    }
    /// Decode a non-negative magnitude encoded by [`GammaContext::encode`].
    pub fn decode<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
    ) -> Result<u32, &'static str> {
        if ENCODING {
            return Err("decoder required");
        }
        let mut k = 0usize;
        loop {
            if self.unary[k].code(coder, 0)? == 0 {
                break;
            }
            k += 1;
            if k >= 20 {
                break;
            }
        }
        let mut value = 1u32;
        for j in (0..k).rev() {
            let bit = if j == k - 1 {
                self.first[k].code(coder, 0)?
            } else if j == k - 2 {
                self.second[k * 2 + (value as usize & 1)].code(coder, 0)?
            } else if j == k - 3 {
                self.third[k * 4 + (value as usize & 3)].code(coder, 0)?
            } else {
                coder.bit(32768, 0)?
            };
            value = (value << 1) | bit as u32;
        }
        Ok(value - 1)
    }
    pub fn encode_signed<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
        value: i32,
    ) -> Result<(), &'static str> {
        let mag = value.unsigned_abs();
        self.encode(coder, mag)?;
        if mag != 0 {
            self.sign.code(coder, u8::from(value < 0))?;
        }
        Ok(())
    }
    pub fn decode_signed<const ENCODING: bool>(
        &mut self,
        coder: &mut RangeCoder<'_, ENCODING>,
    ) -> Result<i32, &'static str> {
        let mag = self.decode(coder)? as i32;
        if mag == 0 {
            return Ok(0);
        }
        Ok(if self.sign.code(coder, 0)? != 0 {
            -mag
        } else {
            mag
        })
    }
}

fn parse_block(b: &[u8]) -> Result<(i32, i32, [i32; 14]), &'static str> {
    let (mx, mn, _, _, deltas) = parse_sensor_block(b)?;
    Ok((mx, mn, deltas))
}

/// Read the fixed 128-bit ARW2 block fields without expanding the samples.
fn parse_sensor_block(b: &[u8]) -> Result<(i32, i32, usize, usize, [i32; 14]), &'static str> {
    if b.len() != 16 {
        return Err("invalid sensor block");
    }
    let lo = u64::from_le_bytes(b[..8].try_into().unwrap());
    let hi = u64::from_le_bytes(b[8..].try_into().unwrap());
    let mx = (lo & 0x7ff) as i32;
    let mn = ((lo >> 11) & 0x7ff) as i32;
    let ix = ((lo >> 22) & 15) as usize;
    let iy = ((lo >> 26) & 15) as usize;
    let mut deltas = [0; 14];
    for (k, d) in deltas.iter_mut().enumerate() {
        let shift = 30 + 7 * k;
        let v = if shift < 64 {
            let mut x = lo >> shift;
            if shift + 7 > 64 {
                x |= hi << (64 - shift);
            }
            x
        } else {
            hi >> (shift - 64)
        };
        *d = (v & 127) as i32;
    }
    Ok((mx, mn, ix, iy, deltas))
}

/// Serialize endpoint fields and fourteen 7-bit differences back to 16 bytes.
fn store_sensor_block(
    block: &mut [u8],
    mx: i32,
    mn: i32,
    ix: usize,
    iy: usize,
    deltas: &[i32; 14],
) {
    let mut lo = (mx as u64 & 0x7ff)
        | ((mn as u64 & 0x7ff) << 11)
        | ((ix as u64 & 15) << 22)
        | ((iy as u64 & 15) << 26);
    let mut hi = 0u64;
    for (k, &delta) in deltas.iter().enumerate() {
        let bit = 30 + 7 * k;
        let v = (delta as u64) & 127;
        if bit < 64 {
            lo |= v << bit;
        }
        if bit + 7 > 64 {
            hi |= if bit < 64 {
                v >> (64 - bit)
            } else {
                v << (bit - 64)
            };
        }
    }
    block[..8].copy_from_slice(&lo.to_le_bytes());
    block[8..].copy_from_slice(&hi.to_le_bytes());
}

/// Expand one 16-pixel Sony packed block into its 11-bit sample values.
///
/// The block stores its 11-bit minimum and maximum, their positions, and
/// fourteen 7-bit differences. A range-dependent scale is applied to each
/// difference to reconstruct the other samples. This routine is used to make
/// predictor state; the archive still preserves the original packed fields.
pub fn unpack_block(b: &[u8]) -> Result<[u16; 16], &'static str> {
    if b.len() != 16 {
        return Err("invalid sensor block");
    }
    let lo = u64::from_le_bytes(b[..8].try_into().unwrap());
    let hi = u64::from_le_bytes(b[8..].try_into().unwrap());
    let mx = (lo & 0x7ff) as i32;
    let mn = ((lo >> 11) & 0x7ff) as i32;
    let ix = ((lo >> 22) & 15) as usize;
    let iy = ((lo >> 26) & 15) as usize;
    let range = mx - mn;
    let mut shift = 0;
    while shift < 4 && (0x80 << shift) <= range {
        shift += 1;
    }
    let mut result = [0u16; 16];
    let mut d_index = 0usize;
    for (i, pixel) in result.iter_mut().enumerate() {
        if i == ix {
            *pixel = mx as u16;
        } else if i == iy {
            *pixel = mn as u16;
        } else {
            let bit = 30 + 7 * d_index.min(13);
            let packed = if bit < 64 {
                let mut v = lo >> bit;
                if bit + 7 > 64 {
                    v |= hi << (64 - bit);
                }
                v
            } else {
                hi >> (bit - 64)
            };
            let delta = (packed & 127) as i32;
            *pixel = (mn + (delta << shift)).min(2047) as u16;
            d_index = (d_index + 1).min(13);
        }
    }
    Ok(result)
}

/// Expand a complete strip from packed blocks into spatial sample order.
///
/// Each 32-pixel group contains one block for each alternating pixel
/// position. The returned 16-bit values hold the decoded 11-bit samples and
/// are used only as causal predictor state.
pub fn unpack_sensor_strip(
    blocks: &[u8],
    rows: usize,
    width: usize,
) -> Result<Vec<u16>, &'static str> {
    if width < 32
        || width % 32 != 0
        || rows % 2 != 0
        || blocks.len() != rows.checked_mul(width).ok_or("geometry overflow")?
    {
        return Err("invalid sensor geometry");
    }
    let bpr = width / 16;
    let mut pixels = vec![0u16; rows * width];
    for y in 0..rows {
        for g in 0..width / 32 {
            let even = unpack_block(&blocks[(y * bpr + 2 * g) * 16..(y * bpr + 2 * g + 1) * 16])?;
            let odd =
                unpack_block(&blocks[(y * bpr + 2 * g + 1) * 16..(y * bpr + 2 * g + 2) * 16])?;
            for i in 0..16 {
                pixels[y * width + g * 32 + 2 * i] = even[i];
                pixels[y * width + g * 32 + 2 * i + 1] = odd[i];
            }
        }
    }
    Ok(pixels)
}

/// Learn fixed-point corrections to the causal pixel predictor.
///
/// Samples the strip and fits small regularized linear predictors by Bayer
/// phase and local activity. The fixed-size `RG2` table is optional helper
/// data: the codec uses a fitted estimate only when its running prediction
/// error beats the base predictor. It changes coding efficiency, not samples.
pub fn fit_cross_profile(
    blocks: &[u8],
    rows: usize,
    width: usize,
) -> Result<Vec<u8>, &'static str> {
    if width < 32
        || width % 32 != 0
        || rows < 6
        || rows % 2 != 0
        || blocks.len() != rows.checked_mul(width).ok_or("geometry overflow")?
    {
        return Err("invalid regression geometry");
    }
    let pixels = unpack_sensor_strip(blocks, rows, width)?;
    let bpr = width / 16;
    let slopes = 11usize;
    let dimensions = slopes + 1;
    let mut gram = vec![vec![vec![0.0f64; dimensions * dimensions]; 3]; 4];
    let mut rhs = vec![vec![vec![0.0f64; dimensions]; 3]; 4];
    let mut counts = vec![vec![0usize; 3]; 4];
    let mut pairs: Vec<usize> = (4..rows.saturating_sub(1)).step_by(2).collect();
    if pairs.len() > 64 {
        let n = pairs.len();
        pairs = (0..64).map(|i| pairs[i * (n - 1) / 63]).collect();
    }
    for first in pairs {
        for y in [first, first + 1] {
            let row = y;
            for col in 4..width - 2 {
                let group = col / 32;
                let odd = col & 1 != 0;
                let i = (col % 32) / 2;
                let block_index = y * bpr + (col / 32) * 2 + (col & 1);
                let at = block_index * 16;
                let lo = u64::from_le_bytes(blocks[at..at + 8].try_into().unwrap());
                let mx = (lo & 0x7ff) as i32;
                let mn = ((lo >> 11) & 0x7ff) as i32;
                let ix = ((lo >> 22) & 15) as usize;
                let iy = ((lo >> 26) & 15) as usize;
                let rg = mx - mn;
                if rg < 0 || ix == iy || i == ix || i == iy {
                    continue;
                }
                let mid = mn + rg.max(0) / 2;
                let px = |r: usize, c: usize| pixels[r * width + c] as i32;
                let l = px(row, col - 2);
                let a = px(row - 2, col);
                let al = px(row - 2, col - 2);
                let activity = l.max(a).max(al) - l.min(a).min(al);
                let bin = if activity <= 16 {
                    0
                } else if activity <= 63 {
                    1
                } else {
                    2
                };
                let di = (i as i32 - iy as i32).abs();
                let dx = (i as i32 - ix as i32).abs();
                let numerator = rg * (di - dx);
                let denominator = (di + dx).max(1);
                let pos = numerator.signum() * (numerator.abs() / denominator);
                let even_col = group * 32 + 2 * i;
                let next_col = (even_col + 2).min(group * 32 + 30);
                let proxy = |r: usize| (px(r, even_col) + px(r, next_col) + 1) / 2;
                let current = if odd {
                    proxy(row)
                } else {
                    px(row - 1, even_col)
                };
                let previous = if odd {
                    let prev_i = i.saturating_sub(1);
                    let prev_col = group * 32 + 2 * prev_i;
                    (px(row, prev_col) + px(row, (prev_col + 2).min(group * 32 + 30)) + 1) / 2
                } else {
                    px(row - 1, group * 32 + 2 * i.saturating_sub(1))
                };
                let old = if odd {
                    proxy(row - 2)
                } else {
                    px(row - 3, even_col)
                };
                let mean_odd: i32 = (0..16)
                    .map(|j| {
                        let c = group * 32 + 2 * j;
                        (px(row, c) + px(row, (c + 2).min(group * 32 + 30)) + 1) / 2
                    })
                    .sum::<i32>()
                    .wrapping_add(8)
                    / 16;
                let mean_even: i32 = (0..16)
                    .map(|j| px(row - 1, group * 32 + 2 * j))
                    .sum::<i32>()
                    .wrapping_add(8)
                    / 16;
                let mean = if odd { mean_odd } else { mean_even };
                let features = [
                    l - mid,
                    a - mid,
                    al - mid,
                    px(row - 2, col + 2) - mid,
                    px(row, col - 4) - mid,
                    px(row - 4, col) - mid,
                    px(row - 1, col) - px(row - 3, col),
                    pos,
                    current - mean,
                    current - previous,
                    current - old,
                    1,
                ];
                let phase = (y & 1) * 2 + (col & 1);
                counts[phase][bin] += 1;
                let matrix = &mut gram[phase][bin];
                let target = px(row, col) - mid;
                for i in 0..dimensions {
                    rhs[phase][bin][i] += features[i] as f64 * target as f64;
                    for j in 0..dimensions {
                        matrix[i * dimensions + j] += features[i] as f64 * features[j] as f64;
                    }
                }
            }
        }
    }
    let mut output = Vec::with_capacity(304);
    output.extend_from_slice(b"RG2\0");
    for phase in 0..4 {
        for bin in 0..3 {
            if counts[phase][bin] < 32 {
                output.push(0);
                output.extend_from_slice(&[0; 24]);
                continue;
            }
            let n = counts[phase][bin] as f64;
            let matrix = &mut gram[phase][bin];
            for i in 0..slopes {
                let diag = matrix[i * dimensions + i];
                matrix[i * dimensions + i] += (diag * 0.01).max(n * 64.0);
            }
            matrix[slopes * dimensions + slopes] += n * 0.01;
            let beta = solve_linear(matrix, &rhs[phase][bin], dimensions)?;
            output.push(1);
            for weight in beta[..slopes]
                .iter()
                .map(|v| (v * 512.0).round_ties_even().clamp(-1024.0, 1024.0) as i16)
            {
                output.extend_from_slice(&weight.to_le_bytes());
            }
            let bias = beta[slopes].round_ties_even().clamp(-32768.0, 32767.0) as i16;
            output.extend_from_slice(&bias.to_le_bytes());
        }
    }
    Ok(output)
}

fn solve_linear(matrix: &[f64], rhs: &[f64], n: usize) -> Result<Vec<f64>, &'static str> {
    let mut a = matrix.to_vec();
    let mut b = rhs.to_vec();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&x, &y| a[x * n + col].abs().total_cmp(&a[y * n + col].abs()))
            .unwrap();
        if a[pivot * n + col].abs() < 1e-18 {
            return Err("singular regression fit");
        }
        if pivot != col {
            for j in col..n {
                a.swap(col * n + j, pivot * n + j);
            }
            b.swap(col, pivot);
        }
        let d = a[col * n + col];
        for j in col..n {
            a[col * n + j] /= d;
        }
        b[col] /= d;
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = a[row * n + col];
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }
    Ok(b)
}

/// Build observed-value maps for endpoint values and scaled block differences.
///
/// Four phases separate row parity and alternating block position. For each,
/// maps record observed endpoints and differences at each possible scale.
/// These maps let common values use compact ranks; the decoder receives the
/// same maps so it can reverse those ranks unambiguously.
pub fn sensor_maps(blocks: &[u8], rows: usize, width: usize) -> Result<Vec<u8>, &'static str> {
    if rows < 2
        || rows % 2 != 0
        || width < 32
        || width % 32 != 0
        || rows.checked_mul(width) != Some(blocks.len())
        || rows > i64::MAX as usize
        || width / 16 > i64::MAX as usize
    {
        return Err("invalid sensor geometry");
    }
    if blocks.len() / 16 > 64 * 1024 * 1024 / 16 {
        return Err("sensor strip exceeds map limit");
    }
    let bpr = width / 16;
    let mut maps = vec![0; 4 * 6 * 256];
    for y in 0..rows {
        for j in 0..bpr {
            let (mx, mn, ds) = parse_block(&blocks[(y * bpr + j) * 16..(y * bpr + j + 1) * 16])?;
            let phase = (y % 2) * 2 + (j % 2);
            let mark = |maps: &mut [u8], kind: usize, value: i32| {
                let v = value as usize;
                maps[(phase * 6 + kind) * 256 + (v >> 3)] |= 1 << (v & 7);
            };
            mark(&mut maps, 0, mn);
            mark(&mut maps, 0, mx);
            let range = mx - mn;
            let mut shift = 0;
            while shift < 4 && (0x80 << shift) <= range {
                shift += 1;
            }
            let qhi = (range.max(0) >> shift).min(127);
            if range >= 0 && ds.iter().all(|d| *d <= qhi) {
                for d in ds {
                    mark(&mut maps, 1 + shift as usize, mn + (d << shift));
                }
            }
        }
    }
    Ok(maps)
}

/// Encode supported Sony ARW2 packed blocks into the coded sensor stream.
///
/// `maps` and `regression` must describe this same strip. Decoding the
/// resulting stream with those tables reproduces `blocks` byte-for-byte.
pub fn encode_sensor(
    blocks: &[u8],
    rows: usize,
    width: usize,
    maps: &[u8],
    regression: &[u8],
) -> Result<Vec<u8>, &'static str> {
    sensor_codec::<true>(blocks, &[], rows, width, maps, regression)
}

/// Decode a sensor stream into the original Sony packed-block bytes.
///
/// The helper maps and regression table must be the ones used by the encoder.
pub fn decode_sensor(
    stream: &[u8],
    rows: usize,
    width: usize,
    maps: &[u8],
    regression: &[u8],
) -> Result<Vec<u8>, &'static str> {
    sensor_codec::<false>(&[], stream, rows, width, maps, regression)
}

#[cfg(feature = "profile-counters")]
#[derive(Default)]
struct SensorProfileCounters {
    blocks: u64,
    predicted_pixels: u64,
    regression_evaluations: u64,
    compact_blocks: u64,
    escaped_blocks: u64,
    compact_alphabet_entries: u64,
    compact_mapping_lookups: u64,
    position_model_calls: u64,
    predictor_samples: u64,
    predictor_sample_ns: u128,
    cdf_samples: u64,
    cdf_sample_ns: u128,
    position_samples: u64,
    position_sample_ns: u128,
}

/// Shared raster-order encoder/decoder for the supported packed strip.
///
/// The raster scan codes each block's endpoint range and minimum, extremum
/// positions, then its remaining differences. Spatial predictions and
/// observed-value maps reduce the cost of common symbols; escape coding keeps
/// unusual differences exact. Reconstructed samples immediately update the
/// causal predictors and adaptive probability models.
///
/// A five-row ring retains only the sample history needed by the predictors.
/// Encoder and decoder share this routine so every prediction and model update
/// happens in the same order; otherwise later arithmetic-code decisions would
/// diverge.
fn sensor_codec<const ENCODING: bool>(
    source: &[u8],
    stream: &[u8],
    rows: usize,
    width: usize,
    maps: &[u8],
    regression: &[u8],
) -> Result<Vec<u8>, &'static str> {
    if rows < 2
        || rows % 2 != 0
        || width < 32
        || width % 32 != 0
        || rows.checked_mul(width) != Some(if ENCODING { source.len() } else { rows * width })
        || maps.len() != 4 * 6 * 256
        || regression.len() != 304
        || regression.get(..4) != Some(b"RG2\0")
        || (!ENCODING && stream.len() < 4)
    {
        return Err("invalid sensor codec input");
    }
    let bpr = width / 16;
    let groups = bpr / 2;
    let mut coder = RangeCoder::<ENCODING>::for_sensor(source.len(), stream);
    // Predictors inspect the current row and at most four earlier rows. Keep
    // those rows in a ring instead of retaining a 16-bit copy of the full strip.
    let ring_len = width.checked_mul(5).ok_or("sensor geometry overflow")?;
    let mut image = vec![0u16; ring_len];
    let mut mid_above = vec![0i32; 4 * groups];
    let mut range_above = vec![0i32; 4 * groups];

    let mut values: [Vec<i32>; 4] = std::array::from_fn(|_| Vec::with_capacity(2048));
    let mut ranks = vec![[-1i32; 2048]; 4];
    let mut nearest = vec![[0usize; 2048]; 4];
    for phase in 0..4 {
        for v in 0..2048 {
            if map_present(maps, phase, 0, v) {
                ranks[phase][v] = values[phase].len() as i32;
                values[phase].push(v as i32);
            }
        }
        if values[phase].is_empty() {
            return Err("empty endpoint alphabet");
        }
        let mut k = 0usize;
        for (v, n) in nearest[phase].iter_mut().enumerate() {
            while k + 1 < values[phase].len()
                && values[phase][k + 1] - (v as i32) < v as i32 - values[phase][k]
            {
                k += 1;
            }
            *n = k;
        }
    }

    let mut headers: [HeaderLearner; 24] = std::array::from_fn(|_| HeaderLearner::default());
    let mut header_models = HeaderModels::default();
    let mut cdf = CdfModel::default();
    let mut positions = PositionModel::default();
    let mut escape_probability = Probability::initial();
    let mut tie_exception = [Probability::initial(); 20];
    let mut errors = [[0i32; 9]; 4];
    let mut block_errors = [0i32; 4];
    let mut reg_native = [[0i32; 3]; 4];
    let mut reg_fit = [[0i32; 3]; 4];
    #[cfg(feature = "profile-counters")]
    let mut counters = SensorProfileCounters::default();
    let mut output = if ENCODING {
        Vec::new()
    } else {
        vec![0; rows * width]
    };
    let empty_deltas = [0i32; 14];

    for y in 0..rows {
        let row0 = (y % 5) * width;
        let row1 = ((y + 4) % 5) * width;
        let row2 = ((y + 3) % 5) * width;
        let row3 = ((y + 2) % 5) * width;
        let row4 = ((y + 1) % 5) * width;
        let mut mid_left = [0i32; 2];
        let mut range_left = [0i32; 2];
        let has_a = y >= 2;
        let has_1 = y >= 1;
        for j in 0..bpr {
            #[cfg(feature = "profile-counters")]
            {
                counters.blocks += 1;
            }
            let par = j & 1;
            let g = j >> 1;
            let phase = (y & 1) * 2 + par;
            let byte_at = (y * bpr + j) * 16;
            let parsed = if ENCODING {
                Some(parse_sensor_block(&source[byte_at..byte_at + 16])?)
            } else {
                None
            };
            // The encoder reads block fields from the input. The decoder starts
            // with placeholders and recovers those same fields from the stream.
            let (mut mx, mut mn, mut ix, mut iy, mut deltas) =
                parsed.unwrap_or((0, 0, 0, 0, empty_deltas));
            let mut rmn = if ENCODING {
                ranks[phase][mn as usize]
            } else {
                0
            };
            let mut rmx = if ENCODING {
                ranks[phase][mx as usize]
            } else {
                0
            };
            if ENCODING && (rmn < 0 || rmx < 0) {
                return Err("endpoint absent from alphabet map");
            }
            let has_l = g > 0;
            let rg_l = range_left[par];
            let mid_l = mid_left[par];
            let rg_a = if has_a {
                range_above[phase * groups + g]
            } else {
                0
            };
            let mid_a = if has_a {
                mid_above[phase * groups + g]
            } else {
                0
            };
            let pr = if has_l && has_a {
                (rg_l + rg_a + 1) >> 1
            } else if has_l {
                rg_l
            } else if has_a {
                rg_a
            } else {
                64
            };
            let dl = if has_l && has_a {
                let d = (rg_l - rg_a).abs();
                if d * 4 <= pr {
                    0
                } else if d * 2 <= pr {
                    1
                } else {
                    2
                }
            } else {
                0
            };
            let context_r = bucket12(pr) * 3 + dl as usize;
            let interior = has_a && has_l;
            let up2_start = if has_a { row2 } else { 0 };
            let profile = if has_a {
                header_profile(
                    &image[up2_start..up2_start + width],
                    g * 32 + par,
                    &nearest[phase],
                )
            } else {
                HeaderProfile::default()
            };
            let left_profile = if has_a && has_l {
                header_profile(
                    &image[row0..row0 + width],
                    g * 32 + par - 32,
                    &nearest[phase],
                )
            } else {
                HeaderProfile::default()
            };
            let mut rf = [0i32; 6];
            let mut rp = pr;
            let mut rc = pr;
            if has_a {
                rf = [
                    rg_a - pr,
                    if has_l { rg_l - pr } else { 0 },
                    profile.trimmed - pr,
                    4 * profile.mad - pr,
                    3 * profile.adjacent - pr,
                    16,
                ];
                let learner = &headers[phase * 3 + dl];
                rc = learner.candidate(pr, &rf);
                if learner.corrected_error < learner.native_error {
                    rp = rc;
                }
            }

            let stump_mask = u8::from(phase == 1 || phase == 2)
                | (((y & 1) as u8) << 1)
                | (u8::from(rg_a > rg_l) << 2)
                | (u8::from(mid_a > mid_l) << 3)
                | (u8::from(4 * profile.mad > pr) << 4)
                | (u8::from(profile.mean > values[phase].len() as i32 / 2) << 5)
                | (u8::from((mid_a - mid_l).abs() > pr / 2) << 6)
                | (u8::from(profile.trimmed > pr) << 7);

            // Code the endpoint range and minimum as signed residuals from
            // causal neighboring-block estimates.
            let signed_range = header_models.code(
                &mut coder,
                0,
                context_r,
                interior,
                stump_mask,
                if ENCODING { (rmx - rmn) - rp } else { 0 },
            )? + rp;
            if !(-2047..=2047).contains(&signed_range) {
                return Err("decoded range outside sensor bounds");
            }
            if has_a {
                headers[phase * 3 + dl].update(pr, rc, signed_range, &rf);
            }
            let rg = signed_range.max(0);
            let pmid = if has_l && has_a {
                (mid_l + mid_a + 1) >> 1
            } else if has_l {
                mid_l
            } else if has_a {
                mid_a
            } else {
                1024
            };
            let dm = if has_l && has_a {
                let d = (mid_l - mid_a).abs();
                if d * 2 <= rg {
                    0
                } else if d <= 2 * rg {
                    1
                } else {
                    2
                }
            } else {
                0
            };
            let pm = (pmid - (rg >> 1)).clamp(0, 2047);
            let context_m = bucket12(rg) * 3 + dm as usize;
            let mut mf = [0i32; 6];
            let mut mc = pm;
            let mut mp = pm;
            if has_a {
                mf = [
                    profile.mean - (pm + (rg >> 1)),
                    profile.second - pm,
                    profile.minimum - pm,
                    if has_l {
                        left_profile.mean - (pm + (rg >> 1))
                    } else {
                        0
                    },
                    (rg_a - rg) / 2,
                    16,
                ];
                let learner = &headers[12 + phase * 3 + dm];
                mc = learner.candidate(pm, &mf);
                if learner.corrected_error < learner.native_error {
                    mp = mc;
                }
            }
            let residual_m = header_models.code(
                &mut coder,
                1,
                context_m,
                interior,
                (stump_mask & 127) | (u8::from(rg > pr) << 7),
                if ENCODING { rmn - mp } else { 0 },
            )?;
            rmn = mp + residual_m;
            rmx = rmn + signed_range;
            if rmn < 0
                || rmx < 0
                || rmn as usize >= values[phase].len()
                || rmx as usize >= values[phase].len()
            {
                return Err("endpoint rank outside alphabet");
            }
            if has_a {
                headers[12 + phase * 3 + dm].update(pm, mc, rmn, &mf);
            }
            mn = values[phase][rmn as usize];
            mx = values[phase][rmx as usize];
            let rank_range = rg;
            let rank_mid = rmn + (rg >> 1);
            let range = mx - mn;
            let rg = range.max(0);

            let base_col = g * 32 + par;
            let paired = par != 0;
            let score_scale = (if has_a { 2 } else { 0 })
                + (if has_1 { 1 } else { 0 })
                + (if paired { 2 } else { 0 });
            let score_arr = if base_col < width && width - base_col > 30 {
                let scores = std::array::from_fn(|i| {
                    (if has_a {
                        2 * image[up2_start + base_col + 2 * i] as i32
                    } else {
                        0
                    }) + (if has_1 {
                        image[row1 + base_col + 2 * i] as i32
                    } else {
                        0
                    }) + (if paired {
                        2 * image[row0 + base_col + 2 * i - 1] as i32
                    } else {
                        0
                    })
                });
                Some(scores)
            } else {
                None
            };
            #[cfg(feature = "profile-counters")]
            let position_started = (counters.position_model_calls % 64 == 0).then(Instant::now);
            // The two extremum positions identify which fourteen sample
            // differences remain to be coded in this block.
            let (decoded_ix, decoded_iy) = positions.code_pair(
                &mut coder,
                phase,
                rg,
                score_arr.as_ref(),
                score_scale,
                ix,
                iy,
            )?;
            #[cfg(feature = "profile-counters")]
            {
                counters.position_model_calls += 1;
                if let Some(started) = position_started {
                    counters.position_samples += 1;
                    counters.position_sample_ns += started.elapsed().as_nanos();
                }
            }
            ix = decoded_ix;
            iy = decoded_iy;

            // Sony scales each 7-bit interior difference by a range-dependent
            // power of two shared by all samples in the block.
            let mut shift = 0usize;
            while shift < 4 && (0x80 << shift) <= range {
                shift += 1;
            }
            let qhi = (rg >> shift).min(127);
            let midpoint = mn + (rg >> 1);
            let mut escape = false;
            if ENCODING {
                escape = deltas.iter().any(|&d| d > qhi);
            }
            escape = escape_probability.code(&mut coder, u8::from(escape))? != 0;
            let hiq = if escape { 127 } else { qhi };

            // qhi is capped at 127, so keep the compact alphabet on the stack.
            // This code runs once for every 16-pixel block; allocating a Vec here
            // caused a heap allocation per block on the encoder's hot path.
            // Use the observed per-phase/per-scale difference alphabet when
            // possible; an escape selects the complete 7-bit alphabet.
            let mut allowed = [0u8; 128];
            let mut allowed_rank = [u8::MAX; 128];
            let mut allowed_mask = [0u64; 2];
            let mut allowed_len = 0usize;
            let compact = !escape && range >= 0;
            #[cfg(feature = "profile-counters")]
            {
                if escape {
                    counters.escaped_blocks += 1;
                } else if compact {
                    counters.compact_blocks += 1;
                }
            }
            if compact {
                for q in 0..=qhi {
                    let value = mn + (q << shift);
                    if map_present(maps, phase, shift + 1, value as usize) {
                        allowed_rank[q as usize] = allowed_len as u8;
                        allowed[allowed_len] = q as u8;
                        allowed_len += 1;
                        allowed_mask[q as usize >> 6] |= 1u64 << (q as usize & 63);
                    }
                }
                #[cfg(feature = "profile-counters")]
                {
                    counters.compact_alphabet_entries += allowed_len as u64;
                }
                if allowed_len == 0 {
                    return Err("empty compact delta alphabet");
                }
            }
            let mut tie_active = false;
            let tie_low = compact && range > 0 && ix != iy && allowed_len >= 2 && allowed[0] == 0;
            let tie_high = compact
                && range > 0
                && ix != iy
                && allowed_len >= 2
                && mn + ((allowed[allowed_len - 1] as i32) << shift) == mx;
            if compact && range > 0 && ix != iy && allowed_len >= 2 {
                let mut any = false;
                let mut empty = false;
                let mut exception = false;
                let mut slot = 0;
                for i in 0..16 {
                    if i == ix || i == iy {
                        continue;
                    }
                    let drop_lo = tie_low && i < iy;
                    let drop_hi = tie_high && i > ix;
                    any |= drop_lo || drop_hi;
                    empty |= drop_lo && drop_hi && allowed_len == 2;
                    if ENCODING
                        && ((drop_lo && deltas[slot] == allowed[0] as i32)
                            || (drop_hi && deltas[slot] == allowed[allowed_len - 1] as i32))
                    {
                        exception = true;
                    }
                    slot += 1;
                }
                if any && !empty {
                    exception = tie_exception[phase * 5 + shift]
                        .code(&mut coder, u8::from(exception))?
                        != 0;
                    tie_active = !exception;
                }
            }

            let mut k = 0usize;
            let mut profile = [0i32; 16];
            let mut old_profile = [0i32; 16];
            let mut profile_mean = 0i32;
            if y >= 4 {
                let now_start = if par != 0 { row0 } else { row1 };
                let old_start = if par != 0 { row2 } else { row3 };
                for t in 0..16 {
                    let col = base_col + 2 * t;
                    if par != 0 {
                        let right = if t == 15 { col - 1 } else { col + 1 };
                        profile[t] = (image[now_start + col - 1] as i32
                            + image[now_start + right] as i32
                            + 1)
                            / 2;
                        old_profile[t] = (image[old_start + col - 1] as i32
                            + image[old_start + right] as i32
                            + 1)
                            / 2;
                    } else {
                        profile[t] = image[now_start + col] as i32;
                        old_profile[t] = image[old_start + col] as i32;
                    }
                    profile_mean += profile[t];
                }
                profile_mean = (profile_mean + 8) / 16;
            }

            for i in 0..16 {
                let col = base_col + 2 * i;
                let p = if i == ix {
                    mx
                } else if i == iy {
                    mn
                } else if k >= 14 {
                    mn
                } else {
                    #[cfg(feature = "profile-counters")]
                    {
                        counters.predicted_pixels += 1;
                    }
                    #[cfg(feature = "profile-counters")]
                    let predictor_started =
                        (counters.predicted_pixels % 256 == 1).then(Instant::now);
                    let l = if col >= 2 {
                        image[row0 + col - 2] as i32
                    } else if has_a {
                        image[up2_start + col] as i32
                    } else {
                        midpoint
                    };
                    let a = if has_a {
                        image[up2_start + col] as i32
                    } else {
                        l
                    };
                    let al = if has_a && col >= 2 {
                        image[up2_start + col - 2] as i32
                    } else {
                        (l + a) >> 1
                    };
                    let ar = if has_a && col + 2 < width {
                        image[up2_start + col + 2] as i32
                    } else {
                        a
                    };
                    let d1 = if has_1 && col >= 1 {
                        image[row1 + col - 1] as i32
                    } else {
                        -1
                    };
                    let d2 = if has_1 && col + 1 < width {
                        image[row1 + col + 1] as i32
                    } else {
                        -1
                    };
                    let diagonal = d1 >= 0 && d2 >= 0;
                    let median = {
                        let low = l.min(a);
                        let high = l.max(a);
                        if al >= high {
                            low
                        } else if al <= low {
                            high
                        } else {
                            l + a - al
                        }
                    };
                    // Candidate estimates use causal same-row, above-row,
                    // diagonal, gradient and median-neighbor information.
                    let cd = [
                        (l + a + 1) >> 1,
                        midpoint,
                        (l + a - al).clamp(0, 2047),
                        (((l + a + 1) >> 1) + midpoint + 1) >> 1,
                        (l + a + ar + midpoint + 2) >> 2,
                        if diagonal {
                            (d1 + d2 + 1) >> 1
                        } else {
                            (l + a + 1) >> 1
                        },
                        median,
                        if diagonal {
                            (d1 + d2 + l + a + 2) >> 2
                        } else {
                            (l + a + 1) >> 1
                        },
                        if has_1 && col >= 2 {
                            (l + image[row1 + col] as i32 - image[row1 + col - 2] as i32)
                                .clamp(0, 2047)
                        } else {
                            (l + a + 1) >> 1
                        },
                    ];
                    let e = &mut errors[phase];
                    let mut sw = 0u64;
                    let mut sp = 0u64;
                    for u in 0..9 {
                        let q = (e[u] + 8) as u32;
                        let w = 0x8000_0000u32 / (q * q);
                        sw += w as u64;
                        sp += w as u64 * cd[u] as u64;
                    }
                    debug_assert!(sw > 0 && sw <= u32::MAX as u64);
                    let mut pred = div_u64_by_u32(sp + (sw >> 1), sw as u32) as i32;
                    pred = pred.clamp(mn, mn + rg);
                    let native_pred = pred;
                    let mut fit_pred = 0i32;
                    let mut eligible = false;
                    let mut regbin = 0usize;
                    if y >= 4 && col >= 4 && col + 2 < width {
                        let low = l.min(a).min(al);
                        let high = l.max(a).max(al);
                        let activity = high - low;
                        regbin = if activity <= 16 {
                            0
                        } else if activity <= 63 {
                            1
                        } else {
                            2
                        };
                        let rec = &regression[4 + (phase * 3 + regbin) * 25..];
                        eligible = rec[0] != 0;
                        if eligible {
                            #[cfg(feature = "profile-counters")]
                            {
                                counters.regression_evaluations += 1;
                            }
                            let di = (i as i32 - iy as i32).abs();
                            let dx = (i as i32 - ix as i32).abs();
                            let denominator = di + dx;
                            let pos = if denominator != 0 {
                                range * (di - dx) / denominator
                            } else {
                                0
                            };
                            let features = [
                                l - midpoint,
                                a - midpoint,
                                al - midpoint,
                                ar - midpoint,
                                image[row0 + col - 4] as i32 - midpoint,
                                image[row4 + col] as i32 - midpoint,
                                image[row1 + col] as i32 - image[row3 + col] as i32,
                                pos,
                                profile[i] - profile_mean,
                                if i > 0 {
                                    profile[i] - profile[i - 1]
                                } else {
                                    0
                                },
                                profile[i] - old_profile[i],
                            ];
                            let mut sum = 0i64;
                            for u in 0..11 {
                                let coef =
                                    i16::from_le_bytes([rec[1 + 2 * u], rec[2 + 2 * u]]) as i64;
                                sum += coef * features[u] as i64;
                            }
                            let bias = i16::from_le_bytes([rec[23], rec[24]]) as i32;
                            fit_pred = (midpoint + bias + round_div_i64(sum, 512) as i32)
                                .clamp(mn, mn + rg);
                            if reg_fit[phase][regbin] < reg_native[phase][regbin] {
                                pred = fit_pred;
                            }
                        }
                    }
                    let mut pred_q4 = ((pred - mn) << 2) >> shift;
                    let mut error_q8 = block_errors[phase] >> shift;
                    let mut hq = hiq;
                    let mut input = if ENCODING { deltas[k] } else { 0 };
                    if compact {
                        hq = allowed_len as i32 - 1;
                        if pred_q4 <= 4 * allowed[0] as i32 {
                            pred_q4 = 0;
                        } else if pred_q4 >= 4 * allowed[allowed_len - 1] as i32 {
                            pred_q4 = 4 * (allowed_len as i32 - 1);
                        } else {
                            let q = (pred_q4 >> 2) as usize;
                            let limit = q + usize::from(pred_q4 & 3 != 0);
                            let t = allowed_count_before(allowed_mask, limit) - 1;
                            #[cfg(feature = "profile-counters")]
                            {
                                counters.compact_mapping_lookups += 1;
                            }
                            pred_q4 = 4 * t as i32
                                + (pred_q4 - 4 * allowed[t] as i32)
                                    / (allowed[t + 1] as i32 - allowed[t] as i32);
                        }
                        if ENCODING {
                            let delta = deltas[k] as usize;
                            let rank = allowed_rank
                                .get(delta)
                                .copied()
                                .filter(|&rank| rank != u8::MAX)
                                .ok_or("delta missing from compact alphabet")?;
                            input = rank as i32;
                        }
                        error_q8 = error_q8 * (allowed_len as i32 - 1) / qhi.max(1);
                    }
                    let low = l.min(a).min(al).min(ar);
                    let high = l.max(a).max(al).max(ar);
                    let spread = high - low;
                    let texture = if spread * 4 <= rg {
                        0
                    } else if spread * 2 <= rg {
                        1
                    } else if spread <= rg {
                        2
                    } else {
                        3
                    };
                    let mut gradient = ((l - a).abs() + (a - al).abs() + (l - al).abs()) >> shift;
                    if compact {
                        gradient = gradient * (allowed_len as i32 - 1) / qhi.max(1);
                    }
                    #[cfg(feature = "profile-counters")]
                    if let Some(started) = predictor_started {
                        counters.predictor_samples += 1;
                        counters.predictor_sample_ns += started.elapsed().as_nanos();
                    }
                    let drop_low = tie_active && tie_low && i < iy;
                    let drop_high = tie_active && tie_high && i > ix;
                    #[cfg(feature = "profile-counters")]
                    let cdf_started = (cdf.profile_calls % 256 == 0).then(Instant::now);
                    // Code the scaled difference's rank around the predictor.
                    // Encoder and decoder update the same CDF and error totals.
                    let qv = cdf.code_bounded(
                        &mut coder,
                        hq,
                        pred_q4,
                        error_q8,
                        input,
                        shift,
                        phase,
                        gradient,
                        (midpoint >> 8) as usize,
                        texture,
                        drop_low,
                        drop_high,
                    )?;
                    #[cfg(feature = "profile-counters")]
                    if let Some(started) = cdf_started {
                        counters.cdf_samples += 1;
                        counters.cdf_sample_ns += started.elapsed().as_nanos();
                    }
                    let qv = if compact {
                        allowed[qv as usize] as i32
                    } else {
                        qv
                    };
                    deltas[k] = qv;
                    k += 1;
                    let pixel = (mn + (qv << shift)).min(2047);
                    for u in 0..9 {
                        e[u] = e[u] - (e[u] >> 3) + (pixel - cd[u]).abs();
                    }
                    block_errors[phase] =
                        block_errors[phase] - (block_errors[phase] >> 3) + (pixel - pred).abs();
                    if eligible {
                        reg_native[phase][regbin] = reg_native[phase][regbin]
                            - (reg_native[phase][regbin] >> 4)
                            + (pixel - native_pred).abs();
                        reg_fit[phase][regbin] = reg_fit[phase][regbin]
                            - (reg_fit[phase][regbin] >> 4)
                            + (pixel - fit_pred).abs();
                    }
                    pixel
                };
                image[row0 + col] = p as u16;
            }
            if !ENCODING {
                store_sensor_block(&mut output[byte_at..byte_at + 16], mx, mn, ix, iy, &deltas);
            }
            range_left[par] = rank_range;
            mid_left[par] = rank_mid;
            range_above[phase * groups + g] = rank_range;
            mid_above[phase * groups + g] = rank_mid;
        }
    }
    if ENCODING {
        #[cfg(feature = "profile-counters")]
        eprintln!(
            "sensor profile: blocks={}, predicted_pixels={}, regression_evaluations={}, compact_blocks={}, escaped_blocks={}, compact_alphabet_entries={}, compact_mapping_lookups={}, position_calls={}, cdf_calls={}, cdf_search_steps={}, cdf_rebuilds={}, cdf_decay_rebuilds={}, cdf_forced_bits={}, range_bits={}, range_renormalizations={}, sampled_predictor_ns={} ({} samples), sampled_cdf_ns={} ({} samples), sampled_position_ns={} ({} samples)",
            counters.blocks,
            counters.predicted_pixels,
            counters.regression_evaluations,
            counters.compact_blocks,
            counters.escaped_blocks,
            counters.compact_alphabet_entries,
            counters.compact_mapping_lookups,
            counters.position_model_calls,
            cdf.profile_calls,
            cdf.profile_search_steps,
            cdf.profile_rebuilds,
            cdf.profile_decay_rebuilds,
            cdf.profile_forced_bits,
            coder.profile_bit_calls,
            coder.profile_renormalizations,
            counters.predictor_sample_ns,
            counters.predictor_samples,
            counters.cdf_sample_ns,
            counters.cdf_samples,
            counters.position_sample_ns,
            counters.position_samples,
        );
        Ok(coder.finish())
    } else if coder.decoder_at_end() {
        Ok(output)
    } else {
        Err("sensor stream has trailing or inconsistent range-coder state")
    }
}

fn map_present(maps: &[u8], phase: usize, kind: usize, value: usize) -> bool {
    value < 2048 && (maps[(phase * 6 + kind) * 256 + (value >> 3)] & (1 << (value & 7))) != 0
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum Level {
    Quiet,
    Info,
    Debug,
}

impl Level {
    pub fn parse(value: &str) -> Option<Self> {
        match value {
            "quiet" => Some(Self::Quiet),
            "info" => Some(Self::Info),
            "debug" => Some(Self::Debug),
            _ => None,
        }
    }
}

/// Serialize a single log event into the provided stream.
pub fn log_to(
    writer: &mut impl Write,
    configured: Level,
    level: Level,
    message: &str,
) -> io::Result<()> {
    if level != Level::Quiet && configured >= level {
        writeln!(writer, "{message}")?;
        writer.flush()?;
    }
    Ok(())
}

/// An explicitly finished phase timer, using monotonic time.
pub struct PhaseTimer(Instant);

impl Default for PhaseTimer {
    fn default() -> Self {
        Self::start()
    }
}

impl PhaseTimer {
    pub fn start() -> Self {
        Self(Instant::now())
    }

    pub fn elapsed(&self) -> Duration {
        self.0.elapsed()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn levels_filter_messages() {
        let mut out = Vec::new();
        log_to(&mut out, Level::Info, Level::Debug, "hidden").unwrap();
        log_to(&mut out, Level::Info, Level::Info, "visible").unwrap();
        assert_eq!(out, b"visible\n");
        assert_eq!(Level::parse("verbose"), None);
    }

    #[test]
    fn quiet_suppresses_info() {
        let mut out = Vec::new();
        log_to(&mut out, Level::Quiet, Level::Info, "hidden").unwrap();
        assert!(out.is_empty());
    }

    #[test]
    fn div_u64_by_u32_matches_integer_division() {
        let mut state = 0x9e37_79b9u32;
        for _ in 0..50_000 {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let denominator = state.max(1);
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let numerator_q16 = state as u64 % (denominator as u64 + 1);
            let numerator = numerator_q16 * 65536;
            assert_eq!(
                div_u64_by_u32(numerator, denominator),
                (numerator / denominator as u64) as u32,
                "numerator={numerator}, denominator={denominator}"
            );
        }
        for denominator in [1, 2, 3, 65535, 65536, u32::MAX] {
            let numerator = denominator as u64 * 65536;
            assert_eq!(
                div_u64_by_u32(numerator, denominator),
                (numerator / denominator as u64) as u32
            );
        }
    }

    #[test]
    fn compact_prediction_lookup_matches_linear_mapping() {
        for seed in 0u32..64 {
            let allowed: Vec<u8> = (0u8..=127)
                .filter(|q| *q == 0 || *q == 127 || (*q as u32 * 17 + seed * 13) % 11 < 6)
                .collect();
            let mut mask = [0u64; 2];
            for &q in &allowed {
                mask[q as usize >> 6] |= 1u64 << (q as usize & 63);
            }
            for prediction in 0..=4 * allowed[allowed.len() - 1] as i32 {
                let expected = if prediction <= 4 * allowed[0] as i32 {
                    0
                } else if prediction >= 4 * allowed[allowed.len() - 1] as i32 {
                    4 * (allowed.len() as i32 - 1)
                } else {
                    let mut t = 0;
                    while t + 1 < allowed.len() && prediction > 4 * allowed[t + 1] as i32 {
                        t += 1;
                    }
                    4 * t as i32
                        + (prediction - 4 * allowed[t] as i32)
                            / (allowed[t + 1] as i32 - allowed[t] as i32)
                };
                let actual = if prediction <= 4 * allowed[0] as i32 {
                    0
                } else if prediction >= 4 * allowed[allowed.len() - 1] as i32 {
                    4 * (allowed.len() as i32 - 1)
                } else {
                    let q = (prediction >> 2) as usize;
                    let limit = q + usize::from(prediction & 3 != 0);
                    let t = allowed_count_before(mask, limit) - 1;
                    4 * t as i32
                        + (prediction - 4 * allowed[t] as i32)
                            / (allowed[t + 1] as i32 - allowed[t] as i32)
                };
                assert_eq!(actual, expected, "seed={seed}, prediction={prediction}");
            }
        }
    }

    #[test]
    fn phase_clock_is_monotonic() {
        let timer = PhaseTimer::start();
        let first = timer.elapsed();
        assert!(timer.elapsed() >= first);
    }

    #[test]
    fn native_maps_reject_invalid_geometry() {
        assert!(sensor_maps(&[], 0, 0).is_err());
        assert!(sensor_maps(&[0; 63], 2, 32).is_err());
    }

    #[test]
    fn unpack_sony_block_and_even_odd_layout() {
        let lo = 1000u64 | (100u64 << 11) | (1u64 << 26);
        let mut block = [0u8; 16];
        block[..8].copy_from_slice(&lo.to_le_bytes());
        let pixels = unpack_block(&block).unwrap();
        assert_eq!(pixels[0], 1000);
        assert_eq!(pixels[1], 100);
        assert!(pixels[2..].iter().all(|v| *v == 100));

        let strip = [block, block, block, block].concat();
        let image = unpack_sensor_strip(&strip, 2, 32).unwrap();
        assert_eq!(image[0], 1000);
        assert_eq!(image[1], 1000);
        assert_eq!(image[2], 100);
        assert_eq!(image[16], 100);
        assert_eq!(image[17], 100);
    }

    #[test]
    fn cross_profile_fit_emits_valid_rg2_table() {
        let blocks: Vec<u8> = (0usize..64 * 64)
            .map(|i| (i.wrapping_mul(73).wrapping_add(i / 17)) as u8)
            .collect();
        let table = fit_cross_profile(&blocks, 64, 64).unwrap();
        assert_eq!(table.len(), 304);
        assert_eq!(&table[..4], b"RG2\0");
        assert!(table[4..].chunks_exact(25).all(|r| r[0] <= 1));
    }
}
