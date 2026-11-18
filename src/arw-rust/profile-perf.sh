#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INPUT_DIR="${ARW_PERF_INPUT_DIR:-${SCRIPT_DIR}/../../experiments/performance-data/arw-rust/perf-input}"
STAMP="$(date +%Y%m%d-%H%M%S)"
OUT_DIR="${ARW_PERF_OUT_DIR:-${SCRIPT_DIR}/../../experiments/performance-data/arw-rust/perf-results-${STAMP}}"
TARGET_DIR="${SCRIPT_DIR}/target/perf"
BIN="${TARGET_DIR}/release/arw-archive"

if ! command -v perf >/dev/null 2>&1; then
    echo "error: 'perf' is not installed or not on PATH" >&2
    exit 1
fi
if [[ ! -f "${INPUT_DIR}/DSC04281.ARW" ]]; then
    echo "error: expected benchmark input at ${INPUT_DIR}/DSC04281.ARW" >&2
    echo "Set ARW_PERF_INPUT_DIR to the directory containing DSC04281.ARW." >&2
    exit 1
fi

mkdir -p "${OUT_DIR}"
echo "Input:  ${INPUT_DIR}/DSC04281.ARW"
echo "Output: ${OUT_DIR}"

# Keep the profiling binary separate from the normal release build. Debug info
# enables useful symbols and DWARF call stacks without enabling debug checks.
CARGO_TARGET_DIR="${TARGET_DIR}" \
CARGO_PROFILE_RELEASE_DEBUG=1 \
cargo build --locked --release --manifest-path "${SCRIPT_DIR}/Cargo.toml"

mkdir -p "${OUT_DIR}/warmup"
"${BIN}" compress --log-level quiet "${INPUT_DIR}" "${OUT_DIR}/warmup"

# Separate output directories are required because compress refuses to
# overwrite an existing archive.
for run in 1 2 3 4 5; do
    mkdir -p "${OUT_DIR}/stat-${run}"
    perf stat -x, -o "${OUT_DIR}/stat-${run}.csv" \
        -e cycles,instructions,branches,branch-misses,cache-references,cache-misses \
        -- "${BIN}" compress --log-level quiet \
        "${INPUT_DIR}" "${OUT_DIR}/stat-${run}"
done

mkdir -p "${OUT_DIR}/record"
perf record -o "${OUT_DIR}/perf.data" -F 999 \
    --call-graph dwarf,16384 \
    -- "${BIN}" compress --log-level quiet \
    "${INPUT_DIR}" "${OUT_DIR}/record"

perf report -i "${OUT_DIR}/perf.data" --stdio \
    --children --call-graph graph,0.5,caller \
    --percent-limit 0.5 \
    > "${OUT_DIR}/report.txt"
perf script -i "${OUT_DIR}/perf.data" > "${OUT_DIR}/samples.txt"

echo
echo "Profiling complete. Share these files:"
echo "  ${OUT_DIR}/report.txt"
echo "  ${OUT_DIR}/samples.txt"
echo "  ${OUT_DIR}/stat-*.csv"
echo "  ${OUT_DIR}/perf.data"
