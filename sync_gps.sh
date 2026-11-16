#!/bin/bash
# Downloads today's GPS broadcast ephemeris (RINEX nav) from NASA CDDIS.
#
# Requires a free NASA Earthdata (URS) login: https://urs.earthdata.nasa.gov/
# Put your credentials in ~/.netrc (NOT in this script), then `chmod 600 ~/.netrc`:
#   machine urs.earthdata.nasa.gov login YOUR_USERNAME password YOUR_PASSWORD
# wget reads them automatically via --netrc below.
BASE_URL="https://cddis.nasa.gov/archive/gnss/data/daily"

# Get current date info
YEAR=$(date -u +%Y)
DOY=$(date -u +%j)
FILE="brdc${DOY}0.${YEAR:2:2}n"

echo "[SYNC] Downloading $FILE for DOY $DOY..."

# Download and decompress
# -u: only download if remote is newer
wget -q --netrc --keep-session-cookies --save-cookies ~/.urs_cookies \
     --no-check-certificate --content-disposition \
     "$BASE_URL/$YEAR/$DOY/${YEAR:2:2}n/$FILE.gz" -O "$FILE.gz"

if [ -f "$FILE.gz" ]; then
    gunzip -f "$FILE.gz"
    echo "[SYNC] Successfully updated $FILE"
else
    echo "[ERROR] Download failed. Check network or credentials."
fi
