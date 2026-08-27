#!/bin/bash
# Record the dashboard camera feed on the robot, as insurance behind the
# operator's screen capture.
#
# Stream-copy, no re-encode: the dashboard already serves MJPEG, so this costs
# almost no CPU on the Orin and cannot drop frames because of encoder load
# during a filmed run. The result is large but it is a 5 minute clip.
#
#   record_feed.sh [seconds] [output.mkv]
set -e
SECS="${1:-360}"
OUT="${2:-$HOME/limo_project/data/demo_feed_$(date +%Y%m%d_%H%M%S).mkv}"

echo "recording ${SECS}s of /stream.mjpg -> ${OUT}"
ffmpeg -nostdin -loglevel warning \
       -f mjpeg -i "http://127.0.0.1:8080/stream.mjpg" \
       -t "${SECS}" -c copy "${OUT}"
echo "done: $(ls -la "${OUT}")"
