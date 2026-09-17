#!/usr/bin/env bash
# Record a screen capture of the full mission.
#
# The mission is NOT started by this script: the point of the demo video is the
# GUI run, and the controllers must be up before recording starts, otherwise the
# first seconds show a robot that is not yet under control.
#
#   Terminal 1:  bash scripts/clean_ros.sh
#                ./scripts/run_native.sh ros2 launch pas_dual_arm_bringup mission.launch.py
#                ... wait until all eight controllers are active and RViz has drawn the map
#   Terminal 2:  ./scripts/record_demo.sh            # then press the mission button
#
# Stop with q in this terminal, or Ctrl-C.
#
#   --out FILE     output file            (default: dist/demo-<timestamp>.mp4)
#   --fps N        frames per second      (default: 30)
#   --region WxH+X+Y   capture a region instead of the whole screen
#   --window       pick a window with the mouse and capture only that
set -euo pipefail

cd "$(dirname "$0")/.."
FPS=30
OUT="dist/demo-$(date +%Y%m%d_%H%M%S).mp4"
REGION=""
PICK=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out)    OUT="$2"; shift 2 ;;
    --fps)    FPS="$2"; shift 2 ;;
    --region) REGION="$2"; shift 2 ;;
    --window) PICK=1; shift ;;
    -h|--help) sed -n '2,18p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

command -v ffmpeg >/dev/null || { echo "ffmpeg is not installed: sudo apt install ffmpeg" >&2; exit 1; }
[[ "${XDG_SESSION_TYPE:-}" == "x11" ]] || echo "warning: session is '${XDG_SESSION_TYPE:-unknown}', x11grab expects X11" >&2

if [[ "$PICK" == "1" ]]; then
  command -v xwininfo >/dev/null || { echo "--window needs x11-utils: sudo apt install x11-utils" >&2; exit 1; }
  echo "Click the window you want to record..."
  info=$(xwininfo)
  X=$(awk '/Absolute upper-left X/ {print $NF}' <<<"$info")
  Y=$(awk '/Absolute upper-left Y/ {print $NF}' <<<"$info")
  W=$(awk '/Width:/ {print $NF}' <<<"$info")
  H=$(awk '/Height:/ {print $NF}' <<<"$info")
  REGION="${W}x${H}+${X}+${Y}"
fi

if [[ -n "$REGION" ]]; then
  SIZE="${REGION%%+*}"
  OFFS="+${REGION#*+}"
else
  SIZE=$(xdpyinfo | awk '/dimensions:/ {print $2}')
  OFFS="+0,0"
fi
# ffmpeg wants +X,Y on the input URL; accept the familiar +X+Y spelling too.
OFFS="${OFFS//+/,}"; OFFS="+${OFFS#,}"

# x264 needs even dimensions.
SIZE=$(awk -F x '{printf "%dx%d", int($1/2)*2, int($2/2)*2}' <<<"$SIZE")

mkdir -p "$(dirname "$OUT")"
echo "Recording ${SIZE} at ${OFFS}, ${FPS} fps  ->  ${OUT}"
echo "Press q to stop."

ffmpeg -hide_banner -loglevel warning -stats \
       -f x11grab -framerate "$FPS" -video_size "$SIZE" -i "${DISPLAY}${OFFS}" \
       -c:v libx264 -preset veryfast -crf 23 -pix_fmt yuv420p \
       -movflags +faststart \
       "$OUT"

echo
echo "Saved: $OUT"
echo "Size:  $(du -h "$OUT" | cut -f1)"
