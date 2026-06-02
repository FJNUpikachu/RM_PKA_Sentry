#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bag_record_all.sh [OUTPUT_ROOT] [SPLIT_DURATION_S]

Records ALL ROS 2 topics into a rosbag2 (db3/sqlite3) folder:
  <OUTPUT_ROOT>/<YYYYMMDD_HHMMSS>/

Defaults:
  OUTPUT_ROOT        = ~/rosbags/rm_bringup
  SPLIT_DURATION_S   = 300   (0 disables splitting)

Examples:
  ./bag_record_all.sh
  ./bag_record_all.sh ~/rosbags/my_run 600
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

ROOT="${1:-$HOME/rosbags/rm_bringup}"
SPLIT="${2:-300}"

TS="$(date +%Y%m%d_%H%M%S)"
OUT="$ROOT/$TS"
mkdir -p "$OUT"

ARGS=(ros2 bag record -a --storage sqlite3 -o "$OUT")
if [[ "$SPLIT" != "0" ]]; then
  ARGS+=("--max-bag-duration" "$SPLIT")
fi

echo "[bag_record_all] recording to: $OUT"
exec "${ARGS[@]}"

