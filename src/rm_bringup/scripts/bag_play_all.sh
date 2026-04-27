#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  bag_play_all.sh <BAG_DIR> [--rate R] [--loop]

Plays back a rosbag2 folder (db3/sqlite3) with /clock enabled so RViz and
other tools can replay time consistently.

Examples:
  ./bag_play_all.sh ~/rosbags/rm_bringup/20260407_213000
  ./bag_play_all.sh ~/rosbags/rm_bringup/20260407_213000 --rate 0.5
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${#}" -lt 1 ]]; then
  usage
  exit 0
fi

BAG_DIR="$1"
shift

exec ros2 bag play "$BAG_DIR" --clock "$@"

