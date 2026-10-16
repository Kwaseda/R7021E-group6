#!/usr/bin/env bash
# replay_record.sh: play a hall bag through the filter and record what the filter publishes.
#
#   lab4/video/replay_record.sh <hall bag dir> <run log .json of the live run> <output bag dir>
#
# The filter gets the same values as the live run (read from its .json), with two changes for a
# replay: use_sim_time true (the bag's /clock) and a deep scan queue, so no scan is dropped.
# The output bag holds /map, /particles, /slam_path, /scan and /tf for bag_to_video.py.
# A replay is not the live run: the particles draw new random numbers. It shows the same filter
# on the same sensor data.

set -euo pipefail
set -m

BAG=$(realpath "$1")
LIVE_JSON=$(realpath "$2")
OUT=$(realpath -m "$3")

PKG=~/R7021E-group6/lab4/ros2_ws/src/r7021e_fast_slam
TMP=$(mktemp -d)

export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-34}
set +u
source /opt/ros/jazzy/setup.bash
source ~/R7021E-group6/lab4/ros2_ws/install/setup.bash
set -u


# Params file: the live run's values, plus the two replay changes.
python3 - "$LIVE_JSON" "$TMP/params.yaml" <<'EOF'
import json
import sys

import yaml


live = json.load(open(sys.argv[1]))
params = {k: v for k, v in live.items() if v is not None}
params['scan_queue_depth'] = 1000
params['run_name'] = 'replay_' + str(live.get('run_name', 'run'))
params['log_dir'] = sys.argv[2].rsplit('/', 1)[0]
yaml.safe_dump({'grid_slam_node': {'ros__parameters': params}}, open(sys.argv[2], 'w'))
EOF


# Filter, recorder, then the bag with its clock.
cd $PKG
ros2 launch r7021e_fast_slam r7021e_fast_slam.launch.py \
    params_file:=$TMP/params.yaml use_rviz:=false use_sim_time:=true &
filter_pid=$!

ros2 bag record /map /particles /slam_path /scan /tf /tf_static -o "$OUT" &
record_pid=$!

sleep 5
ros2 bag play "$BAG" --clock

sleep 5
kill -INT $record_pid
wait $record_pid 2>/dev/null || true
kill -INT $filter_pid
wait $filter_pid 2>/dev/null || true

echo "done: $OUT"
