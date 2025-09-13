#!/bin/bash

JY_COG_PATH="/home/deep/deeprobotics"
workspace_name="passable_humble_ws"
home_path="${JY_COG_PATH}"
ws_path="${home_path}/${workspace_name}"


bash "${ws_path}/scripts/kill_passable_node.sh"
sleep 1

if [ -f "${ws_path}/install/setup.bash" ]; then
    source "${ws_path}/install/setup.bash"
else
    echo "❌ Cannot source ${ws_path}/install/setup.bash"
    exit 1
fi

ros2 launch passable_node passable_launch.py