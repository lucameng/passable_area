#!/bin/bash

JY_COG_PATH="/home/deep/deeprobotics"
workspace_name="passable_noetic_ws"
home_path="${JY_COG_PATH}"
ws_path="${home_path}/${workspace_name}"


bash "${ws_path}/scripts/kill_node.sh"
sleep 1

if [ -f "${ws_path}/devel/setup.bash" ]; then
    source "${ws_path}/devel/setup.bash"
else
    echo "❌ Cannot source ${ws_path}/devel/setup.bash"
    exit 1
fi

roslaunch passable_node passable_mapping.launch
