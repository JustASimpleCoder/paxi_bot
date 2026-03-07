#!/usr/bin/env bash
#
# Copyright 2026 JustASimpleCoder
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# This script opens starts a new tmux session and window in the background.
# It opens x (number of launch files) panes, running ros commands to start paxi's launch files.
# this script assumes you have added 'source /opt/ros/humble/setup.bash' to bashrc
# this script also assumes you have already compiled this repository

#Quick path check
if [[ "$PWD" != */paxi_ws ]]; then 
    echo "You must run this from the paxi_ws directory"
    echo "Run the script by typing ../scripts/tmux_launch.sh"
    echo "The current directory is [$PWD] "
    echo "hint: try running cd ~/paxi_ws"
    exit 1
fi

if [[ "$#" -ge 3 ]]; then 
    echo "Too many arguements given, only using first two..."
    echo "First arguement [$1] and second arguement [$2]"
fi

SESSION="tmux_launch"
WINDOW="tmux_launch"

if [[ "$#" -le 0 ]]; then
    echo "No arguements given to launch, setting window and session name to default"
fi

if [[ "$#" -eq 1 ]]; then
    SESSION="$1"
    WINDOW="$1"
fi

if [[ "$#" -eq 2 ]]; then
    SESSION="$1"
    WINDOW="$2"
fi

echo "Session name is [$SESSION] and Window name is [$WINDOW]"

ROS_COMMANDS=(
    "source install/setup.bash"
    "ros2 launch paxi_bringup"
)
cd ..
cd scripts

LAUNCH_FILES=()

mapfile -t LAUNCH_FILES < files/tmux_launch_files
LAUNCH_FILE_NUM = ${#LAUNCH_FILES[@]}


if [[ ! -f "files/tmux_launch_files" ]]; then
    echo "Error: files/tmux_launch_files not found!"
    exit 1
fi

if [[ ${#LAUNCH_FILES[@]} -eq 0 ]]; then
    echo "Error: No launch files found in files/tmux_launch_files"
    exit 1
fi

LAUNCH_FILE_NUM=${#LAUNCH_FILES[@]}
echo "Found $LAUNCH_FILE_NUM launch files to run:"
for file in "${LAUNCH_FILES[@]}"; do
    echo "  - $file"
done

cd ..
cd paxi_ws

#kill any old previous session that may be running
tmux kill-session -t $SESSION 2>/dev/null

#start main launch with the new session/window
tmux new -d -s "$SESSION" -n "$WINDOW"

#split screen enough times to create one pane for each script we are launching
for (( i=1; i < LAUNCH_FILE_NUM; i++ )); do
    tmux split-window -t "$SESSION:$WINDOW"
done

#make it look pretty with boxes
tmux select-layout -t "$SESSION":"$WINDOW" tiled

#source install and launch each file in all panes
for pane in $(tmux list-panes -F '#P'); do
    if [[ $pane -lt ${#LAUNCH_FILES[@]} ]]; then
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[0]}" C-m
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[1]} ${LAUNCH_FILES[$pane]}" C-m
    fi
done

echo "Sucessfully launched paxi_bot launch files for live mapping"
echo " - To open the tmux session please run: tmux attach -t $SESSION"
echo " - To close the tmux later you can run: tmux kill-session -t $SESSION"
