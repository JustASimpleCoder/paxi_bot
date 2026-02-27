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
#!/usr/bin/env bash

# This script opens starts a new tmux session and window in the background.
# It opens 4 panes, running ros commands to start paxi's launch files.
# this script assumes you have added 'source /opt/ros/humble/setup.bash' to bashrc
# this script also assumes you have already compiled this repository

# #Quick path check
# if [[ "$PWD" != */paxi_bot/scripts ]]; then 
#     echo "You must run this from the paxi_bot/scripts directory"
#     echo "The current direction is [$PWD] "
#     echo "hint: try running cd ~/paxi_bot/scripts"
#     exit 1
# fi


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/../paxi_ws"

SESSION="live_mapping"
WINDOW="live_mapping"



ROS_COMMANDS=(
    "source install/setup.bash"
    "ros2 launch paxi_bringup"
)

#place the scripts you want to run in a new terminal in tmux below:
LAUNCH_FILES=(
    "main_bringup.py ekf_filename:='nav2_ekf_live_mapping.yaml'"
    "live_async_mapping.py"
    #"live_display.py"
    #"manual_control.py"
)
LAUNCH_FILE_NUM=${#LAUNCH_FILES[@]}

#check if session already exist, kill it if it does
if tmux has-session -t "$SESSION"  2>/dev/null; then
    echo "[WARNING] tmux session [$SESSION] already exists, killing old session"
    tmux kill-session -t $SESSION 
fi

#start main launch with the new session/window
tmux new -d -s "$SESSION" -n "$WINDOW"

#split screen enough times to create one pane for each script we are launching
for (( i=0; i < LAUNCH_FILE_NUM - 1; i++ )); do
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

echo "Successfully launched paxi_bot launch files for live mapping"
echo " - To open the tmux session please run: tmux attach -t $SESSION"
echo " - To close the tmux later you can run: tmux kill-session -t $SESSION"
