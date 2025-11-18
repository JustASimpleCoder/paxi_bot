#!/usr/bin/env bash

# This script opens starts a new tmux session and window in the background.
# It opens 4 panes, running ros commands to start paxi's launch files.
# this script assumes you have added 'source /opt/ros/humble/setup.bash' to bashrc

#Quick path check
if [[ "$PWD" != */paxi_bot/scripts ]]; then 
    echo "You must run this from the paxi_bot/scripts directory"
    echo "The current direction is [$PWD] "
    echo "hint: try running cd ~/paxi_bot/scripts"
    exit 1
fi

cd ..
cd paxi_ws


SESSION="live_mapping"
WINDOW="live_mapping"

PANE_NAMES=(
    "main_bringup"
    "live_async_mapping"
    "live_display"
    "manual_control"
)

ROS_COMMANDS=(
    "source install/setup.bash"
    "ros2 launch paxi_bringup"
)

SCRIPT_NAMES=(
    "main_bringup.py"
    "live_async_mapping.py"
    "live_display.py"
    "manual_control.py"
)

#kill any old previous session that may be running
tmux kill-session -t $SESSION 

#start main launch with the new session/window
tmux new -d -s "$SESSION" -n "$WINDOW"

#split screen three times to create one pane for each script we are launching
for (( i=0; i<3; i++ )); do
    tmux split-window -t "$SESSION:$WINDOW"
done

#make it look pretty with boxes
tmux select-layout -t "$SESSION":"$WINDOW" tiled

#source install and launch each file in all panes
for pane in $(tmux list-panes -F '#P'); do
    if [[ $pane -lt ${#SCRIPT_NAMES[@]} ]]; then
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[0]}" C-m
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[1]} ${SCRIPT_NAMES[$pane]}" C-m
    fi
done

echo "Sucessfully launched paxi_bot launch files for live mapping"
echo " - To open the tmux session please run: tmux attach -t $SESSION"
echo " - To close the tmux later you can run: tmux kill-session -t $SESSION"
