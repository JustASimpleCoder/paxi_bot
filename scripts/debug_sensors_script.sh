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

SESSION="live_debugging"
WINDOW="live_debugging"

ROS_COMMANDS=(
    "source install/setup.bash"
    "ros2 topic echo"
)

TOPIC_NAME=(
    /l_wheel/pos
    /r_wheel/pos
    /l_wheel/vel
    /r_wheel/vel
)
TOPIC_NUM=${#TOPIC_NAME[@]}

#check if session already exist, kill it if it does
if tmux has-session -t "$SESSION"  2>/dev/null; then
    echo "[WARNING] tmux session [$SESSION] already exists, killing old session"
    tmux kill-session -t $SESSION 
fi

#start main launch with the new session/window
tmux new -d -s "$SESSION" -n "$WINDOW"

#split screen enough times to create one pane for each script we are launching
for (( i=0; i < TOPIC_NUM - 1; i++ )); do
    tmux split-window -t "$SESSION:$WINDOW"
done

#make it look pretty with boxes
tmux select-layout -t "$SESSION":"$WINDOW" tiled

#source install and launch each file in all panes
for pane in $(tmux list-panes -F '#P'); do
    if [[ $pane -lt ${#TOPIC_NAME[@]} ]]; then
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[0]}" C-m
        tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[1]} ${TOPIC_NAME[$pane]}" C-m
    fi
done

echo "Successfully "
echo " - To open the tmux session please run: tmux attach -t $SESSION"
echo " - To close the tmux later you can run: tmux kill-session -t $SESSION"
