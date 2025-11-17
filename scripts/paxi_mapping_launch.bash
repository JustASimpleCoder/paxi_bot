#!/usr/bin/env bash

#TODO: Quick path check
# if[[ "$(basename "$PWD")" != "paxi_ws" ]]; then 
#     echo "You must run this from the paxi_ws directory"
#     echo "The current direction is [$PWD] "
#     echo "hint: try running cd ~/paxi_ws"
#     exit 1
# fi

cd ..
cd paxi_ws

#this assumes you have added 'source /opt/ros/humble/setup.bash' to bashrc
SESSION="paxibot_session_main_mapping"
WINDOW="paxibot_window_main_mapping"

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
tmux split-window -t "$SESSION:$WINDOW"
tmux split-window -t "$SESSION:$WINDOW"
tmux split-window -t "$SESSION:$WINDOW" 
tmux select-layout -t "$SESSION":"$WINDOW" tiled

#source install all panes

for pane in $(tmux list-panes -F '#P'); do
    tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[0]}" C-m
    tmux send-keys -t "$SESSION:$WINDOW.$pane" "${ROS_COMMANDS[1]} ${SCRIPT_NAMES[$pane]}" C-m
done

#"${ROS_COMMANDS[0]}; ${ROS_COMMANDS[1]} ${SCRIPT_NAMES[0]}"

#tmux select-layout -t "$SESSION":0 tiled

echo "sucesfully ran paxi_bot script"
echo "To see terminals running, please run:  tmux attach -t paxibot_session_main_mapping"














