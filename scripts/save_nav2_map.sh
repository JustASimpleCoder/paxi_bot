printf "Note You must be running ros2 slam_toolbox_launch and nav2_async_live_mapping_launch \n"
printf "To run live mapping for paxi_bot, run live_mapping_launch.sh in the /paxi_bot/scripts directiory \n\n"


MAP_FILENAME="paxi_bot_map"
RAN_FROM_SCRIPT_DIR=false


if [[ "$PWD" != */paxi_bot/scripts && "$PWD" != */paxi_bot/paxi_ws ]]; then
    printf "Saving map failed please run this from ~/paxi_bot/scripts directory or the ~/paxi_bot/paxi_ws directory \n\n"
    exit 1
fi

if [[ "$PWD" = */paxi_bot/scripts ]]; then
    printf "Moving to paxi_ws to save the map\n"
    cd .. 
    cd paxi_ws
    RAN_FROM_SCRIPT_DIR=true
fi

if [[ $# -lt 1 ]]; then
    printf "Saving map with default name $MAP_FILENAME.pgm, this will overide previous map if it already exists\n"
else
    if [[ -n "$1" ]]; then
        MAP_FILENAME="$1"
        printf "Saving map with given name $MAP_FILENAME.pgm\n"
    else
        printf "filename given in arguement list is invalid, saving map to default $MAP_FILENAME.pgm"
    fi
fi

source install/setup.bash
printf "Running ROS2command [ros2 run nav2_map_server map_saver_cli -f $MAP_FILENAME]\n\n"
ros2 run nav2_map_server map_saver_cli -f $MAP_FILENAME