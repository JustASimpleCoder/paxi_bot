#!/usr/bin/env bash
printf "Note You must be running ros2 slam_toolbox_launch and nav2_async_live_mapping_launch\n"
printf "To run live mapping for paxi_bot, run live_mapping_launch.sh in the /paxi_bot/scripts directiory \n\n"

MAP_FILENAME="paxi_bot_map"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"
SAVE_MAP_DIR="$WORKSPACE_DIR"
SAVE_MAP_DIR+="/src/paxi_description/maps"

RED=$'\e[0;31m'
GREEN=$'\e[0;32m'
NC=$'\e[0m'

cd $WORKSPACE_DIR

if [[ $# -lt 1 ]]; then
    printf "Saving map with default name $MAP_FILENAME.pgm"
    printf "This will overide previous map if it already exists\n"
else
    if [[ -n "$1" ]]; then
        MAP_FILENAME="$1"
        printf "Saving map with given name $MAP_FILENAME.pgm\n"
    else
        printf "filename given in arguement list is invalid, saving map to default $MAP_FILENAME.pgm"
    fi
fi

source install/setup.bash
printf "Running command:"
printf " ros2 run nav2_map_server map_saver_cli -f $MAP_FILENAME\n\n"
ros2 run nav2_map_server map_saver_cli -f $MAP_FILENAME

MAP_FILENAME_PGM=$MAP_FILENAME
MAP_FILENAME_PGM+=".pgm"

MAP_FILENAME_YAML=$MAP_FILENAME
MAP_FILENAME_YAML+=".yaml"

if [[ -e $MAP_FILENAME_PGM &&  -e $MAP_FILENAME_YAML ]]; then 
    printf "\n${GREEN}The map $MAP_FILENAME_PGM with YAML file $MAP_FILENAME_YAML was create sucessfuly.${NC}\n"
    printf "Moving to paxi_description/maps ....\n"
    mv $MAP_FILENAME_PGM $SAVE_MAP_DIR
    mv $MAP_FILENAME_YAML $SAVE_MAP_DIR
    printf "${GREEN}Moved $MAP_FILNAME_PGM and $MAP_FILENAME_YAML to $SAVE_MAP_DIR ${NC}\n"
else
    printf "\n${RED}NAV2 failed to create the map $MAP_FILENAME\n"
    printf "If NAV2 map server is running on a seperate machine, map may be too large to send over DDS\n"
    printf "Hint: is slam toolbox and nav2 async live mapping running?\n$"
    printf "Hint: run this on the same machine running nav2 and use SCP to get the map over SSH${NC}\n$"
fi


# TO_DO: update nav2_param file so map server gets new map to work with. 
# CONFIG_FILE="$WORKSPACE_DIR"
# CONFIG_FILE+="/src/paxi_description/config/nav2_params.yaml"
# NEW_YAML_NAME="$MAP_FILENAME_YAML"

# # Replace yaml_filename inside map_server
# yq -i '
#   .map_server.ros__parameters.yaml_filename = strenv($NEW_YAML_NAME)
# ' "$CONFIG_FILE"

# printf "Updated yaml_filename in nav2_params yaml in paxi_description/config $NEW_YAML_NAME"