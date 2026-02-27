#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"

cd $WORKSPACE_DIR 

printf "Reformating C++ files.... \n\n"
ament_uncrustify --reformat src/paxi_hardware/ src/paxi_calibrate/ src/paxi_description/ src/paxi_msgs/


printf "Reformating python files.... \n\n"
ament_black --reformat src/paxi_bringup/ src/paxi_data_analysis/ src/paxi_bringup/
