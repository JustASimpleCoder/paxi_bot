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


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"

SLLIDAR_DIR_SCRIPTS="$WORKSPACE_DIR"
SLLIDAR_DIR_SCRIPTS+="/src/sllidar_ros2/scripts"

SLAMKIT_DIR_SCRIPTS="$WORKSPACE_DIR"
SLAMKIT_DIR_SCRIPTS+="/src/slamkit_ros2/scripts"

SLAMKIT_DIR="$WORKSPACE_DIR"
SLAMKIT_DIR+="/src/slamkit_ros2"

RED=$'\e[0;31m'
GREEN=$'\e[0;32m'
NC=$'\e[0m'

cd $WORKSPACE_DIR

echo "\nInstalling vcstool\n\n"

sudo apt install python3-vcstool

echo "\nImporting repos from paxi.repos\n\n"
vcs import src < src/paxi.repos


cd  $SLLIDAR_DIR_SCRIPTS

echo "\ncreating udev rules for sllidar \n\n"
sudo ./create_udev_rule.sh

cd $SLAMKIT_DIR_SCRIPTS
echo "\nCreating udev rules for slamtec \n\n"
sudo ./add_udev.sh

cd $SLAMKIT_DIR
echo "\nCreating udev rules for slamtec \n\n"
git submodule update --init

cd $WORKSPACE_DIR
echo "\nInstalling ROS2 dependencies \n\n"
rosdep update
rosdep install --from-paths src --ignore-src -r -y