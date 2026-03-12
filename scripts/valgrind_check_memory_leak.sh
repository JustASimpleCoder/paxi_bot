#!/usr/bin/bash
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

help_menu(){
  echo "Flags: "
  echo "  -b            Build system with CMAKE_BUILD_TYPE=Debug"
  echo "  -f <file>     Specify launch file"
  echo "  -h            Show help"
  echo ""
  echo "Usage: "
  echo "$0 -f <launch_file>       runs valgrind on a launch file"
  echo "$0 -b -f <launch_file>    builds system with CMAKE_BUILD_TYPE=debug \
                                    and runs valgrind on the specified file"
}

colcon_build_debug_flag(){
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"

  cd "$WORKSPACE_DIR"
  rm -rf /build /install /log
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
  source install/setup.bash

}

BUILD_WITH_DEBUG_FLAG=false
LAUNCH_FILE=""

while getopts "hbf:" opt; do
  case "$opt" in 
  h)
    help_menu
    exit 0
    ;;
  b)
    BUILD_WITH_DEBUG_FLAG=true
    ;;
  f)
    LAUNCH_FILE=$OPTARG
    ;;
  *)
    echo "Invalid usage, flag not recognized"
    echo "Try $0 -h for help"
    exit 1
    ;;
  esac
done

if [[ -z "$LAUNCH_FILE" ]]; then
  echo "Error: Launch file required"
  echo "Try $0 -h for help"
  exit 1
fi

if [[ "$BUILD_WITH_DEBUG_FLAG" == true ]]; then
  colcon_build_debug_flag
fi

valgrind --leak-check=full --show-leak-kinds=all ros2 launch paxi_bringup $LAUNCH_FILE
