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

# this script is used to reformat the repo to ros2 standrards using ament_uncrustify and 
# ament_black
# Note you still need to run ament_cppcheck and ament_cpplint to pass all ros2 test, these
# tools dont have reformat options yet

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"

cd $WORKSPACE_DIR 

export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

CPP_DIRS=(
  "src/paxi_hardware"
  "src/paxi_calibrate"
  "src/paxi_description" 
  "src/paxi_msgs"
  "src/paxi_common"
)

for dir in "${CPP_DIRS[@]}"; do
  printf "checking C++ files in dir {$dir}.... \n\n"
  cd "$WORKSPACE_DIR"
  cd "$dir"
  ament_cppcheck
done
