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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../paxi_ws" && pwd)"

cd $WORKSPACE_DIR 

printf "Reformating C++ files.... \n\n"
ament_uncrustify --reformat src/paxi_hardware/ src/paxi_calibrate/ src/paxi_description/ src/paxi_msgs/


printf "Reformating python files.... \n\n"
ament_black --reformat src/paxi_bringup/ src/paxi_data_analysis/ src/paxi_bringup/
