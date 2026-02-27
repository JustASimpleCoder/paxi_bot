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

set -e

# Determine ROS distro
if [[ -n "${ROS_DISTRO}" ]]; then
    distro=${ROS_DISTRO}
else
    distro="humble"
fi

echo "Installing ROS ${distro} packages that paxi_bot depends on"

sudo apt update

dependencies_ros_packages=(
    ros-"$distro"-ament-cmake
    ros-"$distro"-navigation2
    ros-"$distro"-nav2-bringup
    ros-"$distro"-robot-localization
    ros-"$distro"-slam-toolbox
    # ros-"$distro"-joint-state-publisher-gui
    ros-"$distro"-joint-state-publisher
    ros-"$distro"-ros2-control
    ros-"$distro"-ros2-controllers
    ros-"$distro"-amcl
)

successful_install_packages=()
failed_install_packages=()

for pkg in "${dependencies_ros_packages[@]}"; do
    echo "Attempting to install package $pkg"
    if sudo apt install -y "$pkg"; then
        successful_install_packages+=("$pkg")
        echo "Successfully installed package $pkg"
    else
        failed_install_packages+=("$pkg")
        echo "Failed to install package $pkg"
    fi
done

if [[ ${#failed_install_packages[@]} -ne 0 ]]; then
    echo "Failed to install ${#failed_install_packages[@]} ROS packages:"
    printf '%s\n' "${failed_install_packages[@]}"
fi

echo "Successfully installed ${#successful_install_packages[@]} ROS packages:"
printf '%s\n' "${successful_install_packages[@]}"

