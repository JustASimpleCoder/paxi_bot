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

if [[ "$EUID" -ne 0 ]]; then
  echo "[ERROR] Please run the script as a sudo user"
  exit 1
fi

echo "Please ensure you sudo apt update && sudo apt upgrade prior to running this script"
echo "Please ensure you have ran reboot now prior to running this script"

# Disable ZRAM:
sudo systemctl disable nvzramconfig
# Create 4GB swap file
sudo fallocate -l 4G /mnt/4GB.swap
sudo chmod 600 /mnt/4GB.swap
sudo mkswap /mnt/4GB.swap

# if having problems you can run the below
# sudo vi /etc/fstab
# # Add this line at the bottom of the file
# /mnt/4GB.swap swap swap defaults 0 0
# # Reboot
# sudo reboot now

# install dependencies
sudo wget -qO- https://docs.luxonis.com/install_dependencies.sh | bash

cd ~/
# create venv for dpethai if needed
sudo apt install python3-venv
python3 -m venv depthai
source depthai/bin/activate

# Clone github repository
git clone https://github.com/luxonis/depthai-python.git
cd depthai-python
python3 examples/install_requirements.py

echo "export OPENBLAS_CORETYPE=ARMV8" >> ~/.bashrc
