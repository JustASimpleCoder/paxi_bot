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

from glob import glob

import os

from setuptools import find_packages, setup


package_name = 'paxi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'paxi_bringup'),
            glob('paxi_bringup/*.py'),
        ),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'controller'), glob('controller/*.yaml')),
        (
            os.path.join('share', package_name, 'data_collection'),
            glob('data_collection/*.yaml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j',
    maintainer_email='thejacobcohen@gmail.com',
    description='launch files for paxi',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'controller.py',
            # 'live_display.py',
            # 'main_bringup.py',
            # 'manual_control.py',
            # 'nav2.py',
            # 'static_display.py'
        ]
    },
)
