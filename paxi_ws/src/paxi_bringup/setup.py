from setuptools import find_packages, setup

import os
from glob import glob


package_name = 'paxi_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'paxi_bringup'), glob('paxi_bringup/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'controller'), glob('controller/*.yaml'))
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
            # "controller.py",
            # "live_display.py",
            # "main_bringup.py",
            # "manual_control.py",   
            # "nav2.py",
            # "static_display.py"           
    ]},
)
