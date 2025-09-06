import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'paxi_nav2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'paxi_nav2'), glob("paxi_nav2/*.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='j',
    maintainer_email='thejacobcohen@gmail.com',
    description='Nav2 launch files for autonomous navigation and mappping',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)