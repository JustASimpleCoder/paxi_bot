from setuptools import find_packages, setup

package_name = "paxi_bag_to_csv"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="j",
    maintainer_email="thejacobcohen@gmail.com",
    description="Converts ros2bag into csv",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["bag2csv = paxi_bag_to_csv.bag2csv:main"],
    },
)
