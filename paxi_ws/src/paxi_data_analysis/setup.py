from setuptools import find_packages, setup

package_name = "paxi_data_analysis"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pandas", "scikit-learn"],
    zip_safe=True,
    maintainer="j",
    maintainer_email="thejacobcohen@gmail.com",
    description=" linear regresion on generated data mapping target RPM to PWM signals",
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": ["lin_reg = paxi_data_analysis.lin_reg_node:main",
                            "inertial_node = paxi_data_analysis.find_inertial_properties_link:main"],
    },
)
