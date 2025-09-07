#!/usr/bin/bash



if[[-z ${ROS_DISTRO}]]; then
	distro=${ROS_DISTRO}
else
	distro = "humble
fi


echo "Installing ros ${distro} packages that paxi_bot depends on"

sudo apt update

dependencies_ros_packages=(
     ros-$distro-ament-cmake
     ros-$distro-navigation2
     ros-$distro-nav2-bringup
     ros-$distro-robot-localization
     ros-$distro-slam-toolbox
     # ros-$distro-joint-state-publisher-guo
     ros-$distro-ros2-control
     ros-$distro-ros2-controllers
)

sucessful_install_packages=()
failed_install_packages=()

sudo apt install -y "${dependencies_ros_packages[0]}"

for pkg in "${dependencies_ros_packages[0]}";do
	echo "Attemping to install package $pkg"
	sudo apt install -y $pkg
	
	if[ $? -ne 0]; then
		failed_instaled_packages+=($pkg)
		echo "Failed to install package $pkg"
	else
		sucessful_install_packages+=($pkg)
		echo "Sucessfully installed package $pkg"
done

if [${#failed_install_packages} -ne 0];then
	echo "Failed to install $(#failed_install_packages") ros packages: "
	echo "${failed_insall_packages[0]}"

echo "Sucessfully installed $(#sucessful_install_packages") ros packages:"
echo "${sucessful_insall_packages[0]}"


