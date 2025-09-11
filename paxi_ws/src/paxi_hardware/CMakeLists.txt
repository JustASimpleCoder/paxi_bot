cmake_minimum_required(VERSION 3.8)
project(paxi_hardware)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies

set(PACKAGE_DEPENDENCIES
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)


find_package(ament_cmake REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach(PKG)


add_library(paxi_hardware SHARED
  src/paxi_interface.cpp
  src/hoverboard_protocol.cpp
  src/serial_communication.cpp
  src/paxi_interface_node.cpp

)
target_compile_features(paxi_hardware PUBLIC cxx_std_20)
target_include_directories(paxi_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/>
)


ament_target_dependencies(
  paxi_hardware PUBLIC
  ${PACKAGE_DEPENDENCIES}
)

pluginlib_export_plugin_description_file(hardware_interface paxi_hardware.xml)


#if we want to turn it into individual node to spin

# add_executable(paxi_interface_node src/paxi_interface_node.cpp)
# target_compile_features(paxi_interface_node PUBLIC cxx_std_20)
# ament_target_dependencies(paxi_interface_node PUBLIC ${PACKAGE_DEPENDENCIES})
# target_link_libraries(paxi_interface_node PRIVATE paxi_hardware)

install(
  DIRECTORY include/
  DESTINATION include/
)

# install(
#   DIRECTORY description/launch description/ros2_control description/urdf
#   DESTINATION share/ros2_control_demo_example_11
# )
# install(
#   DIRECTORY bringup/launch bringup/config
#   DESTINATION share/ros2_control_demo_example_11
# )


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_Dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
