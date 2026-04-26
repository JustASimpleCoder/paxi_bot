cmake_minimum_required(VERSION 3.8)
project(paxi_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    #realtime_tools
    nav_msgs
    geometry_msgs
    sensor_msgs
    paxi_msgs
    paxi_common
)

find_package(ament_cmake REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

add_library(
  paxi_hardware SHARED
  src/paxi_interface.cpp
  src/hoverboard_protocol.cpp
  src/serial_port.cpp
  src/paxi_interface_node.cpp
  src/encoder.cpp
  src/imu.cpp
  src/hardware_worker.cpp
  src/hardware_manager.cpp
)

target_compile_features(paxi_hardware PUBLIC cxx_std_17)
target_include_directories(
  paxi_hardware PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>)

ament_target_dependencies(paxi_hardware PUBLIC ${PACKAGE_DEPENDENCIES})

pluginlib_export_plugin_description_file(hardware_interface paxi_hardware.xml)

install(DIRECTORY include/ DESTINATION include/)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)

  # set(ament_cmake_uncrustify_FOUND TRUE) set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_cppcheck_FOUND REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(${PROJECT_NAME}_encoder_test test/encoder_test.cpp)
  target_include_directories(
    ${PROJECT_NAME}_encoder_test
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
      $<INSTALL_INTERFACE:include>)

  ament_target_dependencies(${PROJECT_NAME}_encoder_test
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_encoder_test paxi_hardware)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
