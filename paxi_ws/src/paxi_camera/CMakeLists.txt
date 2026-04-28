cmake_minimum_required(VERSION 3.8)
project(paxi_visual_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    rclcpp
    nav_msgs
    sensor_msgs
    paxi_msgs
    paxi_common
    OpenCV
    cv_bridge
)

find_package(ament_cmake REQUIRED)

foreach(PKG IN LISTS PACKAGE_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach()

add_executable(camera_node src/camera.cpp src/camera_node.cpp)
target_include_directories(camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(camera_node  PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(camera_node PUBLIC ${PACKAGE_DEPENDENCIES})

install(TARGETS camera_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
