cmake_minimum_required(VERSION 3.22.1)
project(paxi_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

set(PKG_DEPENDENCIES 
  rclcpp
  std_msgs
)

foreach(PKG IN LISTS PKG_DEPENDENCIES)
  find_package(${PKG} REQUIRED)
endforeach(PKG IN LIST PKG_DEPENDENCIES)

add_library(paxi_common INTERFACE)
target_include_directories(paxi_common INTERFACE
  $<BUILD_INTERFACE::${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE::/include>

)
target_compile_features(paxi_common INTERFACE c_std_99 cxx_std_17)


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

ament_package()
