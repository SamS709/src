cmake_minimum_required(VERSION 3.16)
project(rasptank_diff_drive LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  rasptank_diff_drive
  SHARED
  hardware/diffbot_system.cpp
)
target_compile_features(rasptank_diff_drive PUBLIC cxx_std_17)
target_include_directories(rasptank_diff_drive PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/rasptank_diff_drive>
)
ament_target_dependencies(
  rasptank_diff_drive PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface rasptank_diff_drive.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/rasptank_diff_drive
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/rasptank_diff_drive
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/rasptank_diff_drive
)
install(TARGETS rasptank_diff_drive
  EXPORT export_rasptank_diff_drive
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  rasptank_diff_drive/motor_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_2_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_2_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_2_launch test/test_diffbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_rasptank_diff_drive HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
