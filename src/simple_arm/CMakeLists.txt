cmake_minimum_required(VERSION 3.8)
project(simple_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(libserial REQUIRED libserial)

# Include directories for C++ headers (create include/simple_arm/ if you move .hpp there)
include_directories(include)

# Build C++ library (your hardware plugin)
add_library(arm_hardware_plugin SHARED
  src/arm_hardware_interface.cpp
  src/sts_driver.cpp
)
target_include_directories(arm_hardware_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${libserial_INCLUDE_DIRS}
)
ament_target_dependencies(arm_hardware_plugin PUBLIC
  hardware_interface
  pluginlib
  rclcpp
)
target_link_libraries(arm_hardware_plugin PUBLIC
  ${libserial_LIBRARIES}
)


# Export the plugin description file
pluginlib_export_plugin_description_file(hardware_interface arm_hardware_plugin.xml)

# Install C++ targets
install(TARGETS arm_hardware_plugin
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install plugin XML
install(FILES arm_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


# Install directories (configs, launches, etc.)
install(DIRECTORY config description launch meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclpy
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()