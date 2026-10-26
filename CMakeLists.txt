cmake_minimum_required(VERSION 3.8)
project(tiago_velocity_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(urdf REQUIRED)

# Build Controller Plugin Shared Library
add_library(${PROJECT_NAME} SHARED 
  src/tiago_arm_velocity_controller.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  realtime_tools
  urdf
)

# Export plugin description via pluginlib
pluginlib_export_plugin_description_file(controller_interface tiago_arm_velocity_controller_plugins.xml)

# Install rules
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES tiago_arm_velocity_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install configuration and launch directories if they exist
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_load_controller.cpp")
    ament_add_gmock(test_load_controller test/test_load_controller.cpp)
    target_include_directories(test_load_controller PRIVATE include)
    target_compile_definitions(test_load_controller PRIVATE TEST_PARAMS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test")
    ament_target_dependencies(test_load_controller
      rclcpp
      controller_manager
      hardware_interface
      pluginlib
      ros2_control_test_assets
    )
  endif()
endif()

# Optional plugin verification helper
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/verify_plugin_registration.cpp")
  add_executable(verify_plugin_registration test/verify_plugin_registration.cpp)
  target_include_directories(verify_plugin_registration PRIVATE include)
  ament_target_dependencies(verify_plugin_registration
    controller_interface
    pluginlib
  )
  install(
    TARGETS verify_plugin_registration
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Export dependencies and library targets for downstream packages
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  realtime_tools
  urdf
)

ament_package()