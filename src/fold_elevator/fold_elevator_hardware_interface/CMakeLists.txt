cmake_minimum_required(VERSION 3.8)
project(fold_elevator_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_control REQUIRED) 
find_package(fold_elevator_msgs REQUIRED) 

add_library(
  ${PROJECT_NAME}
  SHARED
  src/socketcan_common.cpp
  src/socketcan_id.cpp
  src/socketcan_driver.cpp
  src/Ti5Robot_cra_driver.cpp
  src/fold_elevator_hardware_interface.cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE -fvisibility=default)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  rclcpp_lifecycle 
  hardware_interface 
  pluginlib 
  can_msgs
  std_msgs
  std_srvs
  ros2_control
  fold_elevator_msgs
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME "fold_elevator_hardware_interface"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/install/${PROJECT_NAME}/lib"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/install/${PROJECT_NAME}/lib"
)

pluginlib_export_plugin_description_file(hardware_interface fold_elevator_hardware_interface.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  can_msgs
  ros2_control
)

ament_package()