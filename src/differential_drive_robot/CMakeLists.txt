cmake_minimum_required(VERSION 3.8)
project(differential_drive_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FusedPose.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(sensor_fusion_node src/sensor_fusion_node.cpp)
ament_target_dependencies(sensor_fusion_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)
target_link_libraries(sensor_fusion_node "${cpp_typesupport_target}")

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node
  geometry_msgs
  rclcpp
)
target_link_libraries(controller_node "${cpp_typesupport_target}")

install(PROGRAMS
  src/imu_publisher.py
  src/odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  controller_node
  sensor_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
