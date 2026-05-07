cmake_minimum_required(VERSION 3.8)
project(differential_drive_robot)

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

ament_export_dependencies(rosidl_default_runtime)
ament_package()


install(PROGRAMS
  src/imu_publisher.py
  src/odom_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
