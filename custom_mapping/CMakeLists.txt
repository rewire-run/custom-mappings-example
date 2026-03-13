cmake_minimum_required(VERSION 3.14)
project(custom_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Arrow.msg"
  "msg/BoundingBox2D.msg"
  "msg/BoundingBox3D.msg"
  "msg/BoundingBoxes2D.msg"
  "msg/BoundingBoxes3D.msg"
  "msg/Detection.msg"
  "msg/Detection2D.msg"
  "msg/Detections.msg"
  "msg/Detections2D.msg"
  "msg/ForceVector.msg"
  "msg/GpsCoordinate.msg"
  "msg/GpsReading.msg"
  "msg/MotorStatus.msg"
  "msg/RigidBody.msg"
  "msg/RigidBodies.msg"
  "msg/Trajectory.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_executable(publisher src/publisher.cpp)
ament_target_dependencies(publisher rclcpp std_msgs geometry_msgs)
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(publisher "${typesupport_target}")

install(TARGETS publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
