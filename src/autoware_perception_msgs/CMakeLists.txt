cmake_minimum_required(VERSION 3.8)
project(autoware_perception_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObjectClassification.msg"
  "msg/Shape.msg"
  "msg/DetectedObjectKinematics.msg"
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "msg/TrackedObjectKinematics.msg"
  "msg/TrackedObject.msg"
  "msg/TrackedObjects.msg"
  DEPENDENCIES std_msgs geometry_msgs unique_identifier_msgs
)

ament_package()
