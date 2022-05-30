cmake_minimum_required(VERSION 3.14)
project(autoware_ad_api_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  common/msg/ResponseStatus.msg
  interface/srv/InterfaceVersion.srv
  driving/msg/DrivingState.msg
  driving/srv/DrivingEngage.srv
  motion/msg/MotionState.msg
  route/msg/RouteState.msg
  route/srv/RouteClear.srv
  route/srv/RouteSet.srv
  pose_reset/msg/PoseResetState.msg
  pose_reset/srv/PoseResetRequest.srv
  fail_safe/msg/FailSafeState.msg
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_auto_package()
