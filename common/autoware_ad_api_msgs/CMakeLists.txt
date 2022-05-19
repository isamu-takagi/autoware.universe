cmake_minimum_required(VERSION 3.14)
project(autoware_ad_api_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  common/msg/ResponseStatus.msg
  common/msg/ResponseData.msg
  interface/srv/InterfaceVersion.srv
  driving/srv/DrivingEngage.srv
  driving/msg/DrivingState.msg
  motion/msg/MotionState.msg
  route/msg/Route.msg
  route/msg/RouteNotice.msg
  route/msg/RouteState.msg
  route/srv/RouteClear.srv
  route/srv/RouteSet.srv
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_auto_package()
