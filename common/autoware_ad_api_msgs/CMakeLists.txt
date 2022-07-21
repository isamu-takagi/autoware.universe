cmake_minimum_required(VERSION 3.14)
project(autoware_ad_api_msgs)

find_package(autoware_cmake REQUIRED)
autoware_package()

rosidl_generate_interfaces(${PROJECT_NAME}
  common/msg/ResponseStatus.msg
  interface/srv/InterfaceVersion.srv
  planning/msg/PlanningFactor.msg
  planning/msg/PlanningFactorArray.msg
  planning/msg/PlanningState.msg
  planning/srv/AllowStart.srv
  routing/msg/RouteState.msg
  routing/msg/Route.msg
  routing/msg/RouteOptional.msg
  routing/msg/RoutePrimitive.msg
  routing/msg/RouteSegment.msg
  routing/srv/ClearRoute.srv
  routing/srv/SetRoute.srv
  routing/srv/SetRoutePoints.srv
  localization/msg/LocalizationInitializationState.msg
  localization/srv/InitializeLocalization.srv
  operation/mode/msg/OperationModeState.msg
  operation/mode/srv/ChangeOperationMode.srv
  fail_safe/msg/MrmState.msg
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_auto_package()
