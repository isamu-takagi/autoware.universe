cmake_minimum_required(VERSION 3.5)
project(vehicle_cmd_gate2)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  "src/vehicle_cmd_gate.cpp"
  "src/command/interface.cpp"
  "src/command/diagnostic.cpp"
  "src/command/disengage.cpp"
  "src/command/emergency.cpp"
  "src/command/publisher.cpp"
  "src/command/selector.cpp"
  "src/command/subscription.cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "vehicle_cmd_gate2::VehicleCmdGate"
  EXECUTABLE vehicle_cmd_gate
)

ament_auto_package(INSTALL_TO_SHARE launch config)
