cmake_minimum_required(VERSION 3.8)
project(autoware_control_cmd_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  "src/control_cmd_gate.cpp"
  "src/command/interface.cpp"
  "src/command/selector.cpp"
  "src/command/diagnostics.cpp"
  "src/command/publisher.cpp"
  "src/command/subscription.cpp"
  "src/command/generator.cpp"
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::control_cmd_gate::ControlCmdGate"
  EXECUTABLE "control_cmd_gate_node"
)

if(BUILD_TESTING)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch)
