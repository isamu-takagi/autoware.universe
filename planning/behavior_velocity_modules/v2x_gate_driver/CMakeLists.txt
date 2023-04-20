cmake_minimum_required(VERSION 3.14)
project(v2x_gate_driver)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/loader.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  v2x_gate_driver::Loader
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_auto_package(INSTALL_TO_SHARE launch)
