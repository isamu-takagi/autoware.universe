cmake_minimum_required(VERSION 3.14)
project(system_diagnostic_viewer)

find_package(autoware_cmake REQUIRED)
autoware_package()
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)
install(FILES plugin.xml DESTINATION share/${PROJECT_NAME})
install(PROGRAMS script/system_diagnostic_monitor DESTINATION lib/${PROJECT_NAME})
ament_auto_package(INSTALL_TO_SHARE script)
