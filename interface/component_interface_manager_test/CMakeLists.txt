cmake_minimum_required(VERSION 3.14)
project(component_interface_manager_test)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(cpp_pub_ros test/cpp/msg/cpp_pub_ros.cpp)
ament_auto_add_executable(cpp_sub_ros test/cpp/msg/cpp_sub_ros.cpp)
ament_auto_add_executable(cpp_cli_ros test/cpp/srv/cpp_cli_ros.cpp)
ament_auto_add_executable(cpp_srv_ros test/cpp/srv/cpp_srv_ros.cpp)
ament_auto_add_executable(cpp_pub_v1  test/cpp/msg/cpp_pub_v1.cpp)
ament_auto_add_executable(cpp_sub_v1  test/cpp/msg/cpp_sub_v1.cpp)
ament_auto_add_executable(cpp_cli_v1  test/cpp/srv/cpp_cli_v1.cpp)
ament_auto_add_executable(cpp_srv_v1  test/cpp/srv/cpp_srv_v1.cpp)
ament_auto_add_executable(cpp_pub_v2  test/cpp/msg/cpp_pub_v2.cpp)
ament_auto_add_executable(cpp_sub_v2  test/cpp/msg/cpp_sub_v2.cpp)
ament_auto_add_executable(cpp_cli_v2  test/cpp/srv/cpp_cli_v2.cpp)
ament_auto_add_executable(cpp_srv_v2  test/cpp/srv/cpp_srv_v2.cpp)

ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR python)

ament_auto_package()
