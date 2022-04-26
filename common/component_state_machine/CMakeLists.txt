cmake_minimum_required(VERSION 3.5)
project(component_state_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/machine.cpp
)

ament_auto_add_executable(main
  test/main.cpp
)

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(gtest_${PROJECT_NAME} test/gtest.cpp)
  target_link_libraries(gtest_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
