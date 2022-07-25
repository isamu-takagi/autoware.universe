cmake_minimum_required(VERSION 3.14)
project(pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME}
  src/pose_initializer_core.cpp
  src/initialpose_rviz_helper.cpp
  src/initialpose_gnss_helper.cpp
  src/fitting_to_map_height.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/pose_initializer_node.cpp
)

ament_auto_add_executable(${PROJECT_NAME}_rviz
  src/pose_initializer_rviz.cpp
)

# fmt often fails to automatically link so manually specify here
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} fmt)

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${test_name} fmt)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_copy_vector_to_array.cpp)
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
