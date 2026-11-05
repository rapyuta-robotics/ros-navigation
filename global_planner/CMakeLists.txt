cmake_minimum_required(VERSION 3.20)
project(global_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

set(dependencies
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_ros_common
  nav2_util
  nav_msgs
  pluginlib
  rcl_interfaces
  rclcpp
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

add_library(${PROJECT_NAME} SHARED
  src/quadratic_calculator.cpp
  src/dijkstra.cpp
  src/astar.cpp
  src/grid_path.cpp
  src/gradient_path.cpp
  src/planner_core.cpp
  src/parameter_handler.cpp
  src/global_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
foreach(dep IN LISTS dependencies)
  target_link_libraries(${PROJECT_NAME} PUBLIC ${${dep}_TARGETS})
endforeach()

pluginlib_export_plugin_description_file(nav2_core bgp_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  foreach(test_name
    test_planner_core
    test_global_planner)
    ament_add_ros_isolated_gtest(${test_name} test/${test_name}.cpp)
    target_link_libraries(${test_name} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

ament_package()
