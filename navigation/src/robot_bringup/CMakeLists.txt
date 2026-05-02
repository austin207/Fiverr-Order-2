cmake_minimum_required(VERSION 3.8)
project(robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
  DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES navfn_metrics_plugin.xml smac_metrics_plugin.xml
  DESTINATION share/${PROJECT_NAME}/
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_navfn_planner REQUIRED)
find_package(nav2_smac_planner REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ── NavFn Metrics Wrapper Plugin (client's original, wraps NavfnPlanner) ─────
add_library(navfn_metrics_planner SHARED
  src/navfn_metrics_planner.cpp
)
ament_target_dependencies(navfn_metrics_planner
  rclcpp rclcpp_lifecycle nav2_core nav2_navfn_planner
  nav2_costmap_2d pluginlib std_msgs geometry_msgs nav_msgs tf2_ros
)
pluginlib_export_plugin_description_file(nav2_core navfn_metrics_plugin.xml)
install(TARGETS navfn_metrics_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── Smac Metrics Wrapper Plugin (wraps SmacPlanner2D, works in all map types) ─
add_library(smac_metrics_planner_2d SHARED
  src/smac_metrics_planner_2d.cpp
)
ament_target_dependencies(smac_metrics_planner_2d
  rclcpp rclcpp_lifecycle nav2_core nav2_smac_planner
  nav2_costmap_2d pluginlib std_msgs geometry_msgs nav_msgs tf2_ros
)
pluginlib_export_plugin_description_file(nav2_core smac_metrics_plugin.xml)
install(TARGETS smac_metrics_planner_2d
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
