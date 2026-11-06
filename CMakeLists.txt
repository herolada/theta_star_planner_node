cmake_minimum_required(VERSION 3.5)
project(theta_star_planner_node)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
else()
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# ROS-free planning core (vendored + adapted nav2_theta_star_planner search, costmap
# inflation); depends only on nav2_costmap_2d::Costmap2D.
add_library(theta_star_core STATIC
  src/theta_star.cpp
  src/costmap_inflation.cpp
)
set_target_properties(theta_star_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(theta_star_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)
target_link_libraries(theta_star_core
  PUBLIC
    nav2_costmap_2d::nav2_costmap_2d_core
)

add_executable(theta_star_planner_node
  src/theta_star_planner_node.cpp
)
target_link_libraries(theta_star_planner_node
  theta_star_core
  ${geometry_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
)

install(TARGETS theta_star_planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
