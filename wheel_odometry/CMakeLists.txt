cmake_minimum_required(VERSION 3.20)
project(wheel_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(wheel_only_node src/wheel_only_node.cpp)
target_include_directories(wheel_only_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(wheel_only_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
)

install(TARGETS wheel_only_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/run_smoke.sh
  scripts/trajectory_plotter.py
  scripts/dump_wheel_csv.py
  scripts/odom_to_tum.py
  scripts/eval_glim.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
