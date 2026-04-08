cmake_minimum_required(VERSION 3.20)
project(leg_odometry)

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
find_package(orocos_kdl REQUIRED)

# GTSAM (system aarch64 install at /usr/local)
find_package(GTSAM REQUIRED)

# KDL parser (use ament find)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

# === ROS2 Node ===
add_executable(leg_odom_node src/leg_odom_node.cpp)
target_include_directories(leg_odom_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GTSAM_INCLUDE_DIR}
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(leg_odom_node
  gtsam
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(leg_odom_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros builtin_interfaces
  kdl_parser urdf
)

# === Offline evaluation (CSV, no ROS2 deps at runtime) ===
add_executable(leg_odom_app src/leg_odom_app.cpp)
target_include_directories(leg_odom_app PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(leg_odom_app gtsam)

# === KDL test ===
add_executable(test_kinematics src/test_kinematics.cpp)
target_include_directories(test_kinematics PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(test_kinematics
  ${orocos_kdl_LIBRARIES}
)
ament_target_dependencies(test_kinematics
  kdl_parser urdf
)

# Install
install(TARGETS leg_odom_node leg_odom_app test_kinematics
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/generate_sim_data.py
  scripts/evaluate_sim.py
  scripts/evaluate_hybrid.py
  scripts/bag_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
