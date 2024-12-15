cmake_minimum_required(VERSION 3.5)
project(rl_fra2mo_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Trova le dipendenze
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(dynamic_tf_publisher src/odom_bl_tf.cpp)
ament_target_dependencies(dynamic_tf_publisher rclcpp nav_msgs geometry_msgs tf2 tf2_ros)

# Installa i file necessari
install(DIRECTORY launch urdf models meshes worlds config src maps scripts rviz_conf
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS dynamic_tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)
# Install Python executables
install(PROGRAMS
  scripts/reach_goal.py
  scripts/follow_waypoints.py
  scripts/follow_map_points.py
  DESTINATION lib/${PROJECT_NAME}
)

find_package(tf2_msgs REQUIRED)
add_executable(fra2mo_pose_xy_node src/fra2mo_pose_xy_node.cpp)
ament_target_dependencies(fra2mo_pose_xy_node rclcpp geometry_msgs std_msgs tf2_msgs tf2_geometry_msgs tf2 tf2_ros)
install(TARGETS fra2mo_pose_xy_node DESTINATION lib/${PROJECT_NAME})


ament_package()


