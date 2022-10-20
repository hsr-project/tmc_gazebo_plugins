cmake_minimum_required(VERSION 3.5.0)
project(tmc_gazebo_plugins)

find_package(ament_cmake REQUIRED)

find_package(angles REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tmc_control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_library(tmc_gazebo_ros_mimic_joint SHARED src/gazebo_ros_mimic_joint.cpp)
ament_target_dependencies(tmc_gazebo_ros_mimic_joint gazebo_dev gazebo_ros rclcpp)

add_library(tmc_gazebo_ros_hrh_gripper SHARED src/gazebo_ros_hrh_gripper.cpp)
ament_target_dependencies(tmc_gazebo_ros_hrh_gripper angles control_msgs control_toolbox gazebo_dev gazebo_ros rclcpp rclcpp_action tmc_control_msgs trajectory_msgs)

add_library(tmc_gazebo_ros_bumper SHARED src/gazebo_ros_bumper.cpp)
ament_target_dependencies(tmc_gazebo_ros_bumper gazebo_ros)

install(TARGETS tmc_gazebo_ros_mimic_joint tmc_gazebo_ros_hrh_gripper tmc_gazebo_ros_bumper
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_dependencies(angles)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(rclcpp)
ament_export_dependencies(trajectory_msgs)

ament_export_libraries(tmc_gazebo_mimic_plugin)
ament_export_libraries(tmc_gazebo_ros_hrh_gripper)
ament_export_libraries(tmc_gazebo_ros_bumper)

ament_package()
