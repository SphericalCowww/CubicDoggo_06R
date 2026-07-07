cmake_minimum_required(VERSION 3.8)
project(my_robot_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interface REQUIRED)
find_package(control_toolbox REQUIRED)

include_directories(include)

###########################################################################
add_executable(cubic_doggo_lifecycle src/cubic_doggo_lifecycle.cpp)
ament_target_dependencies(cubic_doggo_lifecycle rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface)

add_executable(cubic_doggo_lifecycle_imuNode 
    src/cubic_doggo_lifecycle_imuNode.cpp
    src/pid_controller.cpp)
ament_target_dependencies(cubic_doggo_lifecycle_imuNode rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface control_toolbox)

add_executable(cubic_doggo_lifecycle_imuController src/cubic_doggo_lifecycle_imuController.cpp)
ament_target_dependencies(cubic_doggo_lifecycle_imuController rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface control_toolbox)

add_executable(cubic_doggo_lifecycle_imuController_walking src/cubic_doggo_lifecycle_imuController_walking.cpp)
ament_target_dependencies(cubic_doggo_lifecycle_imuController_walking rclcpp moveit_ros_planning_interface moveit_core example_interfaces my_robot_interface control_toolbox)

install(TARGETS cubic_doggo_lifecycle cubic_doggo_lifecycle_imuNode cubic_doggo_lifecycle_imuController cubic_doggo_lifecycle_imuController_walking
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY   include
  DESTINATION include
)

ament_package()





