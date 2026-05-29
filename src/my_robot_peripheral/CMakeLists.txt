cmake_minimum_required(VERSION 3.8)
project(my_robot_peripheral)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interface REQUIRED)

###########################################################################
add_executable(imu_bno055_node src/imu_bno055_node.cpp)
ament_target_dependencies(imu_bno055_node rclcpp geometry_msgs sensor_msgs example_interfaces my_robot_interface)

add_executable(rasp_pi_peripheral_node src/rasp_pi_peripheral_node.cpp)
ament_target_dependencies(rasp_pi_peripheral_node rclcpp example_interfaces my_robot_interface)
 
install(TARGETS imu_bno055_node rasp_pi_peripheral_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
