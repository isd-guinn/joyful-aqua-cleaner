# this is for CH104M serial imu
# source: Sea Land Technology https://sealandtech.com.tw/resource.html?s=anrot&type=tutorial&p=ros2/readme

cmake_minimum_required(VERSION 3.14.4) # minimum for ROS2 Humble
# cmake_minimum_required(VERSION 3.5)
project(serial_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
# ament_cmake must be included AFTER the call to find_package(rclcpp REQUIRED).
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# for the new message type
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

set(msg_files
"msg/EulerAngle.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

  
  add_executable(talker 
  src/serial_port.cpp src/ch_serial.c)
  ament_target_dependencies(talker 
  rclcpp std_msgs sensor_msgs)
  
  add_executable(listener 
  src/sub_spec.cpp)
  ament_target_dependencies(listener 
  rclcpp std_msgs sensor_msgs)

  target_link_libraries(talker "${cpp_typesupport_target}")

install(TARGETS 
        talker 
        listener  
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
