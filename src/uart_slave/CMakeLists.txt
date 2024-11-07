cmake_minimum_required(VERSION 3.14.4) # minimum for ROS2 Humble
# cmake_minimum_required(VERSION 3.5)
project(uart_slave)

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

find_package(serial_imu REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_export_dependencies(rosidl_default_runtime)

set(msg_files
"msg/FocAngle.msg"
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

###########
## Build ##
###########

include_directories(
  include
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# to be updated !!!!
add_executable(m2s_sender 
               src/uart_m2s.cpp)
ament_target_dependencies(m2s_sender 
                          rclcpp std_msgs serial_imu sensor_msgs)
target_link_libraries(m2s_sender "${cpp_typesupport_target}")

add_executable(s2m_receiver 
               src/uart_s2m.cpp src/decode_serial.cpp)
ament_target_dependencies(s2m_receiver 
                          rclcpp std_msgs)
target_link_libraries(s2m_receiver "${cpp_typesupport_target}")

add_executable(c2m_receiver
               src/uart_c2m.cpp src/decode_serial.cpp)
ament_target_dependencies(c2m_receiver 
                          rclcpp std_msgs)
target_link_libraries(c2m_receiver "${cpp_typesupport_target}")

install(TARGETS 
        m2s_sender
        s2m_receiver
        c2m_receiver  
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()