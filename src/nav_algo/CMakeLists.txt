cmake_minimum_required(VERSION 3.14.4) # minimum for ROS2 Humble
# cmake_minimum_required(VERSION 3.5)
project(nav_algo)

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

# find dependencies for both c++ and python
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
# ament_cmake must be included AFTER the call to find_package(rclcpp REQUIRED).
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(std_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# find_package(serial_imu REQUIRED)
find_package(imu_process REQUIRED)
find_package(uart_slave REQUIRED)

###########
## Build ##
###########

include_directories(include)

# # Create Cpp executable
# add_executable(cpp_executable src/cpp_node.cpp)
# ament_target_dependencies(cpp_executable rclcpp)
 
# # Install Cpp executables
# install(TARGETS
#   cpp_executable
#   DESTINATION lib/${PROJECT_NAME}
# )

# install python modules (files in the ./nav_algo directory)
ament_python_install_package(${PROJECT_NAME})

# install Python executables
install(PROGRAMS
        scripts/nav_algo_node.py
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch scripts nav_algo
        DESTINATION share/${PROJECT_NAME})

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

ament_package()