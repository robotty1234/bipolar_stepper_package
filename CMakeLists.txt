cmake_minimum_required(VERSION 3.5)
project(bipolar_stepper_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(stepper_interfaces REQUIRED)

add_executable(bipolar_stepper src/bipolar_stepper)
ament_target_dependencies(bipolar_stepper rclcpp std_msgs stepper_interfaces)
target_link_libraries(bipolar_stepper pigpiod_if2)
install(TARGETS bipolar_stepper DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg DESTINATION share/${PROJECT_NAME}/)

ament_package()