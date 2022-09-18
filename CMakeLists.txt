cmake_minimum_required(VERSION 3.8)
project(servo_mgr)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(i2c_pwm REQUIRED PATHS /opt/kerrys-learning-lab)
find_package(rosidl_cmake REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_interfaces
  srvs/ConfigurePCA9685.srv
  srvs/ConfigureServo.srv
  srvs/TestServo.srv
  msgs/ServoControl.msg
)

add_library(${PROJECT_NAME} SHARED
  src/PwmServo.cpp
  src/ServoManager.cpp
  src/ServoManagerNode.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "i2c_pwm"
  "rclcpp"
  "rclcpp_components"
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "servo_mgr::ServoManagerNode"
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Get the name of a Typesupport target so it can be used to depend on
# generated messages in the same package that generated them.
#
# https://github.com/ros2/rosidl/blob/humble/rosidl_cmake/cmake/rosidl_get_typesupport_target.cmake
#
# Specifically, we create a named target(servo_mgr_interfaces_target) representing
# the built CPP code from servo_mgr_interfaces
rosidl_get_typesupport_target(${PROJECT_NAME}_interfaces_target
  ${PROJECT_NAME}_interfaces
  rosidl_typesupport_cpp
)

# Specify libraries or flags to use when linking a given target and/or its
# dependents
#
# https://cmake.org/cmake/help/latest/command/target_link_libraries.html
target_link_libraries(${PROJECT_NAME}
  ${${PROJECT_NAME}_interfaces_target}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
