# Configuration values

set(PROJECT_INTERFACE_FILES
  srvs/ConfigurePCA9685.srv
  srvs/ConfigureServo.srv
  srvs/TestServo.srv
  msgs/ServoControl.msg
)

set(PROJECT_SOURCE_FILES
  src/PwmServo.cpp
  src/ServoManager.cpp
  src/ServoManagerNode.cpp
)

set(PROJECT_DEPENDENCIES
  i2c_pwm
  rclcpp
  rclcpp_components
)

set(PROJECT_NODES
  "servo_mgr::ServoManagerNode"
)

set(DEFAULT_CXX_STANDARD  14)
set(CXX_COMPILE_OPTIONS   -Wall -Wextra -Wpedantic -Werror)
set(DEP_I2CPWM_REPO_URL   https://github.com/kerrys-learning-lab/I2C-PWM)
set(DEP_I2CPWM_VER        v1.0.1)

project(servo_mgr)

# Special prerequisites
include(FetchContent)
FetchContent_Declare(i2c_pwm
  URL ${DEP_I2CPWM_REPO_URL}/releases/download/${DEP_I2CPWM_VER}/i2c_pwm.tar.gz
)
FetchContent_GetProperties(i2c_pwm)
FetchContent_MakeAvailable(i2c_pwm)
find_package(i2c_pwm PATHS ${i2c_pwm_SOURCE_DIR} REQUIRED)

# Do not modify below this line ----------------------------------------------

cmake_minimum_required(VERSION 3.8)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD ${DEFAULT_CXX_STANDARD})
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(${CXX_COMPILE_OPTIONS})
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_cmake REQUIRED)

add_library(${PROJECT_NAME} SHARED ${PROJECT_SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

target_include_directories(${PROJECT_NAME}
  PUBLIC  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC  $<INSTALL_INTERFACE:include>
)


if(PROJECT_INTERFACE_FILES)
  rosidl_generate_interfaces(${PROJECT_NAME}_interfaces ${PROJECT_INTERFACE_FILES})

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
endif()

if(PROJECT_NODES)
  rclcpp_components_register_nodes(${PROJECT_NAME} ${PROJECT_NODES})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()
