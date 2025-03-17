cmake_minimum_required(VERSION 3.5)
project(gh360_control)

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
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gh360 REQUIRED)
find_package(gh360_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include/${PROJECT_NAME}
)

add_library(inverse_jacobian
  src/inverse_jacobian.cpp
)
ament_target_dependencies(inverse_jacobian "rclcpp" "orocos_kdl" "kdl_parser")

add_executable(eef_velocity src/eef_velocity.cpp)
target_link_libraries(
  eef_velocity
  inverse_jacobian
)
ament_target_dependencies(eef_velocity rclcpp std_msgs sensor_msgs geometry_msgs)

add_executable(joint_velocity src/joint_velocity.cpp)
ament_target_dependencies(joint_velocity gh360 rclcpp std_msgs sensor_msgs geometry_msgs gh360_interfaces)

add_executable(motor_position src/motor_position.cpp)
ament_target_dependencies(motor_position gh360 rclcpp std_msgs sensor_msgs geometry_msgs gh360_interfaces)

add_executable(teleop_eef_velocity src/teleop_eef_velocity.cpp)
ament_target_dependencies(teleop_eef_velocity gh360 rclcpp geometry_msgs gh360_interfaces)

add_executable(move_home src/move_home.cpp)
ament_target_dependencies(move_home gh360 rclcpp std_srvs std_msgs gh360_interfaces)

add_executable(robot_stop src/robot_stop.cpp)
ament_target_dependencies(robot_stop gh360 rclcpp std_srvs std_msgs gh360_interfaces)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  eef_velocity
  joint_velocity
  motor_position
  move_home
  robot_stop
  teleop_eef_velocity
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)

#Install config files
install(DIRECTORY
config
DESTINATION share/${PROJECT_NAME}
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

ament_export_include_directories(include)

ament_package()
