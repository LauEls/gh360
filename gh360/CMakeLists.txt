cmake_minimum_required(VERSION 3.5)
project(gh360)

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
find_package(dynamixel_sdk REQUIRED)
# find_package(dynamixel_sdk_custom_interfaces REQUIRED)
# find_package(dynamixel_workbench_toolbox REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include/${PROJECT_NAME}
)

add_library(gh360
  src/mx_64_dict.cpp
  src/mx_106_dict.cpp
  # src/motor_handler.cpp
  src/soft_joint.cpp
  src/motor_joint.cpp
)
ament_target_dependencies(gh360 "rclcpp" "std_msgs" "dynamixel_sdk")

add_executable(motor_handler src/motor_handler.cpp)
target_link_libraries(
  motor_handler
  gh360
)
ament_target_dependencies(motor_handler "rclcpp" "std_msgs")

# add_executable(port_handler src/port_handler.cpp)
# target_link_libraries(
#   motor_handler
#   gh360
# )
# ament_target_dependencies(motor_handler "rclcpp" "std_msgs")



################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  motor_handler
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

################################################################################
# Test
################################################################################
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

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
# ament_export_dependencies(dynamixel_sdk)
# ament_export_dependencies(rclcpp)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(export_${PROJECT_NAME})
ament_package()