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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gh360_interfaces REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(kdl_parser REQUIRED)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/motor_dictionaries/motor_dict.cpp
  src/motor_dictionaries/mx_64_dict.cpp
  src/motor_dictionaries/mx_106_dict.cpp
  src/joint_types/motor.cpp
  src/joint_types/encoder.cpp
  src/joint_types/joint.cpp
  src/joint_types/soft_joint.cpp
  src/joint_types/motor_joint.cpp
  src/util/dynamixel.cpp
  src/util/config_parser.cpp
)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${library_name} rclcpp dynamixel_sdk gh360_interfaces)  
# target_link_libraries(${library_name} PUBLIC 
#   rclcpp::rclcpp
#   dynamixel_sdk
#   gh360_interfaces
# )

add_executable(motor_handler src/motor_handler.cpp)
target_link_libraries(motor_handler ${library_name})
ament_target_dependencies(motor_handler std_msgs std_srvs gh360_interfaces)

add_executable(encoder_handler src/encoder_handler.cpp)
target_link_libraries(encoder_handler PRIVATE ${library_name} rclcpp::rclcpp)
# ament_target_dependencies(encoder_handler std_msgs gh360_interfaces)

add_executable(joint_states src/joint_states.cpp)
target_link_libraries(joint_states ${library_name})
ament_target_dependencies(joint_states std_msgs gh360_interfaces sensor_msgs)

add_executable(motor_states src/motor_states.cpp)
target_link_libraries(motor_states ${library_name})
ament_target_dependencies(motor_states std_msgs gh360_interfaces sensor_msgs)

add_executable(eef_pose src/eef_pose.cpp)
target_link_libraries(eef_pose ${library_name})
ament_target_dependencies(eef_pose geometry_msgs tf2 tf2_ros)

add_executable(camera_frame src/camera_frame.cpp)
target_link_libraries(camera_frame ${library_name})
ament_target_dependencies(camera_frame geometry_msgs tf2 tf2_ros)

################################################################################
# Install
################################################################################
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${library_name}
  EXPORT ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  motor_handler
  encoder_handler
  joint_states
  motor_states
  eef_pose
  camera_frame
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

#Install mesh files
install(DIRECTORY
meshes
DESTINATION share/${PROJECT_NAME}
)

#Install urdf files
install(DIRECTORY
urdf
DESTINATION share/${PROJECT_NAME}
)

#Install rviz files
install(DIRECTORY
rviz
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
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${library_name})

ament_export_dependencies(
  rclcpp
)
ament_export_targets(${library_name})

ament_package()