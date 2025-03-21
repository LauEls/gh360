cmake_minimum_required(VERSION 3.5)
project(gh360_interfaces)

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
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorStatus.msg"
  "msg/DoorEnv.msg"
  "msg/PortStatus.msg"
  "msg/SetPosition.msg"
  "msg/SetVelocity.msg"
  "msg/SetCurrent.msg"
  "msg/SetMotorPositions.msg"
  "msg/SetMotorVelocities.msg"
  "msg/SetMotorCurrents.msg"
  "msg/JointEncoderState.msg"
  "msg/ArmEncoderStates.msg"
  "msg/SpaceMouse.msg"
  "msg/BoolMultiArray.msg"
  "msg/JointLimits.msg"
  "srv/MotorPositionStep.srv"
  "srv/MotorVelocityStep.srv"
  "srv/TestService.srv"
  "srv/SetRobotLimits.srv"
  "srv/SetJointLimits.srv"
  "action/DoorEnvReset.action"
  DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
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

ament_package()
