cmake_minimum_required(VERSION 3.8)
project(odrive_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Encoder.msg"
  "msg/Target.msg"
  "msg/Iq.msg"
  "msg/BusVoltage.msg"
  "msg/Torque.msg"
  "msg/Temperature.msg"
  "msg/OdriveSensor.msg"
  "srv/ClearErrors.srv"
  "srv/Reboot.srv"
  "srv/SetControllerMode.srv"
  "srv/SetAbsPos.srv"
  "srv/SetPosGain.srv"
  "srv/SetVelGains.srv"
  "srv/SetAxisState.srv"
  DEPENDENCIES std_msgs
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
