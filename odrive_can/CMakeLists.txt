cmake_minimum_required(VERSION 3.8)
project(odrive_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ros2_socketcan REQUIRED)
find_package(rclcpp REQUIRED)
find_package(odrive_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include/odrive_can)
add_executable(odrive_can_node src/main.cpp src/odrive.cpp)
add_library(odrive_can_component SHARED src/odrive.cpp)
target_include_directories(odrive_can_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


target_include_directories(odrive_can_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )


target_compile_features(odrive_can_node PUBLIC cxx_std_20)  # Require C99 and C++17
target_compile_features(odrive_can_component PUBLIC cxx_std_20)
ament_target_dependencies(
  odrive_can_node
  "can_msgs"
  "ros2_socketcan"
  "rclcpp"
  "std_msgs"
  "odrive_interfaces"
  "diagnostic_msgs"
  )

ament_target_dependencies(
  odrive_can_component
  "can_msgs"
  "ros2_socketcan"
  "rclcpp"
  "std_msgs"
  "odrive_interfaces"
  "diagnostic_msgs"
  )

rclcpp_components_register_nodes(odrive_can_component 
                                 PLUGIN "odrive_node"
                                 )

install(TARGETS odrive_can_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  odrive_can_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
    DIRECTORY
      launch
      config
    DESTINATION
      share/${PROJECT_NAME}/
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
