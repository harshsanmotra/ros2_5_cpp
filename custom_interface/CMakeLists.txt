cmake_minimum_required(VERSION 3.8)
project(custom_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

#To generate Action Interfaces, ensure that you have access to the following packages:
#  rosidl_default_generators
#  action_msgs

find_package(action_msgs REQUIRED)

# >>>>>>>>>><<<<<<<<<<<<<<
# When Generating a custom message of the Topics, Services, and Actions <<<<<<<<
find_package(rosidl_default_generators REQUIRED)

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

# This function includes all of the messages of this
# package(of the Topics, Services, and Actions i.e the one in 
# the folder msg or srv or ______ folder) to be compiled.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Age.msg"
  "srv/MyCustomServiceMessage.srv"
  "action/Mover.action"
)

# When Generating a custom message <<<<<<<<
ament_package()
