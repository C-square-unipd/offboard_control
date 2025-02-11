cmake_minimum_required(VERSION 3.8)
project(offboard_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Check if ROS_DISTRO is greater or equal to Dashing
# Required since
#   - "create_subscription()" and "create_publisher()" APIs changed
#   - eigen3_cmake_module is only available in these distros
list(APPEND ROS_DISTROS "dashing" "eloquent" "foxy" "galactic" "rolling" "humble")
set(ROS_DISTRO)
if(DEFINED ENV{ROS2_DISTRO})
  set(ROS_DISTRO $ENV{ROS2_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
elseif(DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO $ENV{ROS_DISTRO})
  if(${ROS_DISTRO} IN_LIST ROS_DISTROS)
    add_definitions(-DROS_DEFAULT_API=1)
  endif()
else()
  message(FATAL_ERROR "No ROS Distro set")
endif()

set(USES_DEPRECATED_EXPORT_API "ardent" "bouncy" "crystal" "dashing" "eloquent" "humble")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(offboard_control_quintic src/offboard_control_quintic.cpp)
ament_target_dependencies(offboard_control_quintic
	px4_msgs
	rclcpp
	Eigen3
)

install(TARGETS offboard_control_quintic
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
