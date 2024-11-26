cmake_minimum_required(VERSION 3.8)
project(image_conv_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_srvs REQUIRED)




add_executable(image_conversion_server src/conversion_server.cpp)
add_executable(image_conversion_node src/image_conversion_node.cpp)


ament_target_dependencies(image_conversion_server
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  std_srvs
)
ament_target_dependencies(image_conversion_node
  rclcpp
  std_srvs
)


install(TARGETS image_conversion_server image_conversion_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/launch
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
