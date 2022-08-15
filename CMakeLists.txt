cmake_minimum_required(VERSION 3.5)
project(simple_actions)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_python_install_package(${PROJECT_NAME})

ament_package()
