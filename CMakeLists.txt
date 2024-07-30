cmake_minimum_required(VERSION 3.5)
project(simple_actions)

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

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclpy REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(action_tutorials_interfaces REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_smoke_test test/smoke.cpp)

  ament_target_dependencies(${PROJECT_NAME}_smoke_test
    action_tutorials_interfaces
    action_msgs
    rclcpp
    rclcpp_action
  )
  target_include_directories(${PROJECT_NAME}_smoke_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_add_gtest(${PROJECT_NAME}_complex_test test/complex_setup.cpp)

  ament_target_dependencies(${PROJECT_NAME}_complex_test
    action_tutorials_interfaces
    action_msgs
    rclcpp
    rclcpp_action
  )
  target_include_directories(${PROJECT_NAME}_complex_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

endif()

install(DIRECTORY include/
        DESTINATION include
)
install(FILES resource/simple_actions
        DESTINATION share/${PROJECT_NAME}/resource
)
install(DIRECTORY include/
        DESTINATION include
)
ament_python_install_package(${PROJECT_NAME})
ament_export_include_directories(include)

ament_package()
