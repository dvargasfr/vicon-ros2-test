cmake_minimum_required(VERSION 3.5)

project(parameters_test)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(mocap4ros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(dependencies
    ament_index_cpp
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    tf2_ros
    mocap4ros_msgs
)

include_directories(
  include
  ${rclcpp_lifecycle_INCLUDE_DIRS}
)

add_executable(parameters_test_main
  src/parameters_test.cpp
  src/parameters_test_main.cpp
)
ament_target_dependencies(parameters_test_main ${dependencies})
target_link_libraries(
  parameters_test_main 
  # ${PROJECT_NAME}
)

add_executable(parameters_test_listener
  src/parameters_test_listener.cpp
)
ament_target_dependencies(parameters_test_listener ${dependencies})
target_link_libraries(
  parameters_test_listener 
  # ${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  # ${PROJECT_NAME}
  parameters_test_main
  parameters_test_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include ${rclcpp_lifecycle_INCLUDE_DIRS} ${lifecycle_msgs_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${rclcpp_lifecycle_LIBRARIES})
ament_export_dependencies(${dependencies})
ament_package()
