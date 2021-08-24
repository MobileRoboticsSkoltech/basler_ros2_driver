cmake_minimum_required(VERSION 3.5)
project(basler_ros2)

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

#---ROS packages---#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
#---ROS packages---#

#---Pylon---#
find_package(Pylon QUIET)
if (NOT ${Pylon_FOUND})
  include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindPylon.cmake")
endif()
if (NOT ${Pylon_FOUND})
  message(FATAL "Unable to find Basler Pylon SDK.")
endif()
#---Pylon---#

add_executable(basler_node src/BaslerNode.cpp)
target_include_directories(basler_node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${Pylon_INCLUDE_DIRS})
target_link_libraries(basler_node
        ${Pylon_LIBRARIES})
ament_target_dependencies(basler_node
        rclcpp
        image_transport
        camera_calibration_parsers
        std_msgs)

install(TARGETS basler_node
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        config
        DESTINATION share/${PROJECT_NAME}/)

ament_package()
