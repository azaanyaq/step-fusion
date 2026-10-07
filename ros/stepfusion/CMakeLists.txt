cmake_minimum_required(VERSION 3.16)
project(stepfusion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(STEPFUSION_CORE_ONLY "Build only the library, CLI and tests (no ROS)" OFF)

# ROS-free core
add_library(stepfusion_core STATIC
  src/csv.cpp
  src/heading.cpp
  src/step_detector.cpp
  src/pdr.cpp
  src/recording.cpp
)
target_include_directories(stepfusion_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(stepfusion_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(replay_csv apps/replay_csv.cpp)
target_link_libraries(replay_csv PRIVATE stepfusion_core)
target_compile_options(replay_csv PRIVATE -Wall -Wextra -Wpedantic)

get_filename_component(STEPFUSION_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(STEPFUSION_TEST_SOURCES
  test/test_heading.cpp
  test/test_step_detector.cpp
  test/test_reference.cpp
)

if(STEPFUSION_CORE_ONLY)
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
  )
  FetchContent_MakeAvailable(googletest)

  enable_testing()
  add_executable(stepfusion_tests ${STEPFUSION_TEST_SOURCES})
  target_link_libraries(stepfusion_tests PRIVATE stepfusion_core GTest::gtest_main)
  target_compile_definitions(stepfusion_tests PRIVATE STEPFUSION_REPO_ROOT="${STEPFUSION_REPO_ROOT}")
  include(GoogleTest)
  gtest_discover_tests(stepfusion_tests)
  return()
endif()

# ROS 2 node
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(stepfusion_node src/stepfusion_node.cpp)
target_link_libraries(stepfusion_node PRIVATE
  stepfusion_core
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
)
target_compile_options(stepfusion_node PRIVATE -Wall -Wextra -Wpedantic)

install(TARGETS stepfusion_node replay_csv DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(stepfusion_tests ${STEPFUSION_TEST_SOURCES})
  target_link_libraries(stepfusion_tests stepfusion_core)
  target_compile_definitions(stepfusion_tests PRIVATE STEPFUSION_REPO_ROOT="${STEPFUSION_REPO_ROOT}")
endif()

ament_package()
