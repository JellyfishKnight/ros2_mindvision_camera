cmake_minimum_required(VERSION 3.22.0)
project(mindvision_camera)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(-Wall -Werror)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(
  ${PROJECT_NAME} SHARED
  src/MindVision.cpp
  )

target_include_directories(
  ${PROJECT_NAME} PUBLIC
  include/ros2_mindvision_camera/
  MindVisionSDK/include
)

target_link_directories(
  ${PROJECT_NAME} PUBLIC
  ${PROJECT_SOURCE_DIR}/MindVisionSDK/lib
)
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/MindVisionSDK/lib
  DESTINATION .
)

target_link_libraries(
  ${PROJECT_NAME}
  MVSDK
  ${OpenCV_LIBS}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN mindvision_camera::MindVision
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING) 
  find_package(ament_lint_auto REQUIRED)
  list(
    APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_cpplint
    ament_cmake_uncrustify
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
)