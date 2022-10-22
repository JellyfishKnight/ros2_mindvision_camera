cmake_minimum_required(VERSION 3.22.0)
project(ros2_mindvision_camera)

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_executable(
  driver
  src/driver.cpp
  src/ImgPublisher.cpp
  src/MindVision.cpp
  )

target_include_directories(
  driver PUBLIC
  include/
  MindVisionSDK/include
)

target_link_directories(
  driver PUBLIC
  ${PROJECT_SOURCE_DIR}/MindVisionSDK/lib
)
install(
  DIRECTORY ${PROJECT_SOURCE_DIR}/MindVisionSDK/lib
  DESTINATION lib
)

target_link_libraries(
  driver
  MVSDK
)

rclcpp_components_register_node(
  driver
  
)

ament_auto_package()