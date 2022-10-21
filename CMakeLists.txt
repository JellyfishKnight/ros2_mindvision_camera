cmake_minimum_required(VERSION 3.8)
project(ros2_mindvision_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)



include_directories(
  include
  /usr/include/opencv4
  /opt/ros/humble/include/action_msgs
  /opt/ros/humble/include/image_geometry 
  /opt/ros/humble/include/nav_msgs rcpputils
  /opt/ros/humble/include/rosidl_runtime_c 
  /opt/ros/humble/include/shape_msgs 
  /opt/ros/humble/include/tlsf_cpp 
  /opt/ros/humble/include/action_tutorials_interfaces 
  /opt/ros/humble/include/image_tools  
  /opt/ros/humble/include/pcl_conversions     
  /opt/ros/humble/include/rcutils                          
  /opt/ros/humble/include/rosidl_runtime_cpp                    
  /opt/ros/humble/include/statistics_msgs    
  /opt/ros/humble/include/tracetools
  /opt/ros/humble/include/actionlib_msgs               
  /opt/ros/humble/include/image_transport          
  /opt/ros/humble/include/pcl_msgs               
  /opt/ros/humble/include/resource_retriever               
  /opt/ros/humble/include/rosidl_typesupport_c                  
  /opt/ros/humble/include/std_msgs           
  /opt/ros/humble/include/trajectory_msgs
  /opt/ros/humble/include/ament_index_cpp              
  /opt/ros/humble/include/interactive_markers     
  /opt/ros/humble/include/pendulum_msgs          
  /opt/ros/humble/include/rmw                              
  /opt/ros/humble/include/rosidl_typesupport_cpp                
  /opt/ros/humble/include/std_srvs           
  /opt/ros/humble/include/turtlesim
  /opt/ros/humble/include/angles                       
  /opt/ros/humble/include/intra_process_demo       
  /opt/ros/humble/include/pluginlib              
  /opt/ros/humble/include/rmw_dds_common                   
  /opt/ros/humble/include/rosidl_typesupport_fastrtps_c         
  /opt/ros/humble/include/stereo_msgs        
  /opt/ros/humble/include/unique_identifier_msgs
  /opt/ros/humble/include/builtin_interfaces           
  /opt/ros/humble/include/joy                      
  /opt/ros/humble/include/qt_gui_cpp             
  /opt/ros/humble/include/rmw_fastrtps_cpp                 
  /opt/ros/humble/include/rosidl_typesupport_fastrtps_cpp       
  /opt/ros/humble/include/teleop_twist_joy   
  /opt/ros/humble/include/urdf
  /opt/ros/humble/include/class_loader                 
  /opt/ros/humble/include/kdl_parser               
  /opt/ros/humble/include/rcl                    
  /opt/ros/humble/include/rmw_fastrtps_shared_cpp          
  /opt/ros/humble/include/rosidl_typesupport_interface          
  /opt/ros/humble/include/tf2                
  /opt/ros/humble/include/urdf_parser_plugin
  /opt/ros/humble/include/composition_interfaces       
  /opt/ros/humble/include/keyboard_handler         
  /opt/ros/humble/include/rcl_action             
  /opt/ros/humble/include/robot_state_publisher            
  /opt/ros/humble/include/rosidl_typesupport_introspection_c    
  /opt/ros/humble/include/tf2_bullet         
  /opt/ros/humble/include/urdfdom
  /opt/ros/humble/include/cv_bridge                    
  /opt/ros/humble/include/laser_geometry           
  /opt/ros/humble/include/rcl_interfaces         
  /opt/ros/humble/include/rosbag2_compression              
  /opt/ros/humble/include/rosidl_typesupport_introspection_cpp  
  /opt/ros/humble/include/tf2_eigen          
  /opt/ros/humble/include/urdfdom_headers
  /opt/ros/humble/include/depthimage_to_laserscan      
  /opt/ros/humble/include/libstatistics_collector  
  /opt/ros/humble/include/rcl_lifecycle          
  /opt/ros/humble/include/rosbag2_compression_zstd         
  /opt/ros/humble/include/rqt_gui_cpp                           
  /opt/ros/humble/include/tf2_eigen_kdl      
  /opt/ros/humble/include/visualization_msgs
  /opt/ros/humble/include/diagnostic_msgs              
  /opt/ros/humble/include/libyaml_vendor           
  /opt/ros/humble/include/rcl_logging_interface  
  /opt/ros/humble/include/rosbag2_cpp                      
  /opt/ros/humble/include/rqt_image_view                        
  /opt/ros/humble/include/tf2_geometry_msgs  
  /opt/ros/humble/include/example_interfaces           
  /opt/ros/humble/include/lifecycle_msgs           
  /opt/ros/humble/include/rcl_yaml_param_parser  
  /opt/ros/humble/include/rosbag2_interfaces               
  /opt/ros/humble/include/rttest                                
  /opt/ros/humble/include/tf2_kdl
  /opt/ros/humble/include/fastcdr                      
  /opt/ros/humble/include/logging_demo             
  /opt/ros/humble/include/rclcpp                 
  /opt/ros/humble/include/rosbag2_storage                  
  /opt/ros/humble/include/rviz_common                           
  /opt/ros/humble/include/tf2_msgs
  /opt/ros/humble/include/fastrtps                     
  /opt/ros/humble/include/map_msgs                 
  /opt/ros/humble/include/rclcpp_action          
  /opt/ros/humble/include/rosbag2_storage_default_plugins  
  /opt/ros/humble/include/rviz_default_plugins                  
  /opt/ros/humble/include/tf2_ros
  /opt/ros/humble/include/foonathan_memory             
  /opt/ros/humble/include/message_filters          
  /opt/ros/humble/include/rclcpp_components      
  /opt/ros/humble/include/rosbag2_transport                
  /opt/ros/humble/include/rviz_rendering                        
  /opt/ros/humble/include/tf2_sensor_msgs
  /opt/ros/humble/include/geometry_msgs                
  /opt/ros/humble/include/moodycamel               
  /opt/ros/humble/include/rclcpp_lifecycle       
  /opt/ros/humble/include/rosgraph_msgs                    
  /opt/ros/humble/include/sensor_msgs                           
  /opt/ros/humble/include/tlsf
  /opt/ros/humble/include/rcpputils
)

add_library(
  ImgPublisher
  src/driver.cpp
)

add_library(
  MindVision
  src/MindVision.cpp
)

add_executable(driver src/driver.cpp)
target_include_directories(driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  driver
  "rclcpp"
  "std_msgs"
  "cv_bridge"
  ${ImgPublisher}
  ${MindVision}
  ${OPENCV_LIBS}
)

install(TARGETS driver
  DESTINATION lib/${PROJECT_NAME})

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
