#include "rclcpp/rclcpp.hpp"
#include "include/ros2_mindvision_camera/ImgPublisher.h"
#include "include/ros2_mindvision_camera/MindVision.h"

using namespace std;

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::shutdown();
  return 0;
}
