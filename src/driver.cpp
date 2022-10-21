#include "rclcpp/rclcpp.hpp"
#include "ros2_mindvision_camera/MindVision.h"

using namespace std;

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);
  MindVision mindvision("MindVisionCamera");
  if (mindvision.init() && mindvision.start()) {
    mindvision.publish();
  } else {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init failed or start failed!");
    return -1;
  }
  mindvision.stop();
  rclcpp::shutdown();
  return 0;
}
