#include "rclcpp/rclcpp.hpp"


using namespace std;

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::shutdown();
  return 0;
}
