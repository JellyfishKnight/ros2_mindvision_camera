#include "rclcpp/rclcpp.hpp"
#include "iostream"
#include "opencv4/opencv2/core.hpp"
#include "cv_bridge/cv_bridge.h"


using namespace std;
using namespace cv;
using namespace std::chrono_literals;
using std::placeholders::_1;

class ImgPublisher : public rclcpp::Node
{
private:
    string topic;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;

    sensor_msgs::msg::Image::SharedPtr image_msg;

    bool img_convert(Mat cvImg);

public:
    ImgPublisher(int rate) : Node("Image_publisher") {
        publisher = this->create_publisher<sensor_msgs::msg::Image>("cvImg", rate);
        image_msg = nullptr;
    };

    void publish(Mat input_image);
};