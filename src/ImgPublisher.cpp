#ifndef IMGPUBLISHER
#define IMGPUBLISHER

#include "ros2_mindvision_camera/ImgPublisher.h"
#include "opencv4/opencv2/core.hpp"

using namespace cv;

void ImgPublisher::publish(Mat input_image) {
    if (img_convert(input_image)) {
        publisher->publish((*image_msg));
        RCLCPP_INFO(this->get_logger(), "Publishing...");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Convert Failed!");
        return;
    }
}

bool ImgPublisher::img_convert(Mat cvImg) {
    this->image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", cvImg).toImageMsg();
    if (image_msg = nullptr) {
        return false;
    } else {
        return true;
    }
}

#endif 