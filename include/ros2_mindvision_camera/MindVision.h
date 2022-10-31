#ifndef MINDVISION_H
#define MINDVISION_H

#include <CameraApi.h>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;


namespace mindvision_camera 
{
class MindVision : public rclcpp::Node {
private:
    int                     hCamera;
    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    int                     fps;
    tSdkCameraDevInfo       tCameraEnumList;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage                *iplImage = NULL;
    double*                 pfLineTime;
    int                     channel = 3;
    unsigned char           * g_pRgbBuffer;     //处理后数据缓存区
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
    sensor_msgs::msg::Image::SharedPtr sensor_image_ptr;
    rclcpp::TimerBase::SharedPtr timer;
    FileStorage*            fileStorage;
    string                  root_of_file = "package://mindvision_camera/config/camera_calibration.xml";
    Mat                     cameraMatrix;
    Mat                     distCoeffs;



    bool InitCam();

    bool SetCam();
    
    bool StartGrab();
        
    bool StopGrab();

    bool ReadData();

    void Undistort(Mat& src);

    bool ImageConvert(Mat& src);

    void call_back();

public:
    __attribute__ ((visibility("default")));
    MindVision(const rclcpp::NodeOptions& options) : Node("MindVision", options) {
        if (!ReadData()) {
            RCLCPP_INFO(this->get_logger(), "Data Read Failed! Image is distorted!");
        }
        if (!InitCam()) {
            RCLCPP_INFO(this->get_logger(), "Init Camera Failed!");
        } else {
            if (!SetCam()) {
                RCLCPP_INFO(this->get_logger(), "Set Camera Failed!");
            } else {
                if (!StartGrab()) {
                    RCLCPP_INFO(this->get_logger(), "Start Grab Failed!");
                } else {
                    this->pub = rclcpp::create_publisher<sensor_msgs::msg::Image>(this, "sensor_image", 1000);
                    this->timer = this->create_wall_timer(1ms, std::bind(&MindVision::call_back, this));                    
                }
            }
        }

    }
    ~MindVision() = default;
};

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(mindvision_camera::MindVision);


#endif