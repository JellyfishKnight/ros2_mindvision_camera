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
using namespace rclcpp;


namespace mindvision_camera {
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

    bool InitCam();

    bool SetCam();
    
    bool StartGrab();
        
    bool StopGrab();

    bool ImageConvert();

public:
    MindVision(rclcpp::NodeOptions& options) : Node("MindVision", options) {
        if (!InitCam()) {
            RCLCPP_INFO(this->get_logger(), "Init Camera Failed!");
        } else {
            if (!SetCam()) {
                RCLCPP_INFO(this->get_logger(), "Set Camera Failed!");
            } else {
                if (!StartGrab()) {
                    RCLCPP_INFO(this->get_logger(), "Start Grab Failed!");
                } else {
                    this->pub = rclcpp::create_publisher<sensor_msgs::msg::Image>("sensor_image", 100);
                    Mat src;
                    while (rclcpp::ok()) {
                        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {
                            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
                            /// it takes almost 99.7% of the whole produce time !
                            src = cv::Mat(
                                    cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                                    sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                                    g_pRgbBuffer
                            );
                            ImageConvert(src);
                            CameraReleaseImageBuffer(hCamera, pbyBuffer);
                        else {
                            RCLCPP_INFO(this->get_logger(), "Grab Failed!");
                        }
                    }
                }
            }
        }

    }

    ~MindVision();
};
};

#endif