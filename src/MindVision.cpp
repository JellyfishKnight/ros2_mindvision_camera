#include "iostream"
#include "ros2_mindvision_camera/MindVision.h"
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;
using namespace mindvision_camera;

bool MindVision::init() {
    //语言设置
    CameraSdkInit(1);
    //枚举设备,建立设备列表
    CameraEnumerateDevice(&CameraEnumList,&CameraCounts);
    if (CameraCounts == 0) {
        //函数返回值为0则表示枚举失败
        RCLCPP_INFO(this->get_logger(), "Camera enumerate devices failed!");
        return false;
    }
    //检测是否有相机连接
    if (CameraCounts == 0) {
        return false;
    }
    //相机初始化
    if (CameraInit(&CameraEnumList, -1, -1, &hCamera) != CAMERA_STATUS_SUCCESS) {
    //初始化失败    
        RCLCPP_INFO(this->get_logger(),"Camera init failed!");
        return false;
    } else {
        //获得相机的特性描述结构体
        CameraGetCapability(hCamera, &Capability);
        g_pRgBuffer = (unsigned char*)malloc(Capability.sResolutionRange.iHeightMax * Capability.sResolutionRange.iWidthMax * 3);
        //设置相机参数
        setCameraData();
        //是否为黑白相机
        channel = 3;
        //设置图像处理的输出格式
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
        return true;
    }
}

void MindVision::setCameraData() {
    //停止采集,调整相机参数
    CameraPause(hCamera);
    //释放缓存
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    //设置为手动曝光
    CameraSetAeState(hCamera, false);
    int carName = 1;
    //设置曝光时间
    if (carName == 1) {       //Sentry
        CameraSetExposureTime(hCamera, 6000);
    } else {
        CameraSetExposureTime(hCamera, 8000);
    }
    /* 让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
}

bool MindVision::start() {
    if (CameraPlay(hCamera) == 0) {
        return true;
    } else {
        return false;
    }
}

bool MindVision::stop() {
    if (CameraPause(hCamera) == 0) {
        //释放缓存
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        return true;
    } else {
        return false;
    }
}

bool MindVision::grab() {
    if (CameraGetImageBuffer(hCamera, &FrameInfo, &pbyBuffer, 1000) == 0) {
        CameraImageProcess(hCamera, pbyBuffer, g_pRgBuffer, &FrameInfo);
        src = Mat(
            cvSize(FrameInfo.iWidth, FrameInfo.iHeight),
            FrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
            g_pRgBuffer
        );
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        return true;
    } else {
        return false;
    }
}

void MindVision::publish() {
    //Init publisher
    ImgPublisher = this->create_publisher<sensor_msgs::msg::Image>(topic, frequency);
    //发送器
    while (rclcpp::ok()) {
        grab();
        if (src.empty()) {                  //判空
            RCLCPP_INFO(this->get_logger(), "Grab failed!");
            return ;
        }
        if (img_convert()) {
            this->ImgPublisher->publish((*this->image_msg));
            RCLCPP_INFO(this->get_logger(), "Publishing...");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Convert Failed!");
            return;
        }
    }
}

void MindVision::pre_process() {
    Mat cameraMatrix;
    Mat distCoeff;
    this->filestorage = new FileStorage(this->root, FileStorage::READ);
        if (this->filestorage->isOpened()) {
            (*this->filestorage)["Intrinsic_Matrix_MV"] >> cameraMatrix;
            (*this->filestorage)["Distortion_Coefficients_MV"] >> distCoeff;
        } else {
            RCLCPP_INFO(this->get_logger(), "Data read failed!");
        }
        Mat dist = src.clone();
        undistort(this->src, dist, cameraMatrix, distCoeff);
        src = dist.clone();
}

bool MindVision::img_convert() {
    this->pre_process();
    this->image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", src).toImageMsg();
    if (this->image_msg = nullptr) {
        return false;
    } else {
        return true;
    }
}