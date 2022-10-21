#include "iostream"
#include "ros2_mindvision_camera/MindVision.h"
#include "opencv2/core.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;

bool MindVision::init() {
    //语言设置
    CameraSdkInit(1);
    //枚举设备,建立设备列表
    CameraEnumerateDevice(&CameraEnumList,&CameraCounts);
    if (CameraCounts == 0) {
        //函数返回值为0则表示枚举失败
        cout << "Camera enumerate devices failed!" << endl;
        return false;
    }
    //检测是否有相机连接
    if (CameraCounts == 0) {
        return false;
    }
    //相机初始化
    if (CameraInit(&CameraEnumList, -1, -1, &hCamera) != CAMERA_STATUS_SUCCESS) {
    //初始化失败    
        cout << "Camera init failed!" << endl;
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
    //发送器
    while (rclcpp::ok()) {
        grab();
        if (src.empty()) {                  //判空
            cout << "Grab failed!" << endl;
            return ;
        }
        imgPublisher->publish(src);
    }
}