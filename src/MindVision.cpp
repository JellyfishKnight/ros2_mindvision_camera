#include "MindVision.h"
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

namespace mindvision_camera {

bool MindVision::InitCam() {
    CameraSdkInit(1);
    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);
    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0) {
        return -1;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS) {     //初始化失败
        cout << "CamInit failed !" << endl;
        return false;
    } else {
        //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
        CameraGetCapability(hCamera, &tCapability);
        // 数据缓存区
        g_pRgbBuffer = (unsigned char *) malloc(
                tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
        //设置相机的相关参数
        SetCam();
        // 设置为彩色
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
        return true;
    }
}


bool MindVision::SetCam() {
    CameraSetAeState(hCamera, false); //设置为手动曝光

    // 调整RGB三个通道增益
    int r_gain, g_gain, b_gain;
    CameraGetGain(hCamera, &r_gain, &g_gain, &b_gain);
    CameraSetGain(hCamera, r_gain + 40, g_gain + 20, b_gain);

    //MV-SUV134 Camera
    CameraSetExposureTime(hCamera, 1250); //设置曝光时间
    CameraSetAnalogGainX(hCamera, 3.5); //设置增益系数

    /* 让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(hCamera);
    return true;
}

bool MindVision::StartGrab() {
    CameraPlay(hCamera);
    return true;
}

bool MindVision::StopGrab() {
    CameraPause(hCamera);
    CameraReleaseImageBuffer(hCamera, pbyBuffer);
    return false;
}

bool MindVision::ReadData() {
    fileStorage = new FileStorage(root_of_file, FileStorage::READ);
    if (fileStorage->isOpened()) {
        (*fileStorage)["Intrinsic_Matrix_MV"] >> cameraMatrix;
        (*fileStorage)["Distortion_Coefficients_MV"] >> distCoeffs;
        return true;
    } else {
        return false;
    }
}

void MindVision::Undistort(Mat& src) {
    Mat dst = src.clone();
            RCLCPP_WARN(this->get_logger(), "Check");

    undistort(src, dst, cameraMatrix, distCoeffs);

    src = dst.clone();
}

bool MindVision::ImageConvert(Mat& src) {
    if (src.empty()) {
        RCLCPP_INFO(this->get_logger(), "Src image is empty!");
        return false;
    } else {
        sensor_image_ptr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", src).toImageMsg();
        if (sensor_image_ptr == nullptr) {
            RCLCPP_INFO(this->get_logger(), "Convert Failed!");
            return false;
        }
        return true;
    }
}

void MindVision::call_back() {
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
            Undistort(src);
            if (!ImageConvert(src)) {
                RCLCPP_INFO(this->get_logger(), "Image Convert Failed!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Publishing...");
                pub->publish((*sensor_image_ptr));
                CameraReleaseImageBuffer(hCamera, pbyBuffer);
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Grab Failed!");
        }
    }
}


}