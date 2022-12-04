#include "MindVision.hpp"

using namespace std;
using namespace cv;

namespace Helios {

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

    bool MindVision::Grab() {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS) {

        CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);
        //cout << "get camera buffer: " << RMTools::CalWasteTime(st_,getTickFrequency()) << endl;
        /// it takes almost 99.7% of the whole produce time !
        src = cv::Mat(
                cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer
        );
        // Undistort();
        //cout << "convert to cv mat: " << RMTools::CalWasteTime(st_,getTickFrequency()) << endl;
        CameraReleaseImageBuffer(hCamera, pbyBuffer);
        return true;
    } else
        return false;

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

    void MindVision::Undistort() {
        Mat dst;
        Mat map1, map2;
        Mat newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, src.size(), 0, src.size());
        initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), newCameraMatrix, src.size(), CV_16SC2, map1, map2);
        remap(src, dst, map1, map2, cv::INTER_LINEAR);
        src = dst.clone();
    }

    bool MindVision::ImageConvert() {
        if (src.empty()) {
            RCLCPP_INFO(this->get_logger(), "Src image is empty!");
            return false;
        } else {
            sensor_msgs::msg::Image::SharedPtr tempPtr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", src).toImageMsg();
            if (tempPtr == nullptr) {
                RCLCPP_INFO(this->get_logger(), "Convert Failed!");
                return false;
            }
            timeStampMatMsg.frame = (*tempPtr);
            return true;
        }
    }

    void MindVision::call_back(rm_interfaces::msg::ReceiveData::SharedPtr receiveMsgPtr) {
        while (rclcpp::ok()) {
            double st = (double) getTickCount();
            receive_pop_time = rm_tools::CalWasteTime(st, freq);
            double st1 = (double) getTickCount();
            /*ignore frame height and width temperantly*/
            // || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH
            if (!Grab() ) {
                missCount++;
                //LOGW("FRAME GRAB FAILED!\n");
                if (missCount > 5) {
                    StopGrab();
                    quitFlag = true;
                    RCLCPP_ERROR(this->get_logger(), "Exit for grabbing fail.");
                    raise(SIGINT);
                    break;
                }
            } else {
                time_stamp = rm_tools::CalWasteTime(startT,freq)/1000; // save logs which include time_stamp, yaw, pitch
                saveMission = true;
            }
            FRAMEHEIGHT = src.rows;
            FRAMEWIDTH = src.cols;
            ImageConvert();
            timeStampMatMsg.receive_data = (*receiveMsgPtr);
            timeStampMatMsg.stamp = time_stamp;
            pub->publish(timeStampMatMsg);
            produceTime = rm_tools::CalWasteTime(st1, freq);
            // 读取视频空格暂停
            if (carName == VIDEO) {
                if (waitKey(8) == 32) {
                    while (waitKey() != 32) {}
                }
            }
        }
    }
}