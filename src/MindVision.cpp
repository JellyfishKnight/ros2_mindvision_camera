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

    rcl_interfaces::msg::SetParametersResult MindVision::parametersCallBack(const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (auto &param : parameters) {
        if (param.get_name() == "exposure_time") {
            int status = CameraSetExposureTime(hCamera, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set exposure time, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "analog_gain") {
            int status = CameraSetAnalogGain(hCamera, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set analog gain, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "rgb_gain.r") {
            r_gain_ = param.as_int();
            int status = CameraSetGain(hCamera, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "rgb_gain.g") {
            g_gain_ = param.as_int();
            int status = CameraSetGain(hCamera, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "rgb_gain.b") {
            b_gain_ = param.as_int();
            int status = CameraSetGain(hCamera, r_gain_, g_gain_, b_gain_);
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set RGB gain, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "saturation") {
            int status = CameraSetSaturation(hCamera, param.as_int());
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set saturation, status = " + std::to_string(status);
            }
        } else if (param.get_name() == "gamma") {
            int gamma = param.as_int();
            int status = CameraSetGamma(hCamera, gamma);
            if (status != CAMERA_STATUS_SUCCESS) {
                result.successful = false;
                result.reason = "Failed to set Gamma, status = " + std::to_string(status);
            }
        } else {
            result.successful = false;
            result.reason = "Unknown parameter: " + param.get_name();
        }
        }
        return result;
    }

    void MindVision::declareParameters() {
        rcl_interfaces::msg::ParameterDescriptor param_description;
        param_description.integer_range.resize(1);
        param_description.integer_range[0].step = 1;
        double exposure_line_time;
        //设置曝光时间
        CameraGetExposureLineTime(hCamera, &exposure_line_time);
        param_description.integer_range[0].from_value = tCapability.sExposeDesc.uiExposeTimeMin * exposure_line_time;
        param_description.integer_range[0].to_value = tCapability.sExposeDesc.uiExposeTimeMax * exposure_line_time;
        double exposure_time = this->declare_parameter("exposure_time", 1250, param_description);
        CameraSetExposureTime(hCamera, exposure_time);
        RCLCPP_INFO(this->get_logger(), "Exposure time = %f", exposure_time);
        //Analog Gain And Set
        param_description.description = "Analog gain";
        param_description.integer_range[0].from_value = tCapability.sExposeDesc.uiAnalogGainMin;
        param_description.integer_range[0].to_value = tCapability.sExposeDesc.uiAnalogGainMax;
        int analog_gain;
        CameraGetAnalogGain(hCamera, &analog_gain);
        analog_gain = this->declare_parameter("analog_gain", analog_gain, param_description);
        CameraSetAnalogGain(hCamera, analog_gain);
        RCLCPP_INFO(this->get_logger(), "Analog gain = %d", analog_gain);
        
        // RGB Gain And Set
        // Get default value
        CameraGetGain(hCamera, &r_gain_, &g_gain_, &b_gain_);
        // R Gain
        param_description.integer_range[0].from_value = tCapability.sRgbGainRange.iRGainMin;
        param_description.integer_range[0].to_value = tCapability.sRgbGainRange.iRGainMax;
        r_gain_ = this->declare_parameter("rgb_gain.r", r_gain_, param_description);
        // G Gain
        param_description.integer_range[0].from_value = tCapability.sRgbGainRange.iGGainMin;
        param_description.integer_range[0].to_value = tCapability.sRgbGainRange.iGGainMax;
        g_gain_ = this->declare_parameter("rgb_gain.g", g_gain_, param_description);
        // B Gain
        param_description.integer_range[0].from_value = tCapability.sRgbGainRange.iBGainMin;
        param_description.integer_range[0].to_value = tCapability.sRgbGainRange.iBGainMax;
        b_gain_ = this->declare_parameter("rgb_gain.b", b_gain_, param_description);
        // Set gain
        CameraSetGain(hCamera, r_gain_, g_gain_, b_gain_);
        RCLCPP_INFO(this->get_logger(), "RGB Gain: R = %d", r_gain_);
        RCLCPP_INFO(this->get_logger(), "RGB Gain: G = %d", g_gain_);
        RCLCPP_INFO(this->get_logger(), "RGB Gain: B = %d", b_gain_);
        
        // Saturation Gain And Set
        param_description.description = "Saturation";
        param_description.integer_range[0].from_value = tCapability.sSaturationRange.iMin;
        param_description.integer_range[0].to_value = tCapability.sSaturationRange.iMax;
        int saturation;
        CameraGetSaturation(hCamera, &saturation);
        saturation = this->declare_parameter("saturation", saturation, param_description);
        CameraSetSaturation(hCamera, saturation);
        RCLCPP_INFO(this->get_logger(), "Saturation = %d", saturation);

        // Gamma Gain And Set
        param_description.integer_range[0].from_value = tCapability.sGammaRange.iMin;
        param_description.integer_range[0].to_value = tCapability.sGammaRange.iMax;
        int gamma;
        CameraGetGamma(hCamera, &gamma);
        gamma = this->declare_parameter("gamma", gamma, param_description);
        CameraSetGamma(hCamera, gamma);
        RCLCPP_INFO(this->get_logger(), "Gamma = %d", gamma);
    }
}