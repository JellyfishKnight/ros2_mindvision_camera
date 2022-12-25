#ifndef MINDVISION_H
#define MINDVISION_H

//camera driver
#include <CameraApi.h>
//ros2
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
//opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
//self defined
#include "cv_params/Defines.hpp"
#include "task_shared_params/TaskParams.hpp"
#include "serials/SerialAccessories.hpp"

using namespace cv;
using namespace std;
using namespace std::chrono_literals;


namespace Helios {
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
        Mat                     src;
        int                     r_gain_, g_gain_, b_gain_;
        OnSetParametersCallbackHandle::SharedPtr params_callback_handle;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
        sensor_msgs::msg::Image converted_image;
        std::thread capture;

        bool InitCam();

        bool SetCam();
        
        bool StartGrab();

        bool Grab();
            
        bool StopGrab();

        bool ImageConvert();

        void call_back();

        rcl_interfaces::msg::SetParametersResult parametersCallBack(const std::vector<rclcpp::Parameter> & parameters);

        void declareParameters();

    public:
        __attribute__ ((visibility("default")));
        MindVision(const rclcpp::NodeOptions& options) : Node("MindVision", rclcpp::NodeOptions(options).use_intra_process_comms(true)) {
            if (!InitCam()) {
                RCLCPP_INFO(this->get_logger(), "Init Camera Failed!");
            } else {
                //参数声明
                declareParameters();
                if (!SetCam()) {
                    RCLCPP_INFO(this->get_logger(), "Set Camera Failed!");
                } else {
                    if (!StartGrab()) {
                        RCLCPP_INFO(this->get_logger(), "Start Grab Failed!");
                    } else {
                        //创建消息发布器
                        this->pub = rclcpp::create_publisher<sensor_msgs::msg::Image>(this, "produce_sensor_msgs", 1);
                        //创建参数监听，方便相机动态调参
                        this->params_callback_handle = this->add_on_set_parameters_callback(
                            std::bind(&MindVision::parametersCallBack, this, std::placeholders::_1));
                        //
                        capture = std::thread([this]() -> void {
                            while (rclcpp::ok()) {
                                call_back();
                            }
                        }
                        );
                    }
                }
            }
        }
        ~MindVision() = default;
    };

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Helios::MindVision);


#endif