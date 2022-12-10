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
#include "rm_interfaces/msg/time_stamp_mat.hpp"
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
        rclcpp::Publisher<rm_interfaces::msg::TimeStampMat>::SharedPtr pub;
        rclcpp::Subscription<rm_interfaces::msg::ReceiveData>::SharedPtr subscriber;
        rm_interfaces::msg::TimeStampMat timeStampMatMsg;
        rclcpp::TimerBase::SharedPtr timer;

        bool InitCam();

        bool SetCam();
        
        bool StartGrab();

        bool Grab();
            
        bool StopGrab();

        bool ImageConvert();

        void call_back(rm_interfaces::msg::ReceiveData::SharedPtr receiveMsgPtr);

        void debugCallBack();

        rcl_interfaces::msg::SetParametersResult parametersCallBack(const std::vector<rclcpp::Parameter> & parameters);

        void declareParameters();

    public:
        __attribute__ ((visibility("default")));
        MindVision(const rclcpp::NodeOptions& options) : Node("MindVision", options) {
            if (!InitCam()) {
                RCLCPP_INFO(this->get_logger(), "Init Camera Failed!");
            } else {
                if (!SetCam()) {
                    RCLCPP_INFO(this->get_logger(), "Set Camera Failed!");
                } else {
                    if (!StartGrab()) {
                        RCLCPP_INFO(this->get_logger(), "Start Grab Failed!");
                    } else {
                        //参数声明
                        declareParameters();
                        //创建消息发布器
                        this->pub = rclcpp::create_publisher<rm_interfaces::msg::TimeStampMat>(this, "ProduceTask_node", 1);
                        //创建参数监听，方便相机动态调参
                        this->params_callback_handle = this->add_on_set_parameters_callback(
                            std::bind(&MindVision::parametersCallBack, this, std::placeholders::_1));
                        //debug模式不监听串口
                        if (DEBUG || showArmorBox) {
                            //创建定时器，每一毫秒发送一次图片
                            this->timer = this->create_wall_timer(1ms, std::bind(&MindVision::debugCallBack, this));
                        } else {
                            //创建订阅器，订阅串口节点发送的消息
                            this->subscriber = this->create_subscription<rm_interfaces::msg::ReceiveData>("ReceiveTask_node", 1, std::bind(
                                &MindVision::call_back, this, _1
                            ));
                        }
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