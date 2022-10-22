#ifndef MINDVISION
#define MINDVISION


#include "rclcpp/rclcpp.hpp"
#include "CameraApi.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;

namespace mindvision_camera {
class MindVision : public rclcpp::Node
{
private:
    //相机句柄
    int hCamera{};
    //相机数量
    int CameraCounts = 4;
    //相机枚举列表
    tSdkCameraDevInfo CameraEnumList{};
    //设备描述信息
    tSdkCameraCapbility Capability{};
    tSdkFrameHead FrameInfo{};
    BYTE* pbyBuffer{};
    int iDisplayFrames = 10000;
    IplImage* ilpImage = nullptr;
    double* pfLineTime{};
    int channel = 3;
    //处理后数据缓存区
    unsigned char* g_pRgBuffer{};
    //cv图像
    Mat src;
    //话题名称
    string topic;
    //消息图像指针
    sensor_msgs::msg::Image::SharedPtr image_msg;
    // Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr ImgPublisher;
    //Frequency
    int frequency;
    //root of data file
    string root;
    //Pointer of file read
    FileStorage *filestorage;
    /**
     * @brief 设置相机参数
     */
    void setCameraData();
    /**
     * @brief 将相机的数据转化为Mat
     *
     * @return true 转化成功
     * @return false 转化失败
     */
    bool grab();
    /**
     * @brief 初始化
     * 
     * @return true 初始化成功
     * @return false 初始化失败
     */
    bool init();
    /**
     * @brief 启动相机
     * 
     * @return true 启动成功
     * @return false 启动失败
     */
    bool start();
    /**
     * @brief 关闭相机
     * 
     * @return true 关闭成功
     * @return false 关闭失败
     */
    bool stop();
    /**
     * @brief 发布消息
     */
    void publish();
    /**
     * @brief convert cvImg to sensor_msgs::msg::Image
     */
    bool img_convert();
    /**
     * @brief undisort the cv image
    */
    void pre_process();

public:
    /**
     * @brief 构造函数
     * @param frequency 发送频率(每秒多少次)
     * @param t 话题名称
     */
    explicit MindVision(string data_root, string t, int frequency = 10000) : Node("MindVision"), frequency(frequency), root(data_root) {
        if (this->init() && this->start())
        {
            this->publish();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Init failed or start failed!");
        }
        this->stop();
    }
    /**
     * @brief 析构器
     */
    ~MindVision() {
        delete filestorage;

    }
};
}

#endif