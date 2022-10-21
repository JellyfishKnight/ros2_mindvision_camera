#ifndef MINDVISION
#define MINDVISION


#include "rclcpp/rclcpp.hpp"
#include "CameraApi.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "cv_bridge/cv_bridge.h"
#include "ImgPublisher.h"

using namespace std;

class MindVision
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
    //发布器
    ImgPublisher* imgPublisher = nullptr;
    //消息图像指针
    sensor_msgs::msg::Image::SharedPtr msg;

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


public:
    /**
     * @brief 构造函数
     * @param frequency 发送频率(每秒多少次)
     * @param t 话题名称
     */
    explicit MindVision(string t, int frequency = 10000) : topic(move(t)) {
        imgPublisher = new ImgPublisher(frequency);
    }
    /**
     * @brief 析构器
     */
    ~MindVision() {
        delete imgPublisher;
    }
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
};

#endif