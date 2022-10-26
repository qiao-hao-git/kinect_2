#ifndef K4A_GRABBER_HPP
#define K4A_GRABBER_HPP

#include <k4a/k4a.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pthread.h>

#include <sys/time.h>
#include <unistd.h>

#include <iostream>

class K4a_Grabber
{

// public:
//     K4a_Grabber(); // 构造函数
//     ~K4a_Grabber(); // 析构函数

/***********************设置相机***********************/
private:
    k4a::device Kinect;
    k4a::calibration calibration;
    k4a::transformation transformation;

public:
    void init(); // 初始化KinectAzureDK相机

    void close(); // 关闭KinectAzureDK相机

    void setCamera(); // 设置KinectAzureDK相机参数

/***********************设置相机***********************/

/***********************获取图片/点云***********************/
private:
    enum getType
    {
        Img, 
        FastPointXYZ, 
        PointXYZ, 
        PointXYZRGB
    };

    // KinectAzureDK相机原始数据捕获
    void KinectAzureDK_Source_Grabber(k4a::image &colorImage_k4a, k4a::image &depthImage_k4a, k4a::image &infraredImage_k4a, uint8_t timeout_ms, getType type); 

    // 生成XY对应表
    void create_xy_table(const k4a::calibration *calibration, k4a::image xy_table); 

    // 生成点云
    void generate_point_cloud(const k4a::image depth_image, const k4a::image xy_table, k4a::image point_cloud, int &point_count); 

public:

    /**
     * @brief 获取Kinect相机图像
     * 
     * @param timeout_ms 超时时间
     * 
     * @return std::vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像； pictures[2] 红外线图像
    */
    std::vector<cv::Mat> getImg(uint8_t timeout_ms = 100); 

    /**
     * @brief 获取快速PointXYZ点云，无序
     * 
     * @param timeout_ms 超时时间
     * 
     * @return 快速PointXYZ点云，无序
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFastPointXYZ(uint8_t timeout_ms = 100);

    /**
     * @brief 获取PointXYZ点云
     * 
     * @param timeout_ms 超时时间
     * 
     * @return PointXYZ点云
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPointXYZ(uint8_t timeout_ms = 100);

    /**
     * @brief 获取PointXYZRGB点云
     * 
     * @param timeout_ms 超时时间
     * 
     * @return PointXYZRGB点云
     */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointXYZRGB(uint8_t timeout_ms = 100);

private:
    bool Thread_Capture_Working = false;
    struct RAW_KINECT_DATA_
    {
        pthread_mutex_t mutex_k4a_image_t = PTHREAD_MUTEX_INITIALIZER; // 互斥锁
        k4a::image colorImage_k4a; // 彩色图片
        k4a::image depthImage_k4a; // 深度突破
        k4a::image infraredImage_k4a; // 红外图片
        bool Used_In_Img = true; // 是否被获取图片使用过
        bool Used_In_FastPointXYZ = true; // 是否被FastPointXYZ使用过
        bool Used_In_PointXYZ = true; // 是否被PointXYZ使用过
        bool Used_In_PointXYZRGB = true; // 是否被PointXYZRGB使用过
    }Raw_Kinect_Data_;

    bool kill_thread_capture = false;
    pthread_t id_thread_capture;
    static void* thread_capture(void *arg); // 捕获线程

public:

    /**
     * @brief 开启相机捕获线程
     * 
     */
    void start_Thread_Capture();

    /**
     * @brief 获取相机捕获线程是否在工作
     * 
     * @return 相机捕获线程是否在工作
     */
    bool is_Thread_Capture_Working();

/***********************获取图片/点云***********************/

private:

    double cal_Diff_Time_ms(timeval first_time, timeval second_time); // 计算时间差

};

#endif