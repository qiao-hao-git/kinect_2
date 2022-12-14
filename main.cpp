#include "K4a_Grabber.hpp"
#include "rotate_cloud.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pthread.h>

#include <sys/time.h>
#include <unistd.h>

#include <iostream>
#define CV_EVENT_MOUSEMOVE 0             //滑动
#define CV_EVENT_LBUTTONDOWN 1           //左键点击
#define CV_EVENT_RBUTTONDOWN 2           //右键点击
#define CV_EVENT_MBUTTONDOWN 3           //中键点击
#define CV_EVENT_LBUTTONUP 4             //左键放开
#define CV_EVENT_RBUTTONUP 5             //右键放开
#define CV_EVENT_MBUTTONUP 6             //中键放开
#define CV_EVENT_LBUTTONDBLCLK 7         //左键双击
#define CV_EVENT_RBUTTONDBLCLK 8         //右键双击
#define CV_EVENT_MBUTTONDBLCLK 9         //中键双击

K4a_Grabber Kinect;
Kinect_Rotate_Cloud Kinect_Rotate;
cv::Mat colorImage_ocv, depthImage_ocv, infraredImage_ocv;
cv::Point p;

void on_MouseHandle(int event, int x, int y, int flags, void* param) {
    p.x = x;
    p.y = y;
    Kinect_Rotate.DepthToCloud(Kinect.depth_image, colorImage_ocv, p);
}

using namespace std;
using namespace cv;
using namespace pcl;

// ctrl c中断
#include <signal.h>
bool ctrl_c_pressed=false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

// 可视化
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgb_show(new pcl::PointCloud<pcl::PointXYZ>);
pthread_mutex_t mutex_show = PTHREAD_MUTEX_INITIALIZER; // 互斥锁

pthread_t id_thread_show;
void* thread_show(void *arg);



int main()
{
    // Ctrl C 中断
    signal(SIGINT, ctrlc);



    Kinect.init();
    Kinect.setCamera();

    Kinect.start_Thread_Capture();

//    pthread_create(&id_thread_show, NULL, thread_show, NULL);

    while(1)
    {
        if(ctrl_c_pressed == true)
            break;

        timeval tt1,tt2;
        gettimeofday(&tt1, NULL);
        vector<Mat> pictures = Kinect.getImg(100, Kinect.depth_image);
//        Kinect.pictures_correction(pictures);
//        Kinect.color_extract(pictures);

        gettimeofday(&tt2, NULL);
        double diff_ms = 1e3 * (tt2.tv_sec - tt1.tv_sec) + (tt2.tv_usec - tt1.tv_usec) / 1000.0;
//        printf("total diff time1 %lf\n",diff_ms);
        colorImage_ocv = pictures[0], depthImage_ocv = pictures[1], infraredImage_ocv = pictures[2];
//        std::cout << "colorImage_ocv: " << colorImage_ocv.rows << " " << colorImage_ocv.cols << std::endl;
//        std::cout << "depthImage_ocv: " << depthImage_ocv.rows << " " << depthImage_ocv.cols << std::endl;

//        for(int i = 0; i < depthImage_ocv.rows; i++){
//            for(int j = 0; j < depthImage_ocv.cols; j++){
//                if(depthImage_ocv.at<ushort>(i, j) > 0)
//                    std::cout << depthImage_ocv.at<ushort>(i, j) / 64 << std::endl;
//            }
//        }
        cv::namedWindow("RGB");//创建窗口
        cv::setMouseCallback("RGB", on_MouseHandle);
        if(colorImage_ocv.cols * colorImage_ocv.rows != 0) imshow("RGB",colorImage_ocv);
        if(depthImage_ocv.cols * depthImage_ocv.rows != 0) imshow("Depth",depthImage_ocv);
//        if(infraredImage_ocv.cols * infraredImage_ocv.rows != 0) imshow("Ir",infraredImage_ocv);

        waitKey(30);

//        cout << "Thread Capture Is Working : " << Kinect.is_Thread_Capture_Working() << endl;

        // timeval tt1,tt2;
        gettimeofday(&tt1, NULL);
        PointCloud<pcl::PointXYZ>::Ptr source_cloud = Kinect.getFastPointXYZ();
        Eigen::Vector3f normal(0.0, 0.0, 1.0);

//        PointCloud<pcl::PointXYZRGB>::Ptr cloud = Kinect.getPointXYZRGB();

//        PointCloud<PointXYZRGB>::Ptr cloud = Kinect.getPointXYZRGB();
//        Kinect_Rotate.rotate_cloud(cloud, normal);
//        Kinect_Rotate_Cloud::block src = Kinect_Rotate.CloudToImage(cloud);
//        cv::Mat image = src.image;
//        imshow("image", image);
//        waitKey(30);

        gettimeofday(&tt2, NULL);
        diff_ms = 1e3 * (tt2.tv_sec - tt1.tv_sec) + (tt2.tv_usec - tt1.tv_usec) / 1000.0;
//        printf("total diff time2 %lf\n",diff_ms);

//        cout << "Size " << cloud->points.size() << endl;

//        pthread_mutex_lock(&mutex_show);
//        pcl::copyPointCloud(*cloud, *cloud_rgb_show);
//        pthread_mutex_unlock(&mutex_show);
    }

    Kinect.close();

    return 0;
}

void* thread_show(void *arg)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer_PCL"));

    //设置默认的坐标系
	viewer->addCoordinateSystem(1.0); 
	//设置固定的元素。红色是X轴，绿色是Y轴，蓝色是Z
	viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(10, 0, 0), "x");
	viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 5, 0), "y");
	viewer->addLine(pcl::PointXYZ(0, 0, 0), pcl::PointXYZ(0, 0, 2), "z");

    while(1)
    {
        if(ctrl_c_pressed == true)
            break;

        pcl::PointCloud<pcl::PointXYZ>::Ptr res_cloud_rgb_show(new pcl::PointCloud<pcl::PointXYZ>);

        pthread_mutex_lock(&mutex_show);
        pcl::copyPointCloud(*cloud_rgb_show, *res_cloud_rgb_show);
        pthread_mutex_unlock(&mutex_show);

        viewer->addPointCloud(res_cloud_rgb_show, "cloud");
        viewer->spinOnce(3);
        viewer->removePointCloud("cloud");
    }
}

