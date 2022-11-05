//
// Created by q on 22-11-3.
//

#ifndef KINECT_ROTATE_CLOUD_HPP
#define KINECT_ROTATE_CLOUD_HPP
#include <K4a_Grabber.hpp>
//struct Intrinsic_Parameter
//{
//    double fx;
//    double fy;
//    double cx;
//    double cy;
//    double factor;
//};
//Intrinsic_Parameter intrinsic_parameter[1] = {{503.719635, 503.794678, 327.377747, 325.466949, 1000.0}};

class Kinect_Rotate_Cloud
{
private:

    /*除地相关参数*/
    float th_dist = 0.05;
    float d, th_dist_d;
    /*拟合平面*/
    void estimate_plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_seeds);

    struct Intrinsic_Parameter
    {
        double fx;
        double fy;
        double cx;
        double cy;
        double factor;
    };
    Intrinsic_Parameter intrinsic_parameter[1] = {{503.719635, 503.794678, 327.377747, 325.466949, 1000.0}};

    /*除地最低点*/
    float min_zz;

public:
    /*地面法线*/
    Eigen::Vector3f normal;

    /**
    * 根据地面上的白色与黄色点拟合地面并去除地面
    * @param cloud1 需要去除地面的点云
    * @param Planepara 初始地面参数
    * @param distance_threshold 去除地面的阈值
    * @return 去除地面后的点云
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RemoveGround(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, std::vector<double>& Planepara, float distance_threshold);

    /**
     * 去除地面后，旋转坐标系，从相机坐标转为世界坐标
     * @param cloud2 需要旋转的点云
     * @param normal_ 地面法线
     * @return 旋转坐标系后的点云
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotate_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2, Eigen::Vector3f normal_);

    /**
     * 旋转坐标系，从世界坐标反解回像素坐标
     * @param center 塔块中心的世界坐标x y
     * @param height 塔块中心的世界坐标z
     * @param normal_ 地面法线
     * @return 像素坐标
    */
    cv::Point rotate_back(cv::Point2f center, float height, Eigen::Vector3f normal_);
/*深度图转世界坐标*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr DepthToCloud(const k4a::image &depth, const cv::Mat &rgb);

    struct block{
        cv::Mat image;

        struct point{
           double x, y, z;
           int u, v;
           int r, g, b;

           bool operator < (const point p) const
           {
               if(this->v != p.v)
                   return this->v <= p.v;
               else if(this->u != p.u)
                   return this->u <= p.u;
               else if(this->z != p.z)
                   return this->z >= p.z;
               else
                   return this->y <= p.y;
           }
        };
        std::vector<point> points;
    };
    /**
    * 将点云压成图片
    * @param vis_srccloud 需要压成图片的点云
    * @return block型，包含图片和像素的世界坐标
    */

    block CloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_src_cloud);
};

#endif //KINECT_ROTATE_CLOUD_HPP
