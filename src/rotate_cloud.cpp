//
// Created by q on 22-11-3.
//
#include <rotate_cloud.hpp>


/**
 * 去除地面后，旋转坐标系，从相机坐标转为世界坐标
 * @param cloud2 需要旋转的点云
 * @param normal_ 地面法线
 * @return 旋转坐标系后的点云
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect_Rotate_Cloud::rotate_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2, Eigen::Vector3f normal){
    Eigen::Vector3f normal_ = normal.normalized();
    Eigen::Vector3f z_axis(0.0, 0.0, 1.0);
    Eigen::Vector3f axis = z_axis.cross(normal);

    float angle = acos(z_axis.dot(normal));
    Eigen::AngleAxisf rotation_vector(-angle, axis);
    Eigen::Matrix3f rotation_matrix = rotation_vector.toRotationMatrix();
//    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
//    transform_matrix.block<3, 3>(0, 0) = rotation_matrix;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZRGB>);

    float min_z = 10;
//    std::vector<float> distances;
    for(size_t i = 0; i < cloud2->points.size(); i++){
        pcl::PointXYZRGB p;
//        Eigen::Vector3f up_ground_point;
//        up_ground_point[0] = cloud2->points[i].x;
//        up_ground_point[1] = cloud2->points[i].y;
//        up_ground_point[2] = cloud2->points[i].z;
        Eigen::Vector3f after_point = rotation_matrix * Eigen::Vector3f(cloud2->points[i].x, cloud2->points[i].y, cloud2->points[i].z);
        p.x = after_point[0];
        p.y = after_point[1];
        p.z = after_point[2];
        p.r = cloud2->points[i].r;
        p.g = cloud2->points[i].g;
        p.b = cloud2->points[i].b;
        cloud3->points.push_back(p);

        float distance = sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
//        distances[i][j] = distance;
//        if(distance > 0)
//            std::cout << "distance: " << distance << std::endl;
        if(p.z < min_z){
            min_z = p.z;
        }
    }


    min_zz = min_z;

    for(size_t i = 0; i < cloud2->points.size(); i++)
        cloud3->points[i].z -= min_z;

    cloud3->width = cloud3->points.size();
    cloud3->height = 1;
    cloud3->is_dense = false;
    return cloud3;
}

/*计算均值和协方差矩阵*/
template <typename PointT, typename Scalar>
inline unsigned int computeMeanAndCovarianceMatrix(const pcl::PointCloud<PointT> &cloud, Eigen::Matrix<Scalar, 3, 3> &covariance_matrix, Eigen::Matrix<Scalar, 4, 1> &centroid){
    Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor> xyz_centroid = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero();
    size_t point_cloud;
    if(cloud.is_dense){
        point_cloud = cloud.points.size();
        for(size_t i = 0; i < point_cloud; i++){
            xyz_centroid[0] += cloud[i].x * cloud[i].x;
            xyz_centroid[1] += cloud[i].x * cloud[i].y;
            xyz_centroid[2] += cloud[i].x * cloud[i].z;
            xyz_centroid[3] += cloud[i].y * cloud[i].y;
            xyz_centroid[4] += cloud[i].y * cloud[i].z;
            xyz_centroid[5] += cloud[i].z * cloud[i].z;
            xyz_centroid[6] += cloud[i].x;
            xyz_centroid[7] += cloud[i].y;
            xyz_centroid[8] += cloud[i].z;
        }
    }
    else{
        point_cloud = 0;
        for(size_t i = 0; i < cloud.size(); i++){
            xyz_centroid[0] += cloud[i].x * cloud[i].x;
            xyz_centroid[1] += cloud[i].x * cloud[i].y;
            xyz_centroid[2] += cloud[i].x * cloud[i].z;
            xyz_centroid[3] += cloud[i].y * cloud[i].y;
            xyz_centroid[4] += cloud[i].y * cloud[i].z;
            xyz_centroid[5] += cloud[i].z * cloud[i].z;
            xyz_centroid[6] += cloud[i].x;
            xyz_centroid[7] += cloud[i].y;
            xyz_centroid[8] += cloud[i].z;
            point_cloud++;
        }
    }
    xyz_centroid /= static_cast<Scalar>(point_cloud);
    if(point_cloud != 0){
        centroid[0] = xyz_centroid[6];
        centroid[1] = xyz_centroid[7];
        centroid[2] = xyz_centroid[8];
        centroid[3] = 1;
        covariance_matrix.coeffRef(0) = xyz_centroid[0] - xyz_centroid[6] * xyz_centroid[6];
        covariance_matrix.coeffRef(1) = xyz_centroid[1] - xyz_centroid[6] * xyz_centroid[7];
        covariance_matrix.coeffRef(2) = xyz_centroid[2] - xyz_centroid[6] * xyz_centroid[8];
        covariance_matrix.coeffRef(4) = xyz_centroid[3] - xyz_centroid[7] * xyz_centroid[7];
        covariance_matrix.coeffRef(5) = xyz_centroid[4] - xyz_centroid[7] * xyz_centroid[8];
        covariance_matrix.coeffRef(8) = xyz_centroid[5] - xyz_centroid[8] * xyz_centroid[8];
        covariance_matrix.coeffRef(3) = covariance_matrix.coeff(1);
        covariance_matrix.coeffRef(6) = covariance_matrix.coeff(2);
        covariance_matrix.coeffRef(7) = covariance_matrix.coeff(5);
    }
    return (static_cast<unsigned int>(point_cloud));
}

/*计算点到平面的平均距离*/
float calculateDistance(std::vector<double> &Planepara, pcl::PointXYZRGB Pos){
    float dx = sqrt(pow(Planepara[0], 2) + pow(Planepara[1], 2) + pow(Planepara[2], 2));
    float dy = Planepara[0] * Pos.x + Planepara[1] * Pos.y + Planepara[2] * Pos.z + Planepara[3];
    float distance = fabs(dy) / dx;
    return distance;
}

/*拟合平面*/
void Kinect_Rotate_Cloud::estimate_plane(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_seeds) {
    Eigen::Matrix3f cov;
    Eigen::Vector4f centroid;
    /*计算均值协和方差矩阵*/
    computeMeanAndCovarianceMatrix(*ground_seeds, cov, centroid);
    /*奇异值分解*/
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);

    /*取前三维作为主要方向：U矩阵m*m => m*r r=3  m是g_ground_pc点云的点数*/
    normal = (svd.matrixU().col(2));
    /*要XYZIR前三维,XYZ*/
    Eigen::Vector3f seeds_mean = centroid.head<3>();

    d = -(normal.transpose() * seeds_mean)(0, 0);
    if(d < 0) {
        d = -d;
        normal = -normal;
    }
    /*点云中的这个点到这个平面的正交投影距离小于阈值 Th_dist, 则认为该点属于地面，否则属于非地面*/
    th_dist = th_dist - d;
}
/**
* 根据地面上的白色与黄色点拟合地面并去除地面
* @param cloud1 需要去除地面的点云
* @param Planepara 初始地面参数
* @param distance_threshold 去除地面的阈值
* @return 去除地面后的点云
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect_Rotate_Cloud::RemoveGround(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, std::vector<double> &Planepara, float distance_threshold)
{
    //黄色地面作为初始地面
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground_seeds(new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointCloud<pcl::PointXYZRGB> src_cloud = *cloud1;

    // pointcloud to matrix  点云数据转换为矩阵存储 n*3维度表示
    Eigen::MatrixXf points(src_cloud.size(), 3);
    int j = 0;

    //先选初始黄色点
    for (size_t i = 0; i < src_cloud.size(); ++i)
    {
        pcl::PointXYZRGB p;
        p.x = src_cloud[i].x, p.y = src_cloud[i].y, p.z = src_cloud[i].z;
        p.r = src_cloud[i].r, p.g = src_cloud[i].g, p.b = src_cloud[i].b;

        points.row(j++) << p.x, p.y, p.z;

        float distance = calculateDistance(Planepara, p);

        // 不在预设平面附近不要
        if (distance <= distance_threshold)
            ground_seeds->points.push_back(p);
    }

    ground_seeds->width = ground_seeds->size();
    ground_seeds->height = 1;
    ground_seeds->is_dense = false;

    //拟合平面
    estimate_plane(ground_seeds);

    // th_dist_d_ = -1.127;
    // normal_ = Eigen::Vector3f(-0.020, -0.515, -0.857);

    // ground plane model  所有点与地平面法线的点乘，得到与地平面的距离
    Eigen::VectorXf result = points * normal;

    // std::cout << "地面法向量" << normal_ << " dist " << th_dist_d_ << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr up_ground(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int i = 0; i < result.rows(); i++)
    {
        if (result[i] < th_dist_d)
            continue;
        pcl::PointXYZRGB p;
        p.x = src_cloud[i].x, p.y = src_cloud[i].y, p.z = src_cloud[i].z;
        p.r = src_cloud[i].r, p.g = src_cloud[i].g, p.b = src_cloud[i].b;
        up_ground->points.push_back(p);
    }
    up_ground->width = up_ground->size();
    up_ground->height = 1;
    up_ground->is_dense = false;

    return up_ground;
    // return ground_seeds;
}

/**
* 将点云压成图片
* @param vis_srccloud 需要压成图片的点云
* @return block型，包含图片和像素的世界坐标
*/
Kinect_Rotate_Cloud::block Kinect_Rotate_Cloud::CloudToImage(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr vis_src_cloud)
{
    block ansToreturn;

    ansToreturn.points.clear();

    pcl::PointCloud<pcl::PointXYZRGB> src_cloud = *vis_src_cloud;

    std::vector<block::point> points;
    int maxu = -100000, maxv = -100000;

    // 将相机坐标转为图像坐标
    for(size_t i = 0; i < src_cloud.size(); i++)
    {
        pcl::PointXYZRGB p = src_cloud[i];

        if(p.z > 0.5) continue;

        // 记录二维图像坐标
        block::point pp;
        pp.x = p.x, pp.y = p.y, pp.z = src_cloud[i].z;
        pp.u = -200 * p.x, pp.v = 200 * p.y;
        pp.r = p.r, pp.g = p.g, pp.b = p.b;

        points.push_back(pp);

        if(pp.u > maxu) maxu = pp.u;
        if(pp.v > maxv) maxv = pp.v;
    }

    if(points.size() == 0) return ansToreturn;
    // std::cout << "NUMOF" << NumOfValuePoints << std::endl;
    // 将二维图像坐标转为像素坐标
    int row = 0, col = 0;
    for(size_t i = 0; i < points.size(); i++)
    {
        points[i].u = -(points[i].u - maxu);
        points[i].v = -(points[i].v - maxv);

        if(points[i].u > col) col = points[i].u;
        if(points[i].v > row) row = points[i].v;
    }

    if(row == 0 && col == 0) return ansToreturn;
    // 把像素排序
    std::sort(points.begin(), points.end());

    // 生成图片
    cv::Mat image(540,720,CV_8UC3,cv::Scalar(0,0,0));
    for(size_t i=0; i<points.size(); i++)
    {

        int u = points[i].u, v = points[i].v;

        if(u >= 720 || v >= 540) continue;
        if(u < 0 || u >= col || v < 0 || v >= row) continue;
        if(points[i].g == 0 && points[i].r == 0 && points[i].b == 0) continue;

        if(image.at<cv::Vec3b>(v,u)[0] == 0 && image.at<cv::Vec3b>(v,u)[1] == 0 && image.at<cv::Vec3b>(v,u)[2] == 0)
        {
            // if(points[i].g == 255 && points[i].r == 0 && points[i].b == 0) continue;

            // std::cout<<"cloudtoimage_if_start"<<std::endl;
            image.at<cv::Vec3b>(v,u)[0] = points[i].b;
            image.at<cv::Vec3b>(v,u)[1] = points[i].g;
            image.at<cv::Vec3b>(v,u)[2] = points[i].r;
            ansToreturn.points.push_back(points[i]);
            // std::cout<<"cloudtoimage_if_end"<<std::endl;
        }
    }
    ansToreturn.image = image.clone();
    image.release();
    return ansToreturn;
}

/*深度图转世界坐标*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Kinect_Rotate_Cloud::DepthToCloud(const k4a::image &depth, const cv::Mat &rgb,cv::Point p)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//    std::vector<std::vector<float> > distances;
    cv::Mat depth_image = cv::Mat(depth.get_height_pixels(), depth.get_width_pixels(), CV_16U, (void *)depth.get_buffer());
    int width = depth_image.cols;
    int height = depth_image.rows;
//    std::cout << width << " " << height << std::endl;
//    for(int v = 0; v < depth.get_height_pixels(); v++)
//    {
//        for(int u = 0; u < depth.get_width_pixels(); u++)
//        {
//            pcl::PointXYZRGB p;
            /*获取每个像素点的深度*/
            ushort d = depth_image.at<ushort>(p.y, p.x);

//            std::cout << "d = " << d << std::endl;

        /*将深度图转为世界坐标*/
//            p.z = d ;
//         std::cout << "p.z = " << p.z << std::endl;
//            p.x = (u - intrinsic_parameter[0].cx) * p.z / intrinsic_parameter[0].fx;
//            p.y = (v - intrinsic_parameter[0].cy) * p.z / intrinsic_parameter[0].fy;
//            std::cout << "p.x = " << p.x << std::endl;
//            std::cout << "p.y = " << p.y << std::endl;

//            p.b = rgb.ptr<uchar>(v)[u * 3];
//            p.g = rgb.ptr<uchar>(v)[u * 3 + 1];
//            p.r = rgb.ptr<uchar>(v)[u * 3 + 2];
            double z = (double)d / 1000.0;
            if(z > 0) std::cout << "z = " << z << "m" << std::endl;
            double x = (p.x - intrinsic_parameter[0].cx) * z / intrinsic_parameter[0].fx;
            if(x != 0) std::cout << "x = " << x << "m" << std::endl;
            double y = (p.y - intrinsic_parameter[0].cy) * z / intrinsic_parameter[0].fy;
            if(y != 0)
                std::cout << "y = " << y << "m" << std::endl;
            float distance = sqrt(pow(x,2) + pow(y,2) + pow(z, 2));
            if (distance > 0) {
                std::cout << "distance: " << distance << "m" << std::endl;
            }
//            distances[u][v] = distance;//计算点云中每个点到相机的距离
//            std::cout << distances[u][v] << std::endl;
//            cloud->points.push_back(p);
//        }
//    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = false;

    return cloud;
}
