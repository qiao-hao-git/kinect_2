#include "K4a_Grabber.hpp"

static K4a_Grabber*
K4a_Grabber_Ptr_ = nullptr;

/**
 *  初始化KinectAzureDK相机
*/
void K4a_Grabber::init()
{
    // 查询设备数量
    uint32_t devices_count = k4a::device::get_installed_count();
    if (devices_count == 0)
    {
        printf("No K4a Devices Attached!\n");
        return ;
    }
    else
    {
        printf("Found %u Kinect Devices!\n", devices_count);
    }

    // 设置参数
    k4a_device_configuration_t init_params = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    // 设置color图的分辨率位 720P
    init_params.color_resolution = K4A_COLOR_RESOLUTION_720P;
    // 设置Kinect的相机帧率为30FPS
    init_params.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // 设置Kinect的深度模式为Near FOV unbinned（这一代 Kinect 支持多种深度模式，官方文档推荐使用 K4A_DEPTH_MODE_NFOV_UNBINNED 和 K4A_DEPTH_MODE_WFOV_2X2BINNED 这两种深度模式）
//	init_params.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    init_params.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;

	// 设置Kinect的颜色属性为BGRA32
	init_params.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    //为了同时获取depth和color图，保持这两种图像是同步的
	init_params.synchronized_images_only = true;

    // 打开相机
    int device_id_ = 0;
    Kinect = k4a::device::open(device_id_);
    Kinect.start_cameras(&init_params);

    // 查询设备SN码
    std::string serial_number =  Kinect.get_serialnum();
    std::cout << "Open Kinect Device Serial Number: " << serial_number << std::endl;

    if(!Kinect)
    {
        printf("Kinect Open Error!\n");
        return ;
    }

    // 获取相机参数
    calibration = Kinect.get_calibration(init_params.depth_mode, init_params.color_resolution);
    transformation = k4a::transformation(calibration);

    // 初始化K4a_Grabber_Ptr_
    if(K4a_Grabber_Ptr_ == nullptr)
    {
        K4a_Grabber_Ptr_ = this;
    }
}

/**
 * 关闭KinectAzureDK相机
 */
void K4a_Grabber::close()
{
    if(Kinect)
    {
        if(Thread_Capture_Working)
        {
            kill_thread_capture = true;
            usleep(1e5);
        }
            
        transformation.destroy();
        Kinect.stop_cameras();
        Kinect.close();
        printf("\nKinect Close Successed!\n");
    }
}

/**
 * 设置KinectAzureDK相机参数
 */
void K4a_Grabber::setCamera()
{
    Kinect.set_color_control(K4A_COLOR_CONTROL_EXPOSURE_TIME_ABSOLUTE, K4A_COLOR_CONTROL_MODE_AUTO, 0);     // 曝光时间
    // Kinect.set_color_control(K4A_COLOR_CONTROL_BRIGHTNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 0);               // 亮度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_CONTRAST, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                 // 对比度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_SATURATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);               // 饱和度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_SHARPNESS, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                // 锐度
    // Kinect.set_color_control(K4A_COLOR_CONTROL_WHITEBALANCE, K4A_COLOR_CONTROL_MODE_MANUAL, 0);             // 白平衡
    // Kinect.set_color_control(K4A_COLOR_CONTROL_BACKLIGHT_COMPENSATION, K4A_COLOR_CONTROL_MODE_MANUAL, 0);   // 背光补偿  
    // Kinect.set_color_control(K4A_COLOR_CONTROL_GAIN, K4A_COLOR_CONTROL_MODE_MANUAL, 0);                     // GAIN
    // Kinect.set_color_control(K4A_COLOR_CONTROL_POWERLINE_FREQUENCY, K4A_COLOR_CONTROL_MODE_MANUAL, 0);      // 电力线频率   
}

/**
 * KinectAzureDK相机原始数据捕获
 */
void K4a_Grabber::KinectAzureDK_Source_Grabber(k4a::image &colorImage_k4a, k4a::image &depthImage_k4a, k4a::image &infraredImage_k4a, uint8_t timeout_ms, getType type)
{
    // 清空变量
    colorImage_k4a = depthImage_k4a = infraredImage_k4a = nullptr;

    // 检测捕获线程是否在工作
    if(Thread_Capture_Working)
    {
        timeval capture_start_time;
        gettimeofday(&capture_start_time, NULL);
        while(1)
        {
            bool *Used = nullptr;
            switch (type)
            {
            case Img:
                Used = &(Raw_Kinect_Data_.Used_In_Img);
                break;
            
            case FastPointXYZ:
                Used = &(Raw_Kinect_Data_.Used_In_FastPointXYZ);
                break;

            case PointXYZ:
                Used = &(Raw_Kinect_Data_.Used_In_PointXYZ);
                break;

            case PointXYZRGB:
                Used = &(Raw_Kinect_Data_.Used_In_PointXYZRGB);
                break;
            
            default:
                break;
            }

            // 检测到新图片
            if(*Used == false)
            {
                pthread_mutex_lock(&Raw_Kinect_Data_.mutex_k4a_image_t);
                colorImage_k4a = k4a::image(Raw_Kinect_Data_.colorImage_k4a);
                depthImage_k4a = k4a::image(Raw_Kinect_Data_.depthImage_k4a);
                infraredImage_k4a = k4a::image(Raw_Kinect_Data_.infraredImage_k4a);
                *Used = true;
                pthread_mutex_unlock(&Raw_Kinect_Data_.mutex_k4a_image_t);
                return ;
            }

            // 超时检测
            timeval capture_now_time;
            gettimeofday(&capture_now_time, NULL);
            if(cal_Diff_Time_ms(capture_start_time, capture_now_time) > timeout_ms)
            {
                printf("Grabber Time Out!\n");
                return ;
            }
        }
    }
    else 
    {
        // 如果捕获线程没在工作，则自己调用捕获
        k4a::capture capture;

        if(!Kinect.get_capture(&capture, std::chrono::milliseconds(timeout_ms)))
        {
            printf("KinectAzureDK Grabber Failed!\n");
            return ;
        }


        colorImage_k4a = capture.get_color_image();
        if(colorImage_k4a == nullptr)
            printf("Failed To Get Color Image From Kinect!\n");

        depthImage_k4a = capture.get_depth_image();
        if(depthImage_k4a == nullptr)
            printf("Failed To Get Depth Image From Kinect!\n");

        infraredImage_k4a = capture.get_ir_image();
        if(infraredImage_k4a == nullptr)
            printf("Failed To Get IR Image From Kinect!\n");

    }
}

/**
 * @brief 获取Kinect相机图像
 * 
 * @param timeout_ms 超时时间
 * 
 * @return std::vector<cv::Mat> pictures 表示获取到的图像； pictures[0] 彩色图像； pictures[1] 深度图像； pictures[2] 红外线图像
*/
std::vector<cv::Mat> K4a_Grabber::getImg(uint8_t timeout_ms, k4a::image &depthImage_k4a)
{
    cv::Mat colorImage_ocv, depthImage_ocv, infraredImage_ocv;

    // 获取相机原始数据
//    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    k4a::image colorImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, Img);

    /*深度图和彩色图配准*/
    k4a::transformation transformation = k4a::transformation(calibration);
	
    // 数据格式转换
    if(colorImage_k4a != nullptr)
    {
        colorImage_ocv = cv::Mat(colorImage_k4a.get_height_pixels(), colorImage_k4a.get_width_pixels(), CV_8UC4, (void *)colorImage_k4a.get_buffer());
        cvtColor(colorImage_ocv, colorImage_ocv, cv::COLOR_BGRA2BGR); // 从四通道转到三通道
    }

    if(depthImage_k4a != nullptr)
    {
	k4a::image transformed_depth_image = transformation.depth_image_to_color_camera(depthImage_k4a);
        depthImage_ocv = cv::Mat(depthImage_k4a.get_height_pixels(), depthImage_k4a.get_width_pixels(), CV_16U, (void *)depthImage_k4a.get_buffer());
        depthImage_ocv.convertTo(depthImage_ocv, CV_8U, 1);

    }

    if(infraredImage_k4a != nullptr)
    {
        infraredImage_ocv = cv::Mat(infraredImage_k4a.get_height_pixels(), infraredImage_k4a.get_width_pixels(), CV_16U, (void *)infraredImage_k4a.get_buffer());
        infraredImage_ocv.convertTo(infraredImage_ocv, CV_8U, 1);
    }

    std::vector<cv::Mat> pictures;
    pictures.push_back(colorImage_ocv);
    pictures.push_back(depthImage_ocv);
    pictures.push_back(infraredImage_ocv);

    return pictures;
}

/*颜色提取*/
void K4a_Grabber::color_extract(std::vector<cv::Mat> &pictures)
{
    cv::Mat hsv;
    cv::cvtColor(pictures[0], hsv, cv::COLOR_BGR2HSV);
    cv::Mat mask;
    cv::Scalar lower_withe(0, 0, 50);
    cv::Scalar upper_withe(180,30, 255);
    cv::inRange(hsv, lower_withe, upper_withe, mask);
    cv::Mat dst;
    cv::bitwise_and(pictures[0], pictures[0], dst, mask);
    pictures[0] = dst;
}


/**
 * @brief 获取快速PointXYZ点云，无序
 * 
 * @param timeout_ms 超时时间
 * 
 * @return 快速PointXYZ点云，无序
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr K4a_Grabber::getFastPointXYZ(uint8_t timeout_ms)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 如果第一次进入该函数，创建xy对应表格
    static bool FIRST_ENTER = true;
    static k4a::image xy_table = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                    calibration.depth_camera_calibration.resolution_width,
                                                    calibration.depth_camera_calibration.resolution_height,
                                                    calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t));
    if(FIRST_ENTER)
    {
        FIRST_ENTER = false;
        create_xy_table(&calibration, xy_table);
    }

    // 获取相机原始数据
    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, FastPointXYZ);

    if(colorImage_k4a == nullptr || depthImage_k4a == nullptr)
    {
        printf("Grabber PointXYZ Failed!\n");
        return cloud;
    }

    // 数据格式转换
    k4a::image point_cloud = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                calibration.depth_camera_calibration.resolution_width,
                                                calibration.depth_camera_calibration.resolution_height,
                                                calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float3_t));
    int point_count = 0;

    generate_point_cloud(depthImage_k4a, xy_table, point_cloud, point_count); // 生成点云

    int width = point_cloud.get_width_pixels();
    int height = point_cloud.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(height * width);

    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)point_cloud.get_buffer();

    for(int i = 0; i < width * height; i++)
    {
        if (isnan(point_cloud_data[i].xyz.x) || isnan(point_cloud_data[i].xyz.y) || isnan(point_cloud_data[i].xyz.z))
        {
            continue;
        }

        pcl::PointXYZ p;
        p.x = point_cloud_data[i].xyz.x / 1000.0f;
        p.y = point_cloud_data[i].xyz.y / 1000.0f;
        p.z = point_cloud_data[i].xyz.z / 1000.0f;

        if(p.z == 0) continue;

        cloud->points[i] = p;
    }

    return cloud;
}


/**
 * @brief 获取PointXYZ点云
 * 
 * @param timeout_ms 超时时间
 * 
 * @return PointXYZ点云
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr K4a_Grabber::getPointXYZ(uint8_t timeout_ms)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 获取相机原始数据
    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, PointXYZ);

    if(colorImage_k4a == nullptr || depthImage_k4a == nullptr)
    {
        printf("Grabber PointXYZ Failed!\n");
        return cloud;
    }

    // 数据格式转换
    int width = colorImage_k4a.get_width_pixels();;
    int height = colorImage_k4a.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(height * width);

    k4a::image transformed_depth_image = NULL;
	transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t));
	k4a::image point_cloud_image = NULL;
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * 3 * (int)sizeof(int16_t));
	transformation.depth_image_to_color_camera(depthImage_k4a, &transformed_depth_image);
	transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();

    for(int i = 0; i < width * height; i++)
    {
        pcl::PointXYZ p;
        p.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        p.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        p.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if(p.z == 0) continue;

        cloud->points[i] = p;
    }

    return cloud;
}

/**
 * @brief 获取PointXYZRGB点云
 * 
 * @param timeout_ms 超时时间
 * 
 * @return PointXYZRGB点云
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr K4a_Grabber::getPointXYZRGB(uint8_t timeout_ms)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    // 获取相机原始数据
    k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;
    KinectAzureDK_Source_Grabber(colorImage_k4a, depthImage_k4a, infraredImage_k4a, timeout_ms, PointXYZRGB);

    if(colorImage_k4a == nullptr || depthImage_k4a == nullptr)
    {
        printf("Grabber PointXYZ Failed!\n");
        return cloud;
    }

    // 数据格式转换

    int width = colorImage_k4a.get_width_pixels();
    int height = colorImage_k4a.get_height_pixels();

    cloud->width = width;
    cloud->height = height;
    cloud->is_dense = false;
    cloud->points.resize(height * width);

    k4a::image transformed_depth_image = NULL;
	transformed_depth_image = k4a::image::create(K4A_IMAGE_FORMAT_DEPTH16, width, height, width * (int)sizeof(uint16_t));
	k4a::image point_cloud_image = NULL;
	point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM, width, height, width * 3 * (int)sizeof(int16_t));
	transformation.depth_image_to_color_camera(depthImage_k4a, &transformed_depth_image);
	transformation.depth_image_to_point_cloud(transformed_depth_image, K4A_CALIBRATION_TYPE_COLOR, &point_cloud_image);

    int16_t *point_cloud_image_data = (int16_t *)(void *)point_cloud_image.get_buffer();
    uint8_t *color_image_data = colorImage_k4a.get_buffer();

    for(int i = 0; i < width * height; i++)
    {
        pcl::PointXYZRGB p;
        p.x = point_cloud_image_data[3 * i + 0] / 1000.0f;
        p.y = point_cloud_image_data[3 * i + 1] / 1000.0f;
        p.z = point_cloud_image_data[3 * i + 2] / 1000.0f;

        if(p.z == 0) continue;

        p.b = color_image_data[4 * i + 0];
		p.g = color_image_data[4 * i + 1];
		p.r = color_image_data[4 * i + 2];
        uint8_t alpha = color_image_data[4 * i + 3];
		if (p.b == 0 && p.g == 0 && p.r == 0 && alpha == 0) continue;

        cloud->points[i] = p;
    }

    return cloud;
}

/*聚类*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr K4a_Grabber::dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, std::vector<Classes> &now_classes)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visual_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::KdTreeFLANN<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud_in);
    std::vector<bool> cloud_processed(cloud_in->size(), false);
    std::vector<std::vector<int>> clusters_index;
    double r=0.5;
    int size=2;

    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        if (cloud_processed[i])
        {
            continue;
        }

        std::vector<int>seed_queue;
        //检查近邻数是否大于给定的size（判断是否是核心对象）
        std::vector<int> indices_cloud;
        std::vector<float> dists_cloud;
        if (tree.radiusSearch(cloud_in->points[i], r, indices_cloud, dists_cloud) >= size)
        {
            seed_queue.push_back(i);
            cloud_processed[i] = true;
        }
        else
        {
            //cloud_processed[i] = true;
            continue;
        }

        int seed_index = 0;
        while (seed_index < seed_queue.size())
        {
            std::vector<int> indices;
            std::vector<float> dists;
            if (tree.radiusSearch(cloud_in->points[seed_queue[seed_index]], r, indices, dists) < size)//函数返回值为近邻数量
            {
                ++seed_index;
                continue;
            }
            for (size_t j = 0; j < indices.size(); ++j)
            {
                if (cloud_processed[indices[j]])
                {
                    continue;
                }
                seed_queue.push_back(indices[j]);
                cloud_processed[indices[j]] = true;
            }
            ++seed_index;
        }
        clusters_index.push_back(seed_queue);

    }

    visual_cloud->width = cloud_in->size();
    visual_cloud->height = 1;
    visual_cloud->resize(cloud_in->size());

    now_classes.resize(clusters_index.size());
    for (size_t i = 0; i < clusters_index.size(); ++i)
    {
        //int R = rand()%255, G = rand()%255, B = rand()%255;
        float sumx=0,sumy=0;
        now_classes[i].points.resize(clusters_index[i].size());
        for (size_t j = 0; j < clusters_index[i].size(); ++j)
        {
            pcl::PointXYZ p;
            visual_cloud->points[clusters_index[i][j]].x = cloud_in->points[clusters_index[i][j]].x;
            visual_cloud->points[clusters_index[i][j]].y = cloud_in->points[clusters_index[i][j]].y;
            visual_cloud->points[clusters_index[i][j]].z = cloud_in->points[clusters_index[i][j]].z;
            visual_cloud->points[clusters_index[i][j]].r=255;
            visual_cloud->points[clusters_index[i][j]].g=255;
            visual_cloud->points[clusters_index[i][j]].b=255;

            sumx+=cloud_in->points[clusters_index[i][j]].x;
            sumy+=cloud_in->points[clusters_index[i][j]].y;
            p.x = cloud_in->points[clusters_index[i][j]].x, p.y = cloud_in->points[clusters_index[i][j]].y, p.z = 0;
            now_classes[i].points[j] = p;
        }
        sumx /= clusters_index[i].size(), sumy /= clusters_index[i].size();
        now_classes[i].center.x = sumx, now_classes[i].center.y = sumy, now_classes[i].center.z = 0;
    }

    return visual_cloud;
}

/**
 * @brief 开启相机捕获线程
 * 
 */
void K4a_Grabber::start_Thread_Capture()
{
    if(Thread_Capture_Working)
    {
        printf("Thread Capture Is Working!\n");
        return ;
    }
        
    Thread_Capture_Working = true;
    pthread_create(&id_thread_capture, NULL, thread_capture, NULL);
}

/**
 * @brief 获取相机捕获线程是否在工作
 * 
 * @return 相机捕获线程是否在工作
 */
bool K4a_Grabber::is_Thread_Capture_Working()
{
    return Thread_Capture_Working;
}

// 捕获线程
void* K4a_Grabber::thread_capture(void *arg)
{
    while(1)
    {
        if(K4a_Grabber_Ptr_->kill_thread_capture)
            break;

        k4a::capture capture;

        if(!K4a_Grabber_Ptr_->Kinect.get_capture(&capture, std::chrono::milliseconds(100)))
        {
            printf("KinectAzureDK Grabber Failed!\n");
            continue ;
        }

        k4a::image colorImage_k4a = nullptr, depthImage_k4a = nullptr, infraredImage_k4a = nullptr;

        colorImage_k4a = capture.get_color_image();
        if(colorImage_k4a == nullptr)
            printf("Failed To Get Color Image From Kinect!\n");

        depthImage_k4a = capture.get_depth_image();
        if(depthImage_k4a == nullptr)
            printf("Failed To Get Depth Image From Kinect!\n");

        infraredImage_k4a = capture.get_ir_image();
        if(infraredImage_k4a == nullptr)
            printf("Failed To Get IR Image From Kinect!\n");

        pthread_mutex_lock(&K4a_Grabber_Ptr_->Raw_Kinect_Data_.mutex_k4a_image_t);
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.colorImage_k4a = k4a::image(colorImage_k4a);
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.depthImage_k4a = k4a::image(depthImage_k4a);
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.infraredImage_k4a = k4a::image(infraredImage_k4a);
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.Used_In_Img = false;
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.Used_In_FastPointXYZ = false;
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.Used_In_PointXYZ = false;
        K4a_Grabber_Ptr_->Raw_Kinect_Data_.Used_In_PointXYZRGB = false;
        pthread_mutex_unlock(&K4a_Grabber_Ptr_->Raw_Kinect_Data_.mutex_k4a_image_t);
    }
}

// 生成XY对应表
void K4a_Grabber::create_xy_table(const k4a::calibration *calibration, k4a::image xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)xy_table.get_buffer();

    int width = calibration->depth_camera_calibration.resolution_width;
    int height = calibration->depth_camera_calibration.resolution_height;

    k4a_float2_t p;
    k4a_float3_t ray;
    int valid;

    for (int y = 0, idx = 0; y < height; y++)
    {
        p.xy.y = (float)y;
        for (int x = 0; x < width; x++, idx++)
        {
            p.xy.x = (float)x;

            k4a_calibration_2d_to_3d(calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

            if (valid)
            {
                table_data[idx].xy.x = ray.xyz.x;
                table_data[idx].xy.y = ray.xyz.y;
            }
            else
            {
                table_data[idx].xy.x = nanf("");
                table_data[idx].xy.y = nanf("");
            }
        }
    }
}

// 生成点云
void K4a_Grabber::generate_point_cloud(const k4a::image depth_image, const k4a::image xy_table, k4a::image point_cloud, int &point_count)
{
    int width = depth_image.get_width_pixels();
    int height = depth_image.get_height_pixels();

    uint16_t *depth_data = (uint16_t *)(void *)depth_image.get_buffer();
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)xy_table.get_buffer();
    k4a_float3_t *point_cloud_data = (k4a_float3_t *)(void *)point_cloud.get_buffer();

    point_count = 0;
    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            point_cloud_data[i].xyz.x = xy_table_data[i].xy.x * (float)depth_data[i];
            point_cloud_data[i].xyz.y = xy_table_data[i].xy.y * (float)depth_data[i];
            point_cloud_data[i].xyz.z = (float)depth_data[i];
            point_count++;
        }
        else
        {
            point_cloud_data[i].xyz.x = nanf("");
            point_cloud_data[i].xyz.y = nanf("");
            point_cloud_data[i].xyz.z = nanf("");
        }
    }
}

// 计算时间差
double K4a_Grabber::cal_Diff_Time_ms(timeval first_time, timeval second_time)
{
    double diff_time_ms = 1e3*(second_time.tv_sec - first_time.tv_sec) + (second_time.tv_usec - first_time.tv_usec)/1000.0;
    if(diff_time_ms < 0)
        diff_time_ms *= -1.0;
    return diff_time_ms;
}


