#include "kinect_init.h"

std::ofstream outfile("../record/calib/calib.txt");


KinectInit::KinectInit():pCloud(new pointCloud)
{
	xy_table = NULL;
}

KinectInit::~KinectInit()
{
    
}
//对kinect DK相近进行初始化设置并启动
int KinectInit::init()
{
    /*
		找到并打开 Azure Kinect 设备
	*/
	// 发现已连接的设备数
	const uint32_t device_count = k4a::device::get_installed_count();
	if (0 == device_count)	
	{
		std::cout << "Error: no K4A devices found. " << std::endl;
		return EXIT_FAILURE;
	}
	else
	{
		std::cout << "Found " << device_count << " connected devices. " << std::endl;
 
		if (1 != device_count)// 超过1个设备，也输出错误信息。
		{
			std::cout << "Error: more than one K4A devices found. " << std::endl;
			return EXIT_FAILURE;
		}
		else// 该代码仅限对1个设备操作
		{
			std::cout << "Done: found 1 K4A device. " << std::endl;
		}		
	}
	// 打开（默认）设备
	if (K4A_RESULT_SUCCEEDED != k4a_device_open(K4A_DEVICE_DEFAULT, &device))
    {
        printf("Failed to open device\n");
        return EXIT_FAILURE;
    }
    

	/*
		检索 Azure Kinect 图像数据
	*/
	// 配置并启动设备
	k4a_device_configuration_t config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;

    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
	config.color_resolution = K4A_COLOR_RESOLUTION_720P;
	// config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
	config.depth_mode = K4A_DEPTH_MODE_NFOV_2X2BINNED;
	config.synchronized_images_only = true;// ensures that depth and color images are both available in the capture

	 if (K4A_RESULT_SUCCEEDED != k4a_device_start_cameras(device, &config))
    {
        printf("Failed to start cameras\n");
        return EXIT_FAILURE;
    }

    if (K4A_RESULT_SUCCEEDED != k4a_device_start_imu(device))
    {
        printf("Failed to start imu\n");
        return EXIT_FAILURE;
    }

    // 稳定化
	
	int iAuto = 0;//用来稳定，类似自动曝光
	int iAutoError = 0;// 统计自动曝光的失败次数
	while (true)
	{
		if (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS) == k4a_result_t::K4A_RESULT_SUCCEEDED)
		{
			std::cout << iAuto << ". Capture several frames to give auto-exposure" << std::endl;
 
			// 跳过前 n 个（成功的数据采集）循环，用来稳定
			if (iAuto != 30)
			{
				iAuto++;
				continue;
			}
			else
			{
				std::cout << "Done: auto-exposure" << std::endl;
				break;// 跳出该循环，完成相机的稳定过程
			}
		}
		else
		{
			std::cout << iAutoError << ". K4A_WAIT_RESULT_TIMEOUT." << std::endl;
			if (iAutoError != 30)
			{
				iAutoError++;
				continue;
			}
			else
			{
				std::cout << "Error: failed to give auto-exposure. " << std::endl;
				return EXIT_FAILURE;
			}
		}
	}
	std::cout << "-----------------------------------" << std::endl;
	std::cout << "----- Have Started Kinect DK. -----" << std::endl;
	std::cout << "-----------------------------------" << std::endl;

    //获得xy_table
	
    if (K4A_RESULT_SUCCEEDED !=
        k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration))
    {
        printf("Failed to get calibration\n");
        return EXIT_FAILURE;
    }
    
    k4a_image_create(K4A_IMAGE_FORMAT_CUSTOM,
                     calibration.depth_camera_calibration.resolution_width,
                     calibration.depth_camera_calibration.resolution_height,
                     calibration.depth_camera_calibration.resolution_width * (int)sizeof(k4a_float2_t),
                     &xy_table);

    create_xy_table(&calibration, xy_table);
//get width and height
    colorWidth = calibration.color_camera_calibration.resolution_width;
    colorHeight = calibration.color_camera_calibration.resolution_height;

    std::cout << "-----------------------------------" << std::endl;
	std::cout << "------- Have geted xy_tbale.-------" << std::endl;
	std::cout << "-----------------------------------" << std::endl;

    outfile << "Pose: " << (float)1.f << " " << (float)0.f << " " << (float)0.f << " " << (float)1.f << " "
                        << (float)0.f << " " << (float)1.f << " " << (float)0.f << " " << (float)1.f << " "
                        << (float)0.f << " " << (float)0.f << " " << (float)1.f << " " << (float)1.f << std::endl;


    outfile << "intrinsics: " 
            << calibration.color_camera_calibration.intrinsics.parameters.param.fx << " " 
            << (float)0.f << " " 
            << calibration.color_camera_calibration.intrinsics.parameters.param.cx << " " 
            << (float)0.f << " " 
            << calibration.color_camera_calibration.intrinsics.parameters.param.fy << " "  
            << calibration.color_camera_calibration.intrinsics.parameters.param.cy << " " 
            << (float)0.f << " "
            << (float)0.f << " "
            << (float)1.f << std::endl;
            
    outfile << "Tr_depth_to_rgb: " 
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[0] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[1] << " " 
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[2] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[0] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[3] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[4] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[5] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[1] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[6] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[7] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].rotation[8] << " "
            << calibration.extrinsics[K4A_CALIBRATION_TYPE_DEPTH][K4A_CALIBRATION_TYPE_COLOR].translation[2]
            << std::endl;
            
//get transformation
    transformation = k4a_transformation_create(&calibration);
    // Capture a imu sample
    int flag = 0;
    while(1)
    {
        
        switch (k4a_device_get_imu_sample(device, &imu_sample, TIMEOUT_IN_MS))
        {
            case K4A_WAIT_RESULT_SUCCEEDED:
            flag = 1;
                break;
            case K4A_WAIT_RESULT_TIMEOUT:
                printf("Timed out waiting for a imu sample\n");
                continue;
                break;
            case K4A_WAIT_RESULT_FAILED:
                printf("Failed to read a imu sample\n");
                return EXIT_FAILURE;
        }
    
        if (flag)
        {
            
            cout << " | Accelerometer temperature: " << imu_sample.temperature << " x: " << imu_sample.acc_sample.xyz.x
                << " y: " << imu_sample.acc_sample.xyz.y << " z: " << imu_sample.acc_sample.xyz.z << endl;
            break;
        }
    }

    return 1;

}
//更新图像并获得深度图与RGB图
void KinectInit::update()
{
    
    if (k4a_device_get_capture(device, &capture, TIMEOUT_IN_MS) == k4a_result_t::K4A_RESULT_SUCCEEDED)
    {
        // Get a RGB image
        rgbImage = k4a_capture_get_color_image(capture);
        cv_rgbImage_with_alpha = cv::Mat(colorHeight, colorWidth, CV_8UC4, (void *)k4a_image_get_buffer(rgbImage));
		cv::cvtColor(cv_rgbImage_with_alpha, cv_rgbImage_no_alpha, cv::COLOR_BGRA2BGR);

        // Get a depth image
        depthImage = k4a_capture_get_depth_image(capture);
        cv_depth = cv::Mat(k4a_image_get_height_pixels(depthImage), k4a_image_get_width_pixels(depthImage)
                            , CV_16U, k4a_image_get_buffer(depthImage), static_cast<size_t>(k4a_image_get_stride_bytes(depthImage)));
		// cv_depth.convertTo(cv_depth_8U, CV_8U, 1 );
		pointNum = 0;
		generate_point_cloud(depthImage, xy_table, pCloud, &pointNum);

        

        // k4a_image_t transformed_depth_image = NULL;

        // k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
        //              colorWidth,
        //              colorHeight,
        //              colorWidth * (int)sizeof(uint16_t),
        //              &transformed_depth_image);

        // k4a_transformation_depth_image_to_color_camera(transformation, depthImage, transformed_depth_image);  

    }
    else
    {
        init();
        update();
    }

    // Access the accelerometer readings
    
	
}
//计算xy_table
void KinectInit::create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table)
{
    k4a_float2_t *table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

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

            k4a_calibration_2d_to_3d(
                calibration, &p, 1.f, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, &ray, &valid);

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
//生成点云
//此处使用快速点云生成算法 也可使用k4a_transformation_color_image_to_depth_camera()函数 ，没有测试
void KinectInit::generate_point_cloud(const k4a_image_t depth_image,
                                    const k4a_image_t xy_table,
                                    pPointCloud point_cloud,
                                    int *point_count)
{
    int width = k4a_image_get_width_pixels(depth_image);
    int height = k4a_image_get_height_pixels(depth_image);

    uint16_t *depth_data = (uint16_t *)(void *)k4a_image_get_buffer(depth_image);
    k4a_float2_t *xy_table_data = (k4a_float2_t *)(void *)k4a_image_get_buffer(xy_table);

    point_cloud->width = width;
	point_cloud->height = height;
	point_cloud->is_dense = false;
    point_cloud->points.resize(width*height);

    *point_count = 0;
    // int numcount = 0;

    //录制点云
    pcl::PointCloud<pcl::PointXYZ> cloud; // 创建点云（不是指针） 
    //填充点云数据
    cloud.width = point_cloud->width;                                 //设置点云宽度
    cloud.height = point_cloud->height;                                //设置点云高度
    cloud.is_dense = false;                          //非密集型
    cloud.points.resize(cloud.width * cloud.height); //变形，无序      



    for (int i = 0; i < width * height; i++)
    {
        if (depth_data[i] != 0 && !isnan(xy_table_data[i].xy.x) && !isnan(xy_table_data[i].xy.y))
        {
            float high = xy_table_data[i].xy.y * (float)depth_data[i];
            float depth = (float)depth_data[i];

            point_cloud->points[i].x = (-1.0*xy_table_data[i].xy.x * (float)depth_data[i]);
            point_cloud->points[i].y = (-1.0*xy_table_data[i].xy.y * (float)depth_data[i]);
            point_cloud->points[i].z = ((float)depth_data[i]);
            (*point_count)++;
            // numcount++;

            cloud.points[i].x = (1.0*xy_table_data[i].xy.x * (float)depth_data[i]);
            cloud.points[i].y = (1.0*xy_table_data[i].xy.y * (float)depth_data[i]);
            cloud.points[i].z = ((float)depth_data[i]);
            
            
        }
        else
        {
            point_cloud->points[i].x = nanf("");
            point_cloud->points[i].y = nanf("");
            point_cloud->points[i].z = nanf("");

            // cloud.points[i].x = nanf("");
            // cloud.points[i].y = nanf("");
            // cloud.points[i].z = nanf("");
        }

    

    }
    static int count = 0;
    std::string new_name;
    stringstream ss;
    ss << count;
    count++; 
    std::string path = "../record/pcd";
    string filename = new_name.assign(path).append("/") + string("KIN_") + string(ss.str() + ".pcd");
    //将点云保存到PCD文件中
    //savePCDFileASCII 保存为ASICC格式，可以直接用记事本打开
    //savePCDFileBinary 保存为Binary格式，不可用记事本打开，但更快速
    pcl::io::savePCDFileASCII(filename, cloud);
    std::cerr << "Saved " << cloud.points.size() << " data points to test_pcd.pcd." << std::endl;


    //默认false(可以不写)，保存为ASICC格式  true，保存为Binary格式
    // pcl::PCDWriter writer;
    // writer.write(filename, *point_cloud,false);

    // std::cout << "numcount = " << numcount << std::endl;
}

void KinectInit::release(void)
{
    k4a_image_release(depthImage);
    k4a_image_release(rgbImage);
    k4a_capture_release(capture);
}