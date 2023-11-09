
#define _CRT_SECURE_NO_WARNINGS

#ifndef KINECT_INIT_H_
#define KINECT_INIT_H_
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace cv;
using namespace Eigen;

typedef pcl::PointXYZRGB 				pointType;
typedef pcl::PointCloud<pointType> 	pointCloud;
typedef pointCloud::Ptr 			pPointCloud;
 
#define RECORD

class KinectInit
{
public:
//function
    KinectInit();
	KinectInit(const KinectInit&) = delete;
	KinectInit operator=(const KinectInit&) = delete;
	~KinectInit();
    int init(void);
	void update(void);
    void create_xy_table(const k4a_calibration_t *calibration, k4a_image_t xy_table);
    void generate_point_cloud(const k4a_image_t depth_image,
                                const k4a_image_t xy_table,
                                pPointCloud point_cloud,
                                int *point_count);
	void release(void);
	
//pcl
	pPointCloud pCloud;
//opencv
	cv::Mat cv_rgbImage_no_alpha;
	cv::Mat depthImage_no_alpha;
	cv::Mat cv_depth_8U;
	cv::Mat cv_depth;

//k4a
	k4a_calibration_t calibration;
	k4a_image_t rgbImage = NULL;
	k4a_image_t depthImage = NULL;
	k4a_transformation_t transformation = NULL;
	k4a_imu_sample_t imu_sample;

//std
	int colorWidth;
	int colorHeight;
	int depthWidth;
	int depthHeight;

private:
 //k4a   
	k4a::image irImage;
    k4a_capture_t capture;
    k4a_device_t device;
	k4a_image_t xy_table;
	

//opencv	
	cv::Mat cv_rgbImage_with_alpha;
	
	cv::Mat cv_irImage;
	cv::Mat cv_irImage_8U;

//std
	int pointNum = 0;
	const int32_t TIMEOUT_IN_MS = 1000;

	


};

#endif 
