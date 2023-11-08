#include "kinect_init.h"
#ifndef ROBOT_LOCATOR_H_
#define ROBOT_LOCATOR_H_

#define SHOOT_RGB_Camera 1


struct PicRoi
{
	int leftEdge = 0;
	int rightEdge = 0;
	int topEdge = 0;
	int bottomEdge = 0;
};

typedef struct
{
	float a = 0.f;
	float b = 0.f;
	float c = 0.f;
	float d = 0.f;
}planeIndices;

typedef struct
{
	float a = 0.f;
	float b = 0.f;
	float c = 0.f;
}lineIndices;

typedef struct
{
	float left = 0.f;
	float right = 0.f;
	float height = 0.f;
	float distance = 0.f;
}stair;

typedef struct
{
	float left = 0.f;
	float distance = 0.f;
	float angle = 0.f;
	char status = 0;
}locate;




class RobotLocator
{
public:
	KinectInit*         thiskinectDK;
//function
    RobotLocator();
	RobotLocator(const RobotLocator&) = delete;
	RobotLocator& operator=(const RobotLocator&) = delete;
	~RobotLocator();

    void init(KinectInit& kinectDK);
	void updateImage(void);
	Mat imageThresh(Mat inPic, int highH, int lowH, int highS, int lowS, int highV, int lowV);
	
	void showPointCloud(void);
	void Identify(void);
	void rgbdGet(void);
	
	bool pointCloudGenerate(const vector<Point2i > &roiXYs, pPointCloud &outCloud, vector<cv::Point2f> points);
	bool gen3DPoint(Point2f twodPoint, Point3f& thrdPoint);
	float calPoint2LineDis(Point2f point,float a ,float b,float c);


private:
	

//opencv
	vector<cv::Point2f> points;

	cv::Mat		srcImage;
	cv::Mat		rgbdImage;
	cv::Mat		DImage;
	cv::Mat		dstImage;

	
//k4a	
	k4a_image_t transformed_depth_image = NULL;
	k4a_image_t depth_image = NULL;
    k4a_image_t color_image = NULL;

//pcl
	pPointCloud		srcCloud;
	pPointCloud		dogCloud;
	pPointCloud		firstCloud;
	pPointCloud		secondCloud;
	pPointCloud		thirdCloud;
	pcl::visualization::PCLVisualizer::Ptr dstViewer;

	pcl::ExtractIndices<pcl::PointXYZRGB> extract;

//std
	int picNum = 0;
	std::string picName;
	PicRoi colorRoi;
	double angleAlpha;
	


};
#endif
