#include "robot_locator.h"

RobotLocator::RobotLocator():srcCloud(new pointCloud),
dogCloud(new pointCloud),
firstCloud(new pointCloud),
secondCloud(new pointCloud),
thirdCloud(new pointCloud),
dstViewer(new pcl::visualization::PCLVisualizer("Advanced Viewer"))
{
	dstViewer->setBackgroundColor(0.259, 0.522, 0.957);
	dstViewer->addPointCloud<pointType>(srcCloud, "srcCloud");
	dstViewer->addCoordinateSystem(0.2, "view point");
	dstViewer->initCameraParameters();

};
RobotLocator::~RobotLocator()
{

};

void RobotLocator::init(KinectInit& kinectDK)
{
    cout << "Initializing locator..." << endl;

	//-- Set input device
	thiskinectDK = &kinectDK;

    cout << "Initializing locator..." << endl;

	//-- Set input device
	thiskinectDK = &kinectDK;
    

    k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                    thiskinectDK->colorWidth,
                    thiskinectDK->colorHeight,
                    thiskinectDK->colorWidth * (int)sizeof(uint16_t),
                    &transformed_depth_image);

    colorRoi.bottomEdge = thiskinectDK->colorHeight;
    colorRoi.topEdge = 0;
    colorRoi.leftEdge = thiskinectDK->colorWidth/3;
    colorRoi.rightEdge = 2*thiskinectDK->colorWidth/3;
    // cout<<" ROI bottomEdge "<<colorRoi.bottomEdge<<" topEdge "<<colorRoi.topEdge;
    // cout<<" leftEdge "<<colorRoi.leftEdge<<" rightEdge "<<colorRoi.rightEdge<<endl;
    // cout << "Done initialization." << endl;
}
void RobotLocator::updateImage(void)
{
    thiskinectDK->update();
    //srcCloud->clear();
    
    srcCloud = thiskinectDK->pCloud;

    srcImage = thiskinectDK->cv_rgbImage_no_alpha;

    angleAlpha = (25.f/180.f)*CV_PI;

    // Mat DImage1 = thiskinectDK->cv_depth;
    // DImage = Mat(DImage1.rows, DImage1.cols , CV_8UC1); 
    // for(int i=0; i<DImage1.rows ; i++)
    // {
    //     auto data = DImage1.ptr<uint16_t>(i);
    //     for(int j=0; j<DImage1.cols; j++)
    //     {
    //         if(data[j] > 100 && data[j] < 3000)
    //         {
    //             DImage.at<uchar>(i, j) = data[j]*255.f/3000.f;
    //         }
    //         else
    //         {
    //             DImage.at<uchar>(i, j) = 0;
    //         }
    //     }
    // }
    // equalizeHist(DImage, dstImage);
    
}
void RobotLocator::showPointCloud(void)
{
    cv::imshow("srcImage",srcImage);
    // cv::imshow("DImage",DImage);
    // if(char(cv::waitKey(10)) == 'q') 
    // {
    //     picNum++;
    //     picName= std::to_string(picNum) + ".jpg";
    //     cv::imwrite(picName, DImage);

    // }
    dstViewer->updatePointCloud(srcCloud, "srcCloud");
	dstViewer->spinOnce(1);
    cv::waitKey(1);
}

void RobotLocator::rgbdGet(void)
{   
    k4a_transformation_depth_image_to_color_camera(thiskinectDK->transformation, thiskinectDK->depthImage, transformed_depth_image);  
    rgbdImage = cv::Mat(thiskinectDK->colorHeight, thiskinectDK->colorWidth, CV_16UC1, (void *)k4a_image_get_buffer(transformed_depth_image));
}


bool RobotLocator::pointCloudGenerate(const vector<Point2i > &roiXY, pPointCloud &outCloud, vector<cv::Point2f> points)
{
    if(roiXY.size()==0)
    {
        cout<< " Not have enough roiXYs: in function pointCloudGenerate() " << endl;
        return false;
    }

    outCloud->points.clear();
    points.clear();
    uint16_t *transformed_depth_image_data = (uint16_t *)(void *)k4a_image_get_buffer(transformed_depth_image);
    k4a_float3_t	target_point3d_mm;
    k4a_float2_t 	source_point2d;
    int ifvaule;
    pointType pclpoint;
    cv::Point2f point;
    
    for(int i = 0; i < roiXY.size(); i++)
    {
        source_point2d.xy.x = roiXY[i].x;
        source_point2d.xy.y = roiXY[i].y;

        k4a_calibration_2d_to_3d(&thiskinectDK->calibration, 
                                &source_point2d,
                                transformed_depth_image_data[roiXY[i].y * thiskinectDK->colorWidth + int(roiXY[i].x)],
                                K4A_CALIBRATION_TYPE_COLOR,
                                K4A_CALIBRATION_TYPE_COLOR,
                                &target_point3d_mm ,
                                &ifvaule);
        if(ifvaule)
        {
            pclpoint.x = target_point3d_mm.xyz.x;
            pclpoint.y = -target_point3d_mm.xyz.y;
            pclpoint.z = target_point3d_mm.xyz.z;
            outCloud->points.push_back(pclpoint);
        }
        
    }

    if(outCloud->points.size()>0)
    {
        return true;
    }
    else
    {   
        cout<< " Not get enough roiXYs: in function pointCloudGenerate() " << endl;
        return false;
    }
    
    
}

bool RobotLocator::gen3DPoint(Point2f twodPoint, Point3f& thrdPoint)
{

    uint16_t *transformed_depth_image_data = (uint16_t *)(void *)k4a_image_get_buffer(transformed_depth_image);
    k4a_float3_t	target_point3d_mm;
    k4a_float2_t 	source_point2d;
    int ifvaule;

    source_point2d.xy.x = twodPoint.x;
    source_point2d.xy.y = twodPoint.y;

    k4a_calibration_2d_to_3d(&thiskinectDK->calibration, 
                            &source_point2d,
                            transformed_depth_image_data[int(twodPoint.y) * thiskinectDK->colorWidth + int(twodPoint.x)],
                            K4A_CALIBRATION_TYPE_COLOR,
                            K4A_CALIBRATION_TYPE_DEPTH,
                            &target_point3d_mm ,
                            &ifvaule);
    if(ifvaule)
    {
        thrdPoint.x = target_point3d_mm.xyz.x;
        thrdPoint.y = -target_point3d_mm.xyz.y;
        thrdPoint.z = target_point3d_mm.xyz.z;
    }
}




Mat RobotLocator::imageThresh(Mat inPic, int highH, int lowH, int highS, int lowS, int highV, int lowV)
{
    cv::Mat	hsvImage;
    Mat threshImage = Mat::zeros(inPic.rows, inPic.cols, CV_8UC1);
    cvtColor(inPic, hsvImage, cv::COLOR_BGR2HSV);

    for (size_t row = 0; row < hsvImage.rows; row++)
    {
        auto data = hsvImage.ptr<cv::Vec3b>(row);
        for (size_t col = 0; col < hsvImage.cols; col++)
        {
            int hChannel = data[col][0];
            int sChannel = data[col][1];
            int vChannel = data[col][2];

            if(sChannel <= highS && sChannel >= lowS
            && vChannel <= highV && vChannel >= lowV)
            {
                if(lowH <= highH)
                {
                    if(hChannel <= highH && hChannel >= lowH)
                    {
                        threshImage.at<uchar>(row, col) = 255;
                    }
                }else
                {
                    if(hChannel <= highH || hChannel >= lowH)
                    {
                        threshImage.at<uchar>(row, col) = 255;
                    }
                }
            }
        }
        
    } 
    
    // if(lowH <= highH)
    // {
    //     inRange(hsvImage, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), threshImage);
    // }else
    // {
    //     Mat Image1, Image2, midImage;
    //     inRange(hsvImage, Scalar(0, lowS, lowV), Scalar(highH, highS, highV), Image1);
    //     inRange(hsvImage, Scalar(lowH, lowS, lowV), Scalar(180, highS, highV), Image2);
    //     addWeighted(Image1, 0.5, Image2, 0.5, 0, midImage);
    //     threshold(midImage, threshImage, 20, 255, THRESH_BINARY);
    // }

        // inRange(hsvImage, Scalar(75, 75, 75), Scalar(85, 85, 85), threshImage);

    return threshImage;
}
float RobotLocator::calPoint2LineDis(Point2f point,float a ,float b,float c)
{
    return (float)(point.x * a + point.y * b + c)/sqrt(a * a + b * b);
}



void RobotLocator::Identify(void)
{


    showPointCloud();
}

