#include "kinect_init.h"
#include "robot_locator.h"
int main(int argc, char* argv[])
{
	
	KinectInit KinectDK;
	RobotLocator robotLocator;

	//kinect DK init
	KinectDK.init();
	robotLocator.init(KinectDK);

	chrono::steady_clock::time_point start;
	chrono::steady_clock::time_point stop;

	cv::TickMeter tk;

	while (true)
	{
		//总计时函数
		tk.start(); 
		//更新图片
		robotLocator.updateImage();
		//识别障碍
		robotLocator.Identify();
		//释放图片
		robotLocator.thiskinectDK->release();
		
		tk.stop();
		cout << " 程序总时间: " << tk.getTimeMilli() << endl;
		tk.reset();
	}


	return EXIT_SUCCESS;
}

