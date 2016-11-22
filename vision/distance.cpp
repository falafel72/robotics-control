/* 
	Receives stereo camera feed and calculates disparity map. 
	Returns distance value in the middle of the screen. 
*/ 


//Import header files
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream.h>


using namespace cv;

int main(int args, char** argv) {

	//get two camera feeds
	cv::VideoCapture camera1, camera2;
	camera1.open(1);
	camera2.open(2);

	if(!camera1.isOpened() || !camera2.isOpened()) {
		std::err << "ERROR: Unable to access camera!" << endl;
	}

	while(waitKey(20) != 'q') {
		Mat3b frame1,frame2;
		camera1 >> frameL;
		camera2 >> frameR;

		imshow("Cam 1", frameL);
		imshow("Cam 2", frameR);

		//calculate disparity map between feeds


		//print distance at the middle of the frame


	}



	return 0;
}
