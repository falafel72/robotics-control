/* 
	Receives stereo camera feed and calculates disparity map. 
	Returns distance value in the middle of the screen. 
*/ 


//Import header files
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <stdio.h>


using namespace cv;
using namespace std;

int main(int args, char** argv) {

	//get two camera feeds
	/*cv::VideoCapture camera1, camera2;
	camera1.open(1);
	camera2.open(2);

	if(!camera1.isOpened() || !camera2.isOpened()) {
		cerr << "ERROR: Unable to access camera!" << endl;
	}
*/
	//while(waitKey(20) != 'q') {
		Mat frameL,frameR;
		frameL = imread("img/1.jpg",0);
		frameR = imread("img/2.jpg",0);

		//resize(frameL,frameL,Size(512,384));
		//resize(frameR,frameR,Size(512,384));
		/*camera1 >> frameL;
		camera2 >> frameR;*/

		imshow("Cam 1", frameL);
		imshow("Cam 2", frameR);

		//calculate disparity map between feeds
		int ndisparities = 16*7;
		int SADWindowSize = 21;

		


		Mat disparity16S = Mat(frameL.rows,frameL.cols,CV_16S);
		Mat disparity8U = Mat( frameL.rows, frameR.cols, CV_8UC1 );

		// StereoBM is an abstract class in OpenCV3 for some reason
		//Ptr<StereoBM> stereo = StereoBM::create(ndisparities,SADWindowSize);
		//stereo->compute(frameL,frameR,disparity16S);

		// This is the way to go using OpenCV2
		StereoBM sbm;
		sbm.state->SADWindowSize = 9;
		sbm.state->numberOfDisparities = 112;
		sbm.state->preFilterSize = 5;
		sbm.state->preFilterCap = 61;
		sbm.state->minDisparity = -39;
		sbm.state->textureThreshold = 507;
		sbm.state->uniquenessRatio = 0;
		sbm.state->speckleWindowSize = 0;
		sbm.state->speckleRange = 8;
		sbm.state->disp12MaxDiff = 1;

		sbm(frameL,frameR,disparity16S);

		double min, max;
		minMaxLoc(disparity16S,&min,&max);
		printf("Min disp: %f Max value: %f \n", min, max);

		//convert signed to unsigned to be displayed
		disparity16S.convertTo(disparity8U,CV_8UC1,255/(max-min));
		imshow("disparity", disparity8U);

		waitKey();

		//print distance at the middle of the frame


	//}
	//cout << CV_VERSION << endl;


	return 0;
}
