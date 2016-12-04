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
#include <dirent.h>

#include "cameraCalibrator.cpp" 


using namespace cv;
using namespace std;

#define NUM_DISPARITY 16*5
#define PRE_FILTER_CAP 0
#define SAD_WINDOW_SIZE 5
#define CONST_P1 128
#define CONST_P2 256
#define MIN_DISPARITY 5
#define UNIQUENESS_RATIO 1
#define SPECKLE_WIN_SIZE 0
#define SPECKLE_RANGE 2
#define DISP_12_MAX_DIFF 20
#define FULL_DP true

//#define DISPLACEMENT 

vector<string> getCalibFiles() {
	vector<string> filenames;
	DIR *dir;
	struct dirent *ent; 
	if((dir = opendir("./calibrate")) != NULL) {
		while((ent = readdir(dir)) != NULL) {
			if(strncmp(ent->d_name,".",2) != 0 && strncmp(ent->d_name,"..",2) != 0 && strncmp(ent->d_name,".DS_Store",9) != 0) { // get rid of "." and ".." directories
				filenames.push_back(ent->d_name);
			}
		}
		closedir(dir);
	}
	return filenames;
}

void calibrateCamera(CameraCalibrator &c, Size &imageSize) {
	vector<string> filenames = getCalibFiles();	
	//cout << filenames << endl;	
	Size boardSize = Size(9,6);
	c.addChessboardPoints(filenames,boardSize); 
	double reprojerr = c.calibrate(imageSize);
}

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
		Mat in1,in2;
		in1 = imread("img/1.jpg",0);
		in2 = imread("img/2.jpg",0);

		CameraCalibrator c; 
		//Mat cameraMat = imread("./params/mat1.bmp",0);
		//cout << cameraMat << endl;
		//Mat distCoeffs = imread("./params/dist1.bmp",0);
		Size imageSize = in1.size();
		calibrateCamera(c,imageSize);
		// if(!cameraMat.data) {
		// 	calibrateCamera(c, imageSize);
		// }
		// else {
		// 	c.setCameraMatrix(cameraMat);
		// 	c.setDistCoeffs(distCoeffs);
		// }

		

		Mat frameL,frameR;
		undistort(in1,frameL, c.getCameraMatrix(), c.getDistCoeffs());
		undistort(in2,frameR, c.getCameraMatrix(), c.getDistCoeffs());
		
		// resize(frameL,frameL,Size(512,384));
		// resize(frameR,frameR,Size(512,384));
		/*camera1 >> frameL;
		camera2 >> frameR;*/

		imshow("Cam 1", frameL);
		imshow("Cam 2", frameR);

		//calculate disparity map between feeds
		Mat disparity16S = Mat(frameL.rows,frameL.cols,CV_16S);
		Mat disparity8U = Mat( frameL.rows, frameR.cols, CV_8UC1 );

		// StereoBM is an abstract class in OpenCV3 for some reason
		//Ptr<StereoBM> stereo = StereoBM::create(ndisparities,SADWindowSize);
		//stereo->compute(frameL,frameR,disparity16S);

		// This is the way to go using OpenCV2
		/*StereoBM sbm;
		sbm.state->SADWindowSize = SAD_WINDOW_SIZE;
		sbm.state->numberOfDisparities = NUM_DISPARITY;
		sbm.state->preFilterSize = 5;
		//sbm.state->preFilterCap = PRE_FILTER_CAP;
		sbm.state->minDisparity = MIN_DISPARITY;
		sbm.state->textureThreshold = 507;
		sbm.state->uniquenessRatio = UNIQUENESS_RATIO;
		sbm.state->speckleWindowSize = SPECKLE_WIN_SIZE;
		sbm.state->speckleRange = SPECKLE_RANGE;
		sbm.state->disp12MaxDiff = DISP_12_MAX_DIFF;

		sbm(frameL,frameR,disparity16S);*/

		/*StereoSGBM ssgbm;
		ssgbm.minDisparity = 0;
		ssgbm.numberOfDisparities = 32;
		ssgbm.SADWindowSize = 3;
		ssgbm.P1 = 128;
		ssgbm.P2 = 256;
		ssgbm.disp12MaxDiff = 20;
		ssgbm.preFilterCap = 16;
		ssgbm.uniquenessRatio = 1;
		ssgbm.speckleWindowSize = 100;
		ssgbm.fullDP = true;

		ssgbm.compute(frameL,frameR,disparity16S);*/

		Ptr<StereoSGBM> stereo = StereoSGBM::create(MIN_DISPARITY,
													NUM_DISPARITY,
													SAD_WINDOW_SIZE,
													CONST_P1,
													CONST_P2,
													DISP_12_MAX_DIFF,
													PRE_FILTER_CAP,
													UNIQUENESS_RATIO,
													SPECKLE_WIN_SIZE,
													FULL_DP);
		stereo->compute(frameL,frameR,disparity16S);

		double min, max;
		minMaxLoc(disparity16S,&min,&max);
		printf("Min disp: %f Max value: %f \n", min, max);

		//convert signed to unsigned to be displayed
		disparity16S.convertTo(disparity8U,CV_8UC1,255/(max-min));

		double disparity = disparity8U.at<uchar>(disparity8U.rows/2,disparity8U.cols/2);
		circle(disparity8U,Point(disparity8U.cols/2,disparity8U.rows/2),10,Scalar(255,255,255));
		//print distance at the middle of the frame
		printf("Disparity: %f",disparity);

		/*get camera focal length
			- from camera matrix after calibrating
			- also need dimensions of image sensor & aperture width
		calibrationMatrixValues(c.getCameraMatrix(),imageSize,)

		need to know displacement between different angles

		plug into formula
			Z = Bf/d
			Depth = Base displacement * focal length/disparity*/



		imshow("disparity", disparity8U);

		waitKey();

		
	//}
	//cout << CV_VERSION << endl;


	return 0;
}
