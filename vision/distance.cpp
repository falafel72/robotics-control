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

// constants for StereoSGBM 
#define NUM_DISPARITY 16*25
#define PRE_FILTER_SIZE 5
#define PRE_FILTER_CAP 1
#define SAD_WINDOW_SIZE 21
#define CONST_P1 8*1*SAD_WINDOW_SIZE*SAD_WINDOW_SIZE
#define CONST_P2 32*1*SAD_WINDOW_SIZE*SAD_WINDOW_SIZE
#define MIN_DISPARITY 5
#define UNIQUENESS_RATIO 0
#define SPECKLE_WIN_SIZE 1
#define SPECKLE_RANGE 24
#define DISP_12_MAX_DIFF 96
#define FULL_DP false
#define TEXTURE_THRESH 10

#define DISPLACEMENT 0.2 //displacement in m
#define SENSOR_WIDTH 5.37
#define SENSOR_HEIGHT 4.04

#define POINT_X 837
#define POINT_Y 478

#define POINT_X2 1132
#define POINT_Y2 458

#define POINT_X3 1257
#define POINT_Y3 598

vector<string> getCalibFiles() {
	vector<string> filenames;
	DIR *dir;
	struct dirent *ent; 
	if((dir = opendir("./calibrate/trial3")) != NULL) {
		while((ent = readdir(dir)) != NULL) {
            const char* dirname = ent->d_name;
			if(strlen(dirname) > 4){
                char fext[4];
                strncpy(fext,dirname+strlen(dirname)-4,4);
                if(strncmp(fext,".png",4) == 0) filenames.push_back(ent->d_name);
    	    }
		}
		closedir(dir);
	}
	return filenames;
}

void getFrames(char* path, vector<Mat *> &frames) {
	VideoCapture cap;
	cap.open(path);
	if(!cap.isOpened()) {
		cout << "Cannot open file" << endl;
		return;
	}
	int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	//vector<Mat *> frames;
	Mat frame;
	Mat *ptr;
	while(totalFrames > 0) {
		cap >> frame;
		ptr = &frame;
		frames.push_back(ptr);
	}
	return;
}

void calibrateCamera(CameraCalibrator &c, Size &imageSize) {
	vector<string> filenames = getCalibFiles();	
	Size boardSize = Size(9,6);
	c.addChessboardPoints(filenames,boardSize); 
	double reprojerr = c.calibrate(imageSize);
}

void calibrateCameraVideo(CameraCalibrator &c, Size &imageSize) {
	vector<Mat *> video;
	getFrames("./calibrate/trial3/calibration.avi", video);
	Size boardSize = Size(9,6);
	c.addChessboardPoints(video,boardSize); 
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
		in1 = imread("img/trial3/left.jpg",0);
		in2 = imread("img/trial3/right.jpg",0);

		CameraCalibrator c; 
		Mat cameraMat = imread("./params/mat1.bmp",0);
		//cout << cameraMat << endl;
		Mat distCoeffs = imread("./params/dist1.bmp",0);
		Size imageSize = in1.size();
		//calibrateCamera(c,imageSize);
		if(!cameraMat.data) {
		    calibrateCamera(c, imageSize);
		}
		else {
		   c.setCameraMatrix(cameraMat);
	       c.setDistCoeffs(distCoeffs);
		}

		

		Mat frameL,frameR;
	//	frameL = imread("img/left.jpg",0);
	//	frameR = imread("img/right.jpg",0);
		frameL = c.remap(in1);
		frameR = c.remap(in2);
		//undistort(in1,frameL, c.getCameraMatrix(), c.getDistCoeffs());
		//undistort(in2,frameR, c.getCameraMatrix(), c.getDistCoeffs());
	     Mat frameL2,frameR2;
		resize(frameL,frameL2,Size(imageSize.width/2,imageSize.height/2));
		resize(frameR,frameR2,Size(imageSize.width/2,imageSize.height/2));
		/*camera1 >> frameL;
		camera2 >> frameR;*/

       	imshow("Cam 1", frameL2);
		imshow("Cam 2", frameR2);

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

		Ptr<StereoBM> stereo = StereoBM::create(NUM_DISPARITY,SAD_WINDOW_SIZE);
		stereo->setPreFilterCap(PRE_FILTER_CAP);
		stereo->setPreFilterSize(PRE_FILTER_SIZE);
		stereo->setMinDisparity(MIN_DISPARITY);
		stereo->setTextureThreshold(TEXTURE_THRESH);
		stereo->setUniquenessRatio(UNIQUENESS_RATIO);
		stereo->setSpeckleWindowSize(SPECKLE_WIN_SIZE);
		stereo->setSpeckleRange(SPECKLE_RANGE);
		stereo->setDisp12MaxDiff(DISP_12_MAX_DIFF);
		// Ptr<StereoSGBM> stereo = StereoSGBM::create(MIN_DISPARITY,
		// 											NUM_DISPARITY,
		// 											SAD_WINDOW_SIZE,
		// 											CONST_P1,
		// 											CONST_P2,
		// 											DISP_12_MAX_DIFF,
		// 											PRE_FILTER_CAP,
		// 											UNIQUENESS_RATIO,
		// 											SPECKLE_WIN_SIZE,
		// 											FULL_DP);
		stereo->compute(frameL,frameR,disparity16S);

		double min, max;
		minMaxLoc(disparity16S,&min,&max);
		printf("Min disp: %f Max value: %f \n", min/16, max/16);

		//convert signed to unsigned to be displayed
		disparity16S.convertTo(disparity8U,CV_8UC1,255/(max-min));
		cvtColor(disparity8U,disparity8U,CV_GRAY2RGB);

		//235 1659 344 1260 
		double disparity = disparity16S.at<uchar>(POINT_Y,POINT_X)/16;
		double disparity2 = disparity16S.at<uchar>(POINT_Y2,POINT_X2)/16;
		double disparity3 = disparity16S.at<uchar>(POINT_Y3,POINT_X3)/16;
		circle(disparity8U,Point(POINT_X,POINT_Y),2,Scalar(0,0,255));
		circle(disparity8U,Point(POINT_X2,POINT_Y2),2,Scalar(0,0,255));
		circle(disparity8U,Point(POINT_X3,POINT_Y3),2,Scalar(0,0,255));
		//print distance at the middle of the frame
		printf("Disparity: %f\n",disparity);

		/*get camera focal length
			- from camera matrix after calibrating
			- also need dimensions of image sensor & aperture width
		*/
		double fovx, fovy, focalLength, aspectRatio;
		Point2d principal;
		calibrationMatrixValues(c.getCameraMatrix(),imageSize,SENSOR_WIDTH,SENSOR_HEIGHT,fovx,fovy,focalLength,principal,aspectRatio);
        printf("Focal Length: %f\n",focalLength);
		/*
		need to know displacement between different angles

		plug into formula
			Z = Bf/d
			Depth (m) = Base displacement (m)* focal length (px)/disparity(px)*/


		//resize(disparity8U,disparity8U,Size(512,384));
		double focalPix = (focalLength/SENSOR_WIDTH) * imageSize.width;
		double depth = DISPLACEMENT * focalPix/disparity;
		double depth2 = DISPLACEMENT * focalPix/disparity2;
		double depth3 = DISPLACEMENT * focalPix/disparity3;

		printf("Focal length in pixels: %f\n",focalPix);
		printf("Distances: %f %f %f\n",depth,depth2,depth3);
	    
        Mat dispResize;
        resize(disparity8U,dispResize,Size(imageSize.width/2,imageSize.height/2));
		imshow("disparity", dispResize);

		// Mat R1,R2,P1,P2,Q;
		// vector<float> translation;
		// translation.push_back(0.2);
		// translation.push_back(0);
		// translation.push_back(0);

		// stereoRectify(c.getCameraMatrix(),
		// 				c.getDistCoeffs(),
		// 				c.getCameraMatrix(),
		// 				c.getDistCoeffs(),
		// 				imageSize,
		// 				Mat::zeros(3,3,CV_32F),
		// 				 translation,
		// 				 R1,
		// 				 R2,
		// 				 P1,
		// 				 P2,
		// 				 Q);
		// Mat depthMap;
		// reprojectImageTo3D(disparity16S,depthMap,Q);


		waitKey();

		
	//}
	//cout << CV_VERSION << endl;


	return 0;
}
