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
#define NUM_DISPARITY 16*7
#define PRE_FILTER_CAP 1
#define PRE_FILTER_SIZE 5
#define SAD_WINDOW_SIZE 5
// #define CONST_P1 8*1*SAD_WINDOW_SIZE*SAD_WINDOW_SIZE
// #define CONST_P2 32*1*SAD_WINDOW_SIZE*SAD_WINDOW_SIZE
#define MIN_DISPARITY 0
#define UNIQUENESS_RATIO 5
#define SPECKLE_WIN_SIZE 0
#define SPECKLE_RANGE 20
#define DISP_12_MAX_DIFF 64
#define FULL_DP false
#define TEXTURE_THRESH 10

#define DISPLACEMENT 0.2 //displacement in m
#define SENSOR_WIDTH 5.37
#define SENSOR_HEIGHT 4.04

#define POINT_X 828
#define POINT_Y 190
vector<string> getCalibFiles(char* path) {
	vector<string> filenames;
	DIR *dir;
	struct dirent *ent; 
	if((dir = opendir(path)) != NULL) {
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

vector<Mat *> getFrames(char* path) {
	vector<Mat *> frames;
	VideoCapture cap;
	cap.open(path);
	if(!cap.isOpened()) {
		cout << "Cannot open file" << endl;
		return frames;
	}
	int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
	Mat frame;
	Mat *ptr;
	while(totalFrames > 0) {
		if(totalFrames % 5 == 0) {
			cout << totalFrames << endl;
            cap >> frame;
            frames.push_back(&frame);
            imshow("test",frame);
        }
        else {
            cap.grab();
        }
        totalFrames--;
	}
	cap.release();
	return frames;
}

void calibrateCamera(CameraCalibrator &c, Size &imageSize) {
	vector<string> filenames = getCalibFiles("./calibrate/trial3");	
	Size boardSize = Size(9,6);
	c.addChessboardPoints(filenames,boardSize); 
	double reprojerr = c.calibrate(imageSize);
}

void calibrateCameraVideo(CameraCalibrator &c, Size &imageSize) {
	vector<Mat *> video;
	video = getFrames("./calibrate/trial2/calibration.avi");
	Size boardSize = Size(9,6);
	// Mat* ptr = video[0];
	// Mat image = *ptr;
	// imshow("test",image);
	// waitKey();
	c.addChessboardPoints(video,boardSize); 
	double reprojerr = c.calibrate(imageSize);
}

int main(int args, char** argv) {

	//get two camera feeds
	VideoCapture camera1;
	VideoCapture camera2;
	camera1.open("img/trial4/left.mp4");
	camera2.open("img/trial4/right.mp4");
 	if(!camera1.isOpened()) {
		cerr << "ERROR: Unable to access camera!" << endl;
	}

	//while(waitKey(20) != 'q') {
		CameraCalibrator c; 
		Mat cameraMat = imread("./params/mat1.bmp");
		//cout << cameraMat << endl;
		Mat distCoeffs = imread("./params/dist1.bmp");
		Size imageSize = Size(camera1.get(CV_CAP_PROP_FRAME_WIDTH),camera1.get(CV_CAP_PROP_FRAME_HEIGHT));
		//calibrateCamera(c,imageSize);
		if(!cameraMat.data) {
		    calibrateCamera(c, imageSize);
		}
		else {
		   c.setCameraMatrix(cameraMat);
	       c.setDistCoeffs(distCoeffs);
		}
		int totalFrames = camera1.get(CV_CAP_PROP_FRAME_COUNT);

		Mat disparity16S = Mat(imageSize,CV_16S);
		Mat disparity8U = Mat(imageSize, CV_8UC1 );
		Mat dispResize;


		double fovx, fovy, focalLength, aspectRatio, focalPix, depth, min, max;
		Point2d principal;

		// Ptr<StereoSGBM> stereo = StereoSGBM::create(MIN_DISPARITY,
		// 										NUM_DISPARITY,
		// 										SAD_WINDOW_SIZE,
		// 										CONST_P1,
		// 										CONST_P2,
		// 										DISP_12_MAX_DIFF,
		// 										PRE_FILTER_CAP,
		// 										UNIQUENESS_RATIO,
		// 										SPECKLE_WIN_SIZE,
		// 										FULL_DP);

		Ptr<StereoBM> stereo = StereoBM::create(NUM_DISPARITY,SAD_WINDOW_SIZE);
		// stereo->setPreFilterCap(PRE_FILTER_CAP);
		// stereo->setPreFilterSize(PRE_FILTER_SIZE);
		// stereo->setMinDisparity(MIN_DISPARITY);
		// stereo->setTextureThreshold(TEXTURE_THRESH);
		// stereo->setUniquenessRatio(UNIQUENESS_RATIO);
		// stereo->setSpeckleWindowSize(SPECKLE_WIN_SIZE);
		// stereo->setSpeckleRange(SPECKLE_RANGE);
		// stereo->setDisp12MaxDiff(DISP_12_MAX_DIFF);

		// /*get camera focal length
		// 	- from camera matrix after calibrating
		// 	- also need dimensions of image sensor & aperture width
		// */
		
		calibrationMatrixValues(c.getCameraMatrix(),imageSize,SENSOR_WIDTH,SENSOR_HEIGHT,fovx,fovy,focalLength,principal,aspectRatio);
        printf("Focal Length: %f\n",focalLength);

        // focal length in pixels is the focal length(mm) * image width(px) / sensor width(mm)
		focalPix = focalLength * imageSize.width/SENSOR_WIDTH;
		printf("Focal Length (px): %f\n",focalPix);
  //      // cout << "hello" << endl;
		Mat frameL,frameR;
		Mat one,two;

		do {
			camera1 >> frameL;
			camera2 >> frameR;
			cvtColor(frameL,frameL,CV_RGB2GRAY);
			cvtColor(frameR,frameR,CV_RGB2GRAY);
			//cout << frameL.rows << " " << frameR.rows << endl;
			totalFrames--;

			
			//camera2.read(frameR);
			// undistort(frameL,one,c.getCameraMatrix(),c.getDistCoeffs());
			// undistort(frameR,two,c.getCameraMatrix(),c.getDistCoeffs());
			one = c.remap(frameL);
			two = c.remap(frameR);
			resize(one,frameL,Size(imageSize.width/2,imageSize.height/2));
			resize(two,frameR,Size(imageSize.width/2,imageSize.height/2));
			imshow("left",frameL);
			imshow("right",frameR);


			stereo->compute(one,two,disparity16S);
			minMaxLoc(disparity16S,&min,&max);
			printf("Min disp: %f Max value: %f \n", min/16, max/16);
			disparity16S.convertTo(disparity8U,CV_8UC1,255/(max-min));

			// //235 1659 344 1260 
			double disparity = disparity16S.at<uchar>(POINT_Y,POINT_X)/16;
			circle(disparity8U,Point(POINT_X,POINT_Y),10,Scalar(255,255,255));

			// //print distance at the middle of the frame
			//printf("Disparity: %f\n",disparity);


			/*
			need to know displacement between different angles

			plug into formula
				Z = Bf/d
				Depth (m) = Base displacement (m)* focal length (px)/disparity(px)*/


			//resize(disparity8U,disparity8U,Size(512,384));

			
			depth = DISPLACEMENT * focalPix/disparity;

			Mat disp16Thresh,disp8Thresh;
	        threshold(disparity16S,disp16Thresh, 100*16, 32767,THRESH_BINARY);
	        disp16Thresh.convertTo(disp8Thresh,CV_8UC1,255/(max-min));

	        resize(disparity8U,dispResize,Size(imageSize.width/2,imageSize.height/2));
	        //printf("Distance: %f",depth);
	        cout << depth << endl;
	        imshow("disparity", dispResize);
			// imshow("threshold", disp8Thresh);
		} while(waitKey(30) != 27 || frameL.empty() || frameR.empty()); 
		

	
		

		// double min, max;
		// minMaxLoc(disparity16S,&min,&max);
		// printf("Min disp: %f Max value: %f \n", min/16, max/16);

		//convert signed to unsigned to be displayed

		
		

		

		// printf("Focal length in pixels: %f\n",focalPix);
		// printf("Distance: %f\n",depth);
	    
        

		// waitKey();
	//}

	return 0;
}
