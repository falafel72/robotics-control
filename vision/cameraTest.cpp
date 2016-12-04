#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/calib3d/calib3d.hpp> 

#include <string.h>
#include <dirent.h>
#include "cameraCalibrator.cpp"
#include <iostream>
using namespace std;

int main(int argc, char* argv[]) { //generate and save a mat for camera calibration
    CameraCalibrator c;
    vector<string> filenames;

    // string file = argv[1];
    // string DIR = "calibrate/" + file + ".jpg";

    Mat image = imread("calibrate/1.jpg");
    Size imageSize = image.size();

    // vector<Point2f> corners;
    // bool found = findChessboardCorners(image,Size(9,6),corners);
    // drawChessboardCorners(image, Size(9,6),corners,found);

    imshow("original", image);

    DIR *dir;
    struct dirent *ent;
    if((dir = opendir("./calibrate")) != NULL) {
    	while((ent = readdir(dir)) != NULL) {
    	    if(strncmp(ent->d_name,".",2) != 0 && strncmp(ent->d_name,"..",2) != 0 && strncmp(ent->d_name,".DS_Store",9) != 0){
    		  filenames.push_back(ent->d_name);
    	    }
    	}
    	closedir(dir);
    }
    else {
	   cerr << "Unable to open directory" << endl;
	return -1;
    }
    Size chessSize = Size(9,6);
    c.addChessboardPoints(filenames, chessSize);
    double reprojerr = c.calibrate(imageSize);

    //Mat result = c.remap(image);
    Mat result;
    undistort(image,result,c.getCameraMatrix(),c.getDistCoeffs());
    cout << c.getCameraMatrix() << endl;
    imshow("calibrated", result);
    // imwrite("params/mat1.bmp", c.getCameraMatrix());
    // imwrite("params/dist1.bmp", c.getDistCoeffs());

    waitKey();
    return 0;
}
