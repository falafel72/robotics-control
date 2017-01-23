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

    Mat image = imread("calibrate/1.png");
    Size imageSize = image.size();

    // vector<Point2f> corners;
    // bool found = findChessboardCorners(image,Size(9,6),corners);
    // drawChessboardCorners(image, Size(9,6),corners,found);
    
    imshow("original", image);
    
    cout << "1" << endl;
    DIR *dir;
    struct dirent *ent;
    if((dir = opendir("./calibrate")) != NULL) {
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
    else {
	   cerr << "Unable to open directory" << endl;
	return -1;
    }
    
    cout << "2" << endl;
    Size chessSize = Size(9,6);
    int successes = c.addChessboardPoints(filenames, chessSize);
    cout << "3" << endl;
    cout << "Successes: " << successes << endl;
    double reprojerr = c.calibrate(imageSize);
    cout << "4" << endl;
    Mat result = c.remap(image);
    //Mat result;
    //undistort(image,result,c.getCameraMatrix(),c.getDistCoeffs());

    cout << c.getCameraMatrix() << endl;
    imshow("calibrated", result);
    // imwrite("params/mat1.bmp", c.getCameraMatrix());
    // imwrite("params/dist1.bmp", c.getDistCoeffs());

    waitKey();
    return 0;
}
