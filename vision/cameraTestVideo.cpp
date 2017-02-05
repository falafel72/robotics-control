#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/calib3d/calib3d.hpp> 

#include <string.h>
#include <dirent.h>
#include "cameraCalibrator.cpp"
#include <iostream>
using namespace std;

#define CHESS_SIZE Size(9,6)

int main(int argc, char* argv[]) { //generate and save a mat for camera calibration
    CameraCalibrator c;

    // string file = argv[1];
    // string DIR = "calibrate/" + file + ".jpg";

    cv::VideoCapture cap;
    //cout << "hi" << endl;
    //cap.set(CV_CAP_PROP_FOURCC,CV_FOURCC('I','P','D','V'));
    cap.open(argv[1]);
    if (!cap.isOpened()) {
        cout << "Cannot open file" << endl;
        return -1;
    }


    int totalFrames = cap.get(CV_CAP_PROP_FRAME_COUNT);
    vector<Mat *> frameList;
    Mat image;

    // cout << totalFrames << endl;
    // cap >> image;
    // Size imageSize = image.size();
    // totalFrames--;
    while(waitKey(30) != 27 && totalFrames > 0) {
        //if(totalFrames % 5 == 0) {
            Mat frame;
            cap >> frame;
            frameList.push_back(&frame);
            cout << totalFrames << " " << frame.rows << endl;
            //imshow("test",frame);
        //}
        //else {
            cap.grab();
        //}
        //imshow("video",*ptr);
        totalFrames--;
    }

    // for(int i = 0; i < frameList.size(); i++) {
    //     ptr = frameList[i];
    //     imshow("video",*ptr);
    // }

    // Size chessSize = Size(9,6);

    // c.addChessboardPoints(frameList, chessSize);
    // double reprojerr = c.calibrate(imageSize);

    // //Mat result = c.remap(image);
    // Mat result;
    // undistort(image,result,c.getCameraMatrix(),c.getDistCoeffs());
    // cout << c.getCameraMatrix() << endl;
    // imshow("calibrated", result);
    // imwrite("params/mat1.bmp", c.getCameraMatrix());
    // waitKey();
    // imwrite("params/dist1.bmp", c.getDistCoeffs());
    return 0;
}
