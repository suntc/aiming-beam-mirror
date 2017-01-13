#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H
#include "videoinput.h"
#include "videopointgrey.h"

using namespace std;

class StereoCalibration
{
public:
    StereoCalibration(string filename);
    StereoCalibration(VideoInput * stereoInput, cv::Size boardSize, float squareSize);
    void saveCalibration(string filename);
    void loadCalibration(string filename);
    bool isReady();
    cv::Mat reconstructPoint3D(cv::Mat p_l, cv::Mat p_r);
    cv::Mat getRectifiedIm(cv::Mat img, int camID);
    cv::Mat R, T, E, F;
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat PM1, PM2;
    cv::Mat projectTo3D(cv::Mat frame_l, cv::Mat frame_r);
    cv::Mat projectTo3D_(cv::Mat left, cv::Mat right);
    cv::Mat H1, H2;
    cv::Mat getProjectionMatrix(int ind);
    cv::Point2d getRectifiedPoint(cv::Point2d p, int camID);
    cv::Mat getRotationAngleBetweenCameras();
    //void on_opengl(cv::Mat pts);
    void showStereo(cv::Mat l, cv::Mat r);

private:
    void captureCalibImages(VideoInput * stereoInput, cv::Size boardSize, float squareSize);
    void StereoCalibration::calibrate(const vector<string> imagelist, cv::Size boardSize, float squareSize, bool displayCorners, bool useCalibrated, bool showRectified);


    cv::Mat cameraMatrix0;
    cv::Mat cameraMatrix1;
    cv::Mat distCoeffs0;
    cv::Mat distCoeffs1;
    cv::Mat rmap00; cv::Mat rmap10; cv::Mat rmap01; cv::Mat rmap11; // for rectification
    cv::Size imSize;
    cv::Mat map_x0;
    cv::Mat map_y0;
    cv::Mat map_x1;
    cv::Mat map_y1;

    cv::Mat RX;

    cv::Rect valid0;
    cv::Rect valid1;

    bool ready;
};

#endif // STEREOCALIBRATION_H


