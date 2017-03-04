#ifndef VIDEOPOINTGREY_H
#define VIDEOPOINTGREY_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "videoinput.h"
#include "FlyCapture2.h"


using namespace FlyCapture2;

class VideoPointGrey : public VideoInput
{
public:
    VideoPointGrey();
    cv::Mat getNextFrame();
    void getNextStereoFrame(cv::Mat &f1, cv::Mat &f2);
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    bool isConnected(int camID);
    bool isStereoAvailable();
    void set_resolution(int w, int h);
private:
    CameraInfo camInfo;
    Camera cam1;
    Camera cam2;
    PGRGuid guid1;
    PGRGuid guid2;
    BusManager busMgr;
    unsigned int numCameras;
    FlyCapture2::Error status;
    Image rawImage1;
    Image rawImage2;
    Image rgbImage1;
    Image rgbImage2;
    bool connected = false;
    int width;
    int height;
    cv::Mat image;
    bool stereoAvailable = false;
    bool cam1_available = false;
    bool cam2_available = false;

    // Serial numbers to identify the cameras
    uint serial_top_cam = 16061459;
    uint serial_side_cam = 16061491;
};

#endif // VIDEOPOINTGREY_H
