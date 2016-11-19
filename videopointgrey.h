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
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    bool isConnected();
    void set_resolution(int w, int h);
private:
    CameraInfo camInfo;
    Camera cam;
    PGRGuid guid;
    BusManager busMgr;
    unsigned int numCameras;
    FlyCapture2::Error error;
    Image rawImage;
    Image rgbImage;
    bool connected = false;
    int width;
    int height;
    cv::Mat image;
};

#endif // VIDEOPOINTGREY_H
