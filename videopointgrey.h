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
private:
    CameraInfo camInfo;
    Camera cam;
    PGRGuid guid;
    BusManager busMgr;
    unsigned int numCameras;
    FlyCapture2::Error error;
    Image rawImage;
    Image rgbImage;
};

#endif // VIDEOPOINTGREY_H
