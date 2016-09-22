#ifndef VIDEOPOINTGREYSTEREO_H
#define VIDEOPOINTGREYSTEREO_H

#include "videoinputstereo.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "FlyCapture2.h"

using namespace FlyCapture2;

class VideoPointGreyStereo : public VideoInputStereo
{
public:
    VideoPointGreyStereo();
    cv::Mat getNextFrame(int camID);
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    bool isConnected();
    bool isReady();
private:
    CameraInfo camInfo;
    Camera cam1;
    Camera cam2;
    PGRGuid guid1;
    PGRGuid guid2;
    BusManager busMgr;
    unsigned int numCameras;
    FlyCapture2::Error error;
    Image rawImage;
    Image rgbImage;
    bool connected = false;
    void PrintError( FlyCapture2::Error error );
    bool ready;
    uint serial_top_cam = 16061459;
    uint serial_side_cam = 16061491;
};
#endif // VIDEOPOINTGREYSTEREO_H
