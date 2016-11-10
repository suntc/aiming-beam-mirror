#ifndef VIDEOEPIPHAN_H
#define VIDEOEPIPHAN_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "videoinput.h"

class VideoEpiphan : public VideoInput
{
public:
    VideoEpiphan();
    cv::Mat getNextFrame();
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    void set_resolution(int w, int h);
    bool isConnected();
private:
    cv::VideoCapture cap;
    int frameHeight;
    int frameWidth;
    cv::Mat RGBimage;
};

#endif // VIDEOEPIPHAN_H

