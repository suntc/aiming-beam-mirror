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
    void getNextStereoFrame(cv::Mat &f1, cv::Mat &f2);
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    void set_resolution(int w, int h);
    bool isStereoAvailable();
    bool isConnected();
private:
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
    int frameHeight;
    int frameWidth;
    cv::Mat RGBimage;
    bool stereoAvailable;
};

#endif // VIDEOEPIPHAN_H

