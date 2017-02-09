#ifndef VIDEOINPUTSTEREO_H
#define VIDEOINPUTSTEREO_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class VideoInputStereo {
public:
    VideoInputStereo(void) {};
    ~VideoInputStereo(void) {};
    virtual void getNextFrame(cv::Mat &f1, cv::Mat &f2) = 0;
    virtual bool lastFrame() = 0;
    virtual int getNumberOfFrames() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected(int camID) = 0;
};

#endif // VIDEOINPUTSTEREO_H
