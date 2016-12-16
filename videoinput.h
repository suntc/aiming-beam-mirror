#ifndef VIDEOINPUT_H
#define VIDEOINPUT_H


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class VideoInput {
public:
    VideoInput(void) {};
    ~VideoInput(void) {};
    virtual cv::Mat getNextFrame() = 0;
    virtual void getNextStereoFrame(cv::Mat &f1, cv::Mat &f2) = 0;
    virtual bool lastFrame() = 0;
    virtual int getNumberOfFrames() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() = 0;
    virtual bool isStereoAvailable() = 0;
    virtual void set_resolution(int w, int h) = 0;
};

#endif // VIDEOINPUT_H
