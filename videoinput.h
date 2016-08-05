#ifndef VIDEOINPUT_H
#define VIDEOINPUT_H


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class VideoInput {
public:
    VideoInput(void) {};
    ~VideoInput(void) {};
    virtual cv::Mat getNextFrame() = 0;
    virtual bool lastFrame() = 0;
    virtual int getNumberOfFrames() = 0;
    virtual void disconnect() = 0;
};

#endif // VIDEOINPUT_H
