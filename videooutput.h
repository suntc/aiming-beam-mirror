#ifndef VIDEOOUTPUT_H
#define VIDEOOUTPUT_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class VideoOutput {
public:
    VideoOutput(std::string) {};
    VideoOutput() {};
    ~VideoOutput(void) {};
    virtual void addFrame(cv::Mat) = 0;
    virtual int getNumberOfFrames() = 0;
    virtual void closeFile() = 0;
};

#endif // VIDEOOUTPUT_H
