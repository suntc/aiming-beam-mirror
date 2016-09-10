#ifndef IMAGEACQUISITION_H
#define IMAGEACQUISITION_H

#include "videoinput.h"
#include "videopointgrey.h"
#include "opencv2/core.hpp"

class imageAcquisition
{
public:
    cv::Mat frame;
    bool ctrl = false;
    bool thread = false;
    bool inAcquisition = false;
    bool ready = false;
    int channel = 1;
    double threshold = 0.96;
    imageAcquisition();
    void startAcquisition();
    VideoInput *cam;
    void startupCamera(int ch, float thres);
    void shutdownCamera();
    void set_lifetime(double val, int channel);
private:
    double ch1_tau;
    double ch2_tau;
    double ch3_tau;
    double ch4_tau;
};

#endif // IMAGEACQUISITION_H
