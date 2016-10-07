#ifndef IMAGEACQUISITION_H
#define IMAGEACQUISITION_H

#include "videoinput.h"
#include "videoinputstereo.h"
#include "videopointgrey.h"
#include "opencv2/core.hpp"
#include "segmentation.h"
#include "stereosegmentation.h"

class imageAcquisition
{
public:
    cv::Mat frame;
    bool ctrl = false;
    bool thread = false;
    bool inAcquisition = false;
    bool ready = false;
    bool stereomode = false;
    int channel = 1;
    double threshold = 0.96;
    imageAcquisition(bool stereomode);
    void startAcquisition();
    VideoInput *cam;
    VideoInputStereo *stereo_cam;
    void startupCamera(int ch, float thres);
    void shutdownCamera();
    void set_lifetime(double val, int channel);
    void set_resolution(int w, int h);
    void set_mode(bool stereomode);
    std::string subject;
    int run_number;
    Segmentation* seg;
    StereoSegmentation* seg_stereo;
    StereoCalibration *calib;

private:
    double ch1_tau;
    double ch2_tau;
    double ch3_tau;
    double ch4_tau;
    int width = 852;
    int height = 459;
    std::string infix;
};

#endif // IMAGEACQUISITION_H
