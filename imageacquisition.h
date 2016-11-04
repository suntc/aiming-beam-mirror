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
    cv::Mat ref_frame;
    cv::Mat readout_frame;
    cv::Mat vis_frame;
    cv::Mat frame_on;
    cv::Mat frame_off;
    bool ctrl = false;
    bool thread = false;
    bool inAcquisition = false;
    bool inFocus = false;
    bool ready = false;
    bool stereomode = false;
    bool aiming_beam_bool = false;
    int channel = 1;
    double threshold = 0.96;
    imageAcquisition(bool stereomode);
    void startAcquisition();
    void captureFrame();
    VideoInput *cam;
    VideoInputStereo *stereo_cam;
    void startupCamera(int ch, float thres);
    void shutdownCamera();
    void set_lifetime(double val, int channel);
    void set_resolution(int w, int h);
    void set_mode(bool stereomode);
    void setAimingBeam(bool aiming_beam);
    std::string subject;
    int run_number;
    Segmentation* seg;
    StereoSegmentation* seg_stereo;
    StereoCalibration *calib;

    void setAutoScale(bool autoscale);
    void setScale(double mn, double mx);
    void setAnsi(int ansi);

    std::vector<double> log_pulse_max;
    std::vector<double> log_pulse_min;
    std::vector<double> log_pulse_thres;
    std::vector<double> log_pulse_cur;

private:
    double ch1_tau;
    double ch2_tau;
    double ch3_tau;
    double ch4_tau;
    int width = 852;
    int height = 459;
    std::string infix;
    bool writing = false;
    double scale_max = 2;
    double scale_min = 3;
    bool scale_auto;
    int ansi = 999999;
};

#endif // IMAGEACQUISITION_H
