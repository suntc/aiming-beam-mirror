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

    imageAcquisition(bool stereomode);

    cv::Mat frame;
    cv::Mat ab_frame;
    cv::Mat bg_frame;
    cv::Mat ab_frame2;
    cv::Mat bg_frame2;
    cv::Mat vis_frame;
    cv::Mat frame_on;
    cv::Mat frame_off;
    cv::Mat frame_on2;
    cv::Mat frame_off2;

    bool cleanup = true;
    bool ctrl = false;
    bool thread = false;
    bool inAcquisition = false;
    bool inFocus = false;
    bool ready = false;
    bool ready_fg = false;
    bool ready_usb = false;
    bool stereomode = false;
    bool aiming_beam_bool = false;
    bool invivo = false;
    bool is_synchronized = true;
    int idx = 0;
    int idx_prev = 0;
    int channel = 1;
    double threshold = 0.96;
    int run_number;
    std::string subject;

    VideoInput *cam;
    VideoInput *cam_usb;
    VideoInputStereo *stereo_cam;
    Segmentation *seg;
    StereoSegmentation* seg_stereo;
    StereoCalibration *calib;

    void startupCamera(int ch, float thres);
    void shutdownCamera();
    void setIdx(int idx);
    void setRadius(double radius);
    void set_lifetime(double val, int channel);
    void set_resolution(int w, int h);
    void set_mode(bool stereomode);
    void setAimingBeam(bool aiming_beam);
    void setInVivo(bool invivo);
    void load_calib();
    bool getFGReady();
    bool getUSBReady();
    bool getUSBOpen(int camID);
    bool getFGOpen(int camID);
    void startAcquisition();
    void captureFrame();
    void setAutoScale(bool autoscale);
    void setScale(double mn, double mx);
    void setAnsi(int ansi);
    void setFirefly(bool firefly);
    void setPentero(bool pentero_mode);

    void adjustArea(int event, int x, int y);
    static void adjustArea(int event, int x, int y, int flags, void* userdata);

    std::vector<double> log_pulse_max;
    std::vector<double> log_pulse_min;
    std::vector<double> log_pulse_thres;
    std::vector<double> log_pulse_cur;
    std::vector<double> timer_display;
    std::vector<double> timer_frames;

private:
    double ch1_tau;
    double ch2_tau;
    double ch3_tau;
    double ch4_tau;

    int width = 852;
    int height = 459;
    std::string infix;
    bool writing = false;
    bool firefly = false;
    bool pentero_mode = false;
    double scale_max = 2;
    double scale_min = 3;
    double radius = 1.0;
    bool scale_auto;
    int ansi = 999999;
    bool new_pos = true;
    int new_xpos = 1;
    int new_ypos = 1;

};

#endif // IMAGEACQUISITION_H
