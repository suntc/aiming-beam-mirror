#ifndef STEREOSEGMENTATION_H
#define STEREOSEGMENTATION_H
#include "segmentation.h"
#include "stereocalibration.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "overlay.h"
#include "aimingbeam_segmentation.h"

class StereoSegmentation
{
public:
    StereoSegmentation(StereoCalibration * sc, cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, bool autoscale, int ansi);
    ~StereoSegmentation();

    void startSegmentation(cv::Mat frame_vis, cv::Mat frame_on, cv::Mat frame_off, cv::Mat frame_on2, cv::Mat frame_off2, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4, int idx);
    void setThreshold(double thres);
    void switchChannel(int channel);
    void setAnsi(int ansi);
    void setColorScale(double mn, double mx);
    void setAutoScale(bool autoscale);
    void set_synchronized(bool is_synchronized);

    Segmentation* seg;
    Overlay* height_profile = 0;
    vector<double> log_height;
    vector<double> log_real_x;
    vector<double> log_real_y;

    Overlay* ch1_overlay;
    Overlay* ch2_overlay;
    Overlay* ch3_overlay;
    Overlay* ch4_overlay;
    Overlay* overlay=0;

    cv::Mat firstFrame;

    int res_x;
    int res_y;
    vector<int> log_coords_x;
    vector<int> log_coords_y;
    vector<int> log_radius;
    vector<int> log_synchronized;
    vector<double> log_disparity_y;

    int x0;
    int y0;
    int x1;
    int y1;

private:
    int disparity_range = 200; // (def 700) to be adapted to configuration and resolution
    StereoCalibration* calib;
    bool firstFrameSet;
    int current_channel=1;
    bool scale_auto;
    bool is_synchronized=false;

    int radius;
    float x,y;
};

#endif // STEREOSEGMENTATION_H
