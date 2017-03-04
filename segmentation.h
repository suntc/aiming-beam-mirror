#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "overlay.h"
#include "aimingbeam_segmentation.h"

class Segmentation
{

public:
    Segmentation(cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, bool autoscale, int ansi);
    Segmentation(cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, StereoCalibration * calib, bool autoscale);
    ~Segmentation();

    void startSegmentation(cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4, int idx);
    void setThreshold(double thres);
    void switchChannel(int channel);
    void setAutoScale(bool autoscale);
    void setColorScale(double mn, double mx);
    void setAnsi(int ansi);
    void setIdx(int idx);
    void setRadius(double radius);
    void setTimer(double timer);


    int last_radius;
    vector<int> log_coords_x;
    vector<int> log_coords_y;
    vector<int> log_radius;
    Overlay* ch1_overlay;
    Overlay* ch2_overlay;
    Overlay* ch3_overlay;
    Overlay* ch4_overlay;
    Overlay* overlay=0;

    vector<double> log_lt_ch1;
    vector<double> log_lt_ch2;
    vector<double> log_lt_ch3;
    vector<double> log_lt_ch4;
    vector<double> log_frame_no;
    vector<double> log_timer;

    int last_x;
    int last_y;
    int last_active;
    int idx = 0;
    double timer = 0;

    float thres;

    float correlateGaussian(cv::Mat frame, int &x, int &y, int &radius);
    float simpleThreshold(cv::Mat frame, int &x, int &y, int &radius);
    float doubleRingSegmentation(cv::Mat frame, int &x, int &y, int &radius);
    float pulsedSegmentation(cv::Mat frame_on, cv::Mat frame_off, cv::Rect corrArea,float &x, float &y, int &radius);
    void adjustArea(int x, int y);

    bool new_pos = false;
    int res_x;
    int res_y;
    cv::Mat firstFrame;

    int x0; int y0; int x1; int y1;
    int area_dim;

    cv::Point ROI_left_upper;
    cv::Point ROI_right_lower;

private:

    void init(cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ);

    bool firstFrameSet = false;

    bool scale_auto;
    double scale_min;
    double scale_max;

    int current_channel;

    double last_lt_ch1;
    double last_lt_ch2;
    double last_lt_ch3;
    double last_lt_ch4;

    int size_struct_elem;

    bool stereo_setup;

    int struct_size1;       // Size of structured element is 3
    int struct_size2;       // Size of structured element is 1

    cv::Mat frame1;
    cv::Mat frame2;

    double radius_factor = 1.0;

    StereoCalibration * calib;

};

#endif // SEGMENTATION_H
