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
    //~Segmentation();

    void startSegmentation(cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4);
    void setThreshold(double thres);
    void switchChannel(int channel);
    void setAutoScale(bool autoscale);
    void setColorScale(double mn, double mx);
    void setAnsi(int ansi);

    int last_radius;
    vector<int> log_coords_x;
    vector<int> log_coords_y;
    vector<int> log_radius;
    Overlay* ch1_overlay;
    Overlay* ch2_overlay;
    Overlay* ch3_overlay;
    Overlay* ch4_overlay;
    Overlay* overlay=0;

    long frame_no=0;

    vector<double> log_lt_ch1;
    vector<double> log_lt_ch2;
    vector<double> log_lt_ch3;
    vector<double> log_lt_ch4;
    vector<double> log_frame_no;

    cv::Mat firstFrame;

    int last_x;
    int last_y;
    int last_active;


    float thres;

    cv::Mat* createFilter(int sigma);
    float correlateGaussian(cv::Mat frame, int &x, int &y, int &radius);
    float simpleThreshold(cv::Mat frame, int &x, int &y, int &radius);
    float doubleRingSegmentation(cv::Mat frame, int &x, int &y, int &radius);
    float pulsedSegmentation(cv::Mat frame_on, cv::Mat frame_off, cv::Rect corrArea,int &x, int &y, int &radius);
    int res_x;
    int res_y;

    int y0, y1, x0, x1;


    vector<cv::Mat> frames;
private:
   // const static int* radius_values; // only even!!
    cv::Point ROI_left_upper;
    cv::Point ROI_right_lower;
    void init(cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ);

    bool firstFrameSet = false;

    int *radius_values;
    static const int no_gaussians = 11; // 14;

    bool scale_auto;
    double scale_min;
    double scale_max;

    int current_channel;
    int log_coords_x_failed;
    int log_coords_y_failed;
    int log_radius_failed;
    vector<double> log_lifetime_failed;
    int num_of_int_steps;
    int last_found = 0;
    int last_vanish = 1;
    double last_lt_ch1;
    double last_lt_ch2;
    double last_lt_ch3;
    double last_lt_ch4;
    float thres_intensity;
    int area_dim;
    int area_dim_l;
    double getRadius(double cr[], int x[], int y[], int arrLength, int &x_out, int &y_out);
    bool segmentation_initialized=0;
    bool segmentation_selection=0;
    bool interpolation_successive_frames=0;
    int size_struct_elem;

    bool show_marker;
    int segmentation_mode;
    int segmentation_method;
    bool subtract_first_frame;
    bool stereo_setup;

    int struct_size1;       // Size of structured element is 3
    int struct_size2;       // Size of structured element is 1

    cv::Mat frame1;
    cv::Mat frame2;

    cv::Mat* gaussians[no_gaussians];

    StereoCalibration * calib;
};

#endif // SEGMENTATION_H
