#ifndef SEGMENTATION_H
#define SEGMENTATION_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "overlay.h"

class Segmentation
{

public:
    Segmentation(cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ, bool stero);
    ~Segmentation();
    void startSegmentation(cv::Mat frame, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4);

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

    cv::Mat firstFrame;

    int last_x;
    int last_y;
    int last_active;

    float thres;

    cv::Mat* createFilter(int sigma);
    float correlateGaussian(cv::Mat frame, int &x, int &y, int &radius);
    float simpleThreshold(cv::Mat frame, int &x, int &y, int &radius);
    int res_x;
    int res_y;
private:
   // const static int* radius_values; // only even!!
    bool firstFrameSet = false;


    int *radius_values;
    static const int no_gaussians = 11; // 14;

    cv::Point ROI_left_upper;
    cv::Point ROI_right_lower;
    int current_channel;
    int log_coords_x_failed;
    int log_coords_y_failed;
    int log_radius_failed;
    vector<double> log_lifetime_failed;
    int num_of_int_steps;
    int last_found;
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

    cv::Mat* gaussians[no_gaussians];
};

#endif // SEGMENTATION_H
