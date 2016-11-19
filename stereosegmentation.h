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
    StereoSegmentation(StereoCalibration * sc, cv::Mat frame, cv::Point point1, cv::Point point2, bool interp, int ch_number, bool interp_succ);
    ~StereoSegmentation();

    void startSegmentation(cv::Mat frame_l, cv::Mat frame_r, cv::Mat frame_vis, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4);
    void setThreshold(double thres);
    void switchChannel(int channel);

    Segmentation* seg;
    Overlay* height_profile = NULL;
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

private:
    int disparity_range = 700; // to be adapted to configuration and resolution
    StereoCalibration* calib;
    bool firstFrameSet;
    int current_channel=1;
};

#endif // STEREOSEGMENTATION_H
