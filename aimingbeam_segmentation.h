#ifndef AIMINGBEAM_SEGMENTATION_H
#define AIMINGBEAM_SEGMENTATION_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class AimingbeamSegmentation {
public:
    AimingbeamSegmentation(void) {};
    ~AimingbeamSegmentation(void) {};
    virtual void startSegmentation(cv::Mat frame, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4);
    virtual void startSegmentation(cv::Mat frame_l, cv::Mat frame_r, cv::Mat frame_vis, double lt_ch1, double lt_ch2, double lt_ch3, double lt_ch4);
    virtual void setThreshold(double thres);
};

#endif // AIMINGBEAM_SEGMENTATION_H
