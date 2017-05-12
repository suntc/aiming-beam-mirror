#ifndef THREADWRAPPER_H
#define THREADWRAPPER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "segmentation.h"
#include "stereosegmentation.h"
#include "tcp_ip.h"
#include "imageacquisition.h"
#include "acousticfeedback.h"


class ThreadWrapper
{
public:
    static void startSegmentationThread(Segmentation *seg, cv::Mat frame, cv::Mat frame_on,  cv::Mat frame_off, double lt1, double lt2, double lt3, double lt4, int idx);
    static void startStereoSegmentationThread(StereoSegmentation *seg, cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, cv::Mat frame_on2, cv::Mat frame_off2, std::pair<double,double> lt12, std::pair<double,double> lt34, int idx);
    static void StartRead(TCP_IP &obj, string *output, int *len, vector<double> *data);
    static void StartAcquire(imageAcquisition *acq);
    static void StartFrameCapture(imageAcquisition *acq);
    static void startFeedback(AcousticFeedback *ac);
    static void beep();
};

#endif // THREADWRAPPER_H
