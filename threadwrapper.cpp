#include "threadwrapper.h"
#include "segmentation.h"
#include "stereosegmentation.h"
#include "tcp_ip.h"
#include "imageacquisition.h"
#include <boost/thread.hpp>
#include <QDebug>

void ThreadWrapper::startSegmentationThread(Segmentation *seg, cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, double lt1, double lt2, double lt3, double lt4, int idx)
{
    seg->startSegmentation(frame, frame_on, frame_off, lt1, lt2, lt3, lt4, idx);
}

void ThreadWrapper::startStereoSegmentationThread(StereoSegmentation *seg, cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, cv::Mat frame_on2, cv::Mat frame_off2, std::pair<double,double> lt12, std::pair<double,double> lt34, int idx)
{
    double lt1 = lt12.first;
    double lt2 = lt12.second;
    double lt3 = lt34.first;
    double lt4 = lt34.second;
    seg->startSegmentation(frame, frame_on, frame_off, frame_on2, frame_off2, lt1, lt2, lt3, lt4, idx);
}


void ThreadWrapper::StartRead(TCP_IP &obj, string *output, int *len, vector<double> *data)
{
    if (obj.isOpen())
    {
        obj.read(output, len, data);
    }
}

void ThreadWrapper::StartAcquire(imageAcquisition *acq)
{
    acq->startAcquisition();
}

void ThreadWrapper::StartFrameCapture(imageAcquisition *acq)
{
    acq->captureFrame();
}
