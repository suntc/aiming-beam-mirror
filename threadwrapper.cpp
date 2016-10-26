#include "threadwrapper.h"
#include "segmentation.h"
#include "stereosegmentation.h"
#include "tcp_ip.h"
#include "imageacquisition.h"
#include <boost/thread.hpp>

void ThreadWrapper::startSegmentationThread(Segmentation *seg, cv::Mat frame, cv::Mat frame_on, cv::Mat frame_off, double lt1, double lt2, double lt3, double lt4)
{

    seg->startSegmentation(frame, frame_on, frame_off, lt1, lt2, lt3, lt4);

}

void ThreadWrapper::startStereoSegmentationThread(StereoSegmentation *seg, cv::Mat frame_l, cv::Mat frame_r, cv::Mat frame_vis, double lt1, double lt2, double lt3, double lt4)
{

    seg->startSegmentation(frame_l, frame_r, frame_vis, lt1, lt2, lt3, lt4);

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
