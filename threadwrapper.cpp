#include "threadwrapper.h"
#include "segmentation.h"
#include "tcp_ip.h"
#include "imageacquisition.h"
#include <boost/thread.hpp>

void ThreadWrapper::startSegmentationThread(Segmentation *seg, cv::Mat frame, double lt1, double lt2, double lt3, double lt4)
{

    seg->startSegmentation(frame, lt1, lt2, lt3, lt4);

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
