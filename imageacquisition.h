#ifndef IMAGEACQUISITION_H
#define IMAGEACQUISITION_H

#include "videoinput.h"
#include "videopointgrey.h"
#include "opencv2/core.hpp"

using namespace cv;

class imageAcquisition
{
public:
    Mat frame;
    bool ctrl;
    bool thread;
    imageAcquisition();
    void startAcquisition();
};

#endif // IMAGEACQUISITION_H
