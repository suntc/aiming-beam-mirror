#include "videowriter_ab.h"
#include <iostream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <QDebug>

using namespace std;
using namespace cv;
VideoWriter_ab::VideoWriter_ab() {}

VideoWriter_ab::VideoWriter_ab(string filename,int dim_x,int dim_y)
{
    outputVideo.open(filename, CV_FOURCC('F', 'M', 'P', '4') , 18 , cv::Size(dim_x, dim_y), true);
    if (!outputVideo.isOpened())
    {
        //exit(-1);
    }

}
void VideoWriter_ab::addFrame(cv::Mat frame)
{
    outputVideo << frame;
    return;
}

int VideoWriter_ab::getNumberOfFrames()
{
    return 0;
}

void VideoWriter_ab::closeFile()
{
    outputVideo.release();
}

VideoWriter_ab::~VideoWriter_ab()
{
    outputVideo.release();
}

