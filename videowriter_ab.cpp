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
    //outputVideo.open(file1, -1 , 30 , cv::Size(852, 478)); //8.77678e+08 VideoWriter::fourcc('M','J','P','G')
    qDebug() << filename.c_str();
    outputVideo.open(filename, CV_FOURCC('F', 'M', 'P', '4') , 18 , cv::Size(dim_x, dim_y), true);
    qDebug() << filename.c_str();
    if (!outputVideo.isOpened())
      {
          exit(-1);
      }

}
void VideoWriter_ab::addFrame(cv::Mat frame)
{
    //outputVideo.write(frame);
    outputVideo << frame;
    //vid_seq.push_back(frame.clone());
    return;
}

int VideoWriter_ab::getNumberOfFrames()
{
    return 0;
}

void VideoWriter_ab::closeFile()
{



    //for (vector<Mat>::iterator it = vid_seq.begin() ; it != vid_seq.end(); ++it)
    //{
    //    outputVideo << *it;
    //}
    outputVideo.release();
}
