#ifndef VIDEOWRITER_H
#define VIDEOWRITER_H
#include "videooutput.h"
#include <iostream>
#include <string>

class VideoWriter_ab : public VideoOutput
{
public:
    VideoWriter_ab();
    ~VideoWriter_ab();
    VideoWriter_ab(std::string filename,int dim_x,int dim_y);
    void addFrame(cv::Mat frame);
    int getNumberOfFrames();
    void closeFile();
private:
    cv::VideoWriter outputVideo;
    std::vector<cv::Mat> vid_seq;
};

#endif // VIDEOWRITER_H
