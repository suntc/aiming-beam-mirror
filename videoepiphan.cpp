#include "videoepiphan.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMessageBox>
#include <QDebug>

using namespace cv;

VideoEpiphan::VideoEpiphan()
{
    bool found=0;
    try
    {
        cap1.open(0);
        qDebug() << "frame grabber 1 OK";
    }
    catch(...)
    {
        qDebug() << "no frame grabber";
    }
    stereoAvailable = true;
    try
    {
        cap2.open(1);
        qDebug() << "frame grabber 2 OK";
    }
    catch(...)
    {
        qDebug() << "no second frame grabber";
        stereoAvailable = false;
    }

    cap1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    if(stereoAvailable)
    {
        cap2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
        cap2.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    }

    if(!cap1.isOpened()) // If camera cannot be opened display message and return
    {
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Camera initialization failed!");
        messageBox.setFixedSize(500,200);
    }
    else
    {
        frameHeight = cap1.get(CV_CAP_PROP_FRAME_HEIGHT); // Height in pixels of the frame
        frameWidth = cap1.get(CV_CAP_PROP_FRAME_WIDTH); // Width in pixels of the frame

        RGBimage.create(frameWidth, frameHeight, CV_8UC3);

        //if(cap2.isOpened())
        //    stereoAvailable = true;
        //else
        //    stereoAvailable = false;

    }




}

Mat VideoEpiphan::getNextFrame()
{
    // Acquire image
    cap1 >> RGBimage;
    //qDebug() << RGBimage.cols;
    //qDebug() << RGBimage.rows;
    //imshow("image", RGBimage);
    //resize(RGBimage, RGBimage, Size (1280,720)); //frame_cols, frame_rows));
    return RGBimage;
}

void VideoEpiphan::getNextStereoFrame(Mat &f1, Mat &f2)
{
    cap1 >> f1;
    cap2 >> f2;
    return;
}

bool VideoEpiphan::lastFrame()
{
    return false;
}

int VideoEpiphan::getNumberOfFrames()
{
    return 0;
}

void VideoEpiphan::disconnect()
{
    cap1.release();
    if (stereoAvailable)
        cap2.release();
}

void VideoEpiphan::set_resolution(int w, int h)
{
    return;
}

bool VideoEpiphan::isConnected()
{
    return cap1.isOpened();
}

bool VideoEpiphan::isStereoAvailable()
{
    return stereoAvailable;
}
