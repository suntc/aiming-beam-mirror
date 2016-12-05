#include "videoepiphan.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <QMessageBox>
#include <QDebug>

using namespace cv;

VideoEpiphan::VideoEpiphan()
{
    bool found=0;
    try
    {
        cap.open(0);
        qDebug() << "frame grabber OK";
    }
    catch(...)
    {
        qDebug() << "no frame grabber";
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    //if (found==0)
    //{
        //QMessageBox messageBox;
        //messageBox.critical(0, "Error", "Framegrabber not found!");
        //messageBox.setFixedSize(500,200);
    //}

    if(!cap.isOpened()) // If camera cannot be opened display message and return
    {
        qDebug() << "not well";
        //QMessageBox messageBox;
        //messageBox.critical(0, "Error", "Camera initialization failed!");
        //messageBox.setFixedSize(500,200);
    }
    else
    {
        frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); // Height in pixels of the frame
        frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); // Width in pixels of the frame
        //qDebug() << frameHeight;
        //qDebug() << frameWidth;
        RGBimage.create(frameWidth, frameHeight, CV_8UC3);
        //cap.set(CV_CAP_PROP_FRAME_WIDTH,1280);
        //cap.set(CV_CAP_PROP_FRAME_HEIGHT,720);

    }




}

Mat VideoEpiphan::getNextFrame()
{
    // Acquire image
    cap >> RGBimage;
    //imshow("image", RGBimage);
    //resize(RGBimage, RGBimage, Size (1280,720)); //frame_cols, frame_rows));
    return RGBimage;
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
    cap.release();
}

void VideoEpiphan::set_resolution(int w, int h)
{
    return;
}

bool VideoEpiphan::isConnected()
{
    return cap.isOpened();
}
