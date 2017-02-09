#include "videoepiphan.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QMessageBox>
#include <QDebug>

using namespace cv;

VideoEpiphan::VideoEpiphan()
{
    // assume there is no stereo
    stereoAvailable = false;

    try
    {
        // try to open first frame grabber
        cap1.open(0);

        if (cap1.isOpened())
        {
            // set resolution first frame grabber
            cap1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
            cap1.set(CV_CAP_PROP_FRAME_HEIGHT,720);

            // get resolution
            frameHeight = cap1.get(CV_CAP_PROP_FRAME_HEIGHT); // Height in pixels of the frame
            frameWidth = cap1.get(CV_CAP_PROP_FRAME_WIDTH); // Width in pixels of the frame

            RGBimage.create(frameWidth, frameHeight, CV_8UC3);
        }
    }
    catch(...)
    {
        return;
    }

    // check second frame grabber
    try
    {
        // open second frame grabber
        cap2.open(1);

        if (cap2.isOpened())
        {
            stereoAvailable = true;

            // set resolution
            cap2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
            cap2.set(CV_CAP_PROP_FRAME_HEIGHT,720);
        }
    }
    catch(...)
    {
        return;

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

bool VideoEpiphan::isConnected(int camID)
{
    bool isOpen = false;
    switch (camID)
    {
    case 0: // first camera
        isOpen = cap1.isOpened();
        break;
    case 1: // second camera
        isOpen = cap2.isOpened();
    }

    return isOpen;
}

bool VideoEpiphan::isStereoAvailable()
{
    return stereoAvailable;
}
