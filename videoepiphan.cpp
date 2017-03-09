#include "videoepiphan.h"

#include <QDebug>

using namespace cv;


VideoEpiphan::VideoEpiphan()
{
    // initialization
    fg1 = NULL;
    fg2 = NULL;

    // assume there is no stereo
    stereoAvailable = false;

    // define resolution. Not really sure if necessary
    options.captureFlags = V2U_GRABFRAME_FORMAT_BGR24;
    options.cropRect.x = 0;
    options.cropRect.y = 0;
    options.cropRect.width = frameWidth;
    options.cropRect.height = frameHeight;

    // initialization
    FrmGrab_Init();

    setup();

}

void VideoEpiphan::setup()
{
    // open leading frame grabber - left camera stereo
    fg1 = FrmGrabLocal_OpenSN(sn1);

    // leave the following code as is, open to implement new features as setting resolution and fps

    if (fg1 != NULL)    // fg1 is available
    {
        fg1_available = true;
    }
    else
    {
        fg1_available = false;
    }

    // connect to second frame grabber - right camera stereo
    fg2 = FrmGrabLocal_OpenSN(sn2);

    if (fg2 != NULL)
    {
        fg2_available = true;
    }
    else
    {
        fg2_available = false;
    }

    // check stereo availability
    stereoAvailable = (fg1_available && fg2_available) ? true : false;


}

Mat VideoEpiphan::getNextFrame()
{
    // capture 1 frame. Set last option to NULL to capture entire frame i.e. no cropping
    frame1 = FrmGrab_Frame(fg1, options.captureFlags, NULL);

    // convert to opencv format
    convertFrame(frame1, RGBimage);

    // release frame
    FrmGrab_Release(fg1, frame1);

    return RGBimage;
}


void VideoEpiphan::getNextStereoFrame(Mat &f1, Mat &f2)
{
    // capture from both frame grabbers
    frame1 = FrmGrab_Frame(fg1, options.captureFlags, NULL);
    frame2 = FrmGrab_Frame(fg2, options.captureFlags, NULL);

    // convert top open cv format
    convertFrame(frame1, f1);
    convertFrame(frame2, f2);

    // release frame
    FrmGrab_Release(fg1, frame1);
    FrmGrab_Release(fg2, frame2);

}

void VideoEpiphan::convertFrame(V2U_GrabFrame2* fin, Mat &fout)
{
    if (fin)
    {
        int h = fin->mode.height;
        int w = fin->mode.width;

        fout = Mat(h, w, CV_8UC3,(uchar*)fin->pixbuf, 3*w);
    }
}

void VideoEpiphan::start()
{
    // signal the frame grabber to prepare for capturing frames. helps to get max fps
    FrmGrab_Start(fg1);
    if (stereoAvailable)
        FrmGrab_Start(fg2);

}

void VideoEpiphan::stop()
{
    // signals the frame grabber to stop capturing
    FrmGrab_Stop(fg1);
    if (stereoAvailable)
        FrmGrab_Stop(fg2);
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
    // disconnect frame grabbers & release memory
    FrmGrab_Close(fg1);

    if (stereoAvailable)
    {
        FrmGrab_Close(fg2);
    }

    FrmGrab_Deinit();
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
        isOpen = fg1_available;
        break;
    case 1: // second camera
        isOpen = fg2_available;
    }

    return isOpen;
}

bool VideoEpiphan::isStereoAvailable()
{
    return stereoAvailable;
}
