#include "videopointgrey.h"
#include "FlyCapture2.h"
#include <iostream>
#include <QDebug>
#include "errorhandler.h"

using namespace FlyCapture2;
using namespace std;

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

// ToDo: Identify camera by serial number
VideoPointGrey::VideoPointGrey()
{

    // Connect to a camera
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        //PrintError(error);
        return;
        //exit(-1); //Replace the exit commands
    }

    if ( numCameras < 1 )
    {
        //cout << "No camera detected." << endl;
        return;
    }

    error = busMgr.GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }
    error = cam.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return;
    }

    cam.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960RGB, FlyCapture2::FRAMERATE_120 );

    connected = true;

}

bool VideoPointGrey::isConnected()
{
    return connected;
}

cv::Mat VideoPointGrey::getNextFrame( )
{
    // Get the image
    error = cam.RetrieveBuffer( &rawImage );
    //cam.SetTriggerMode();

    if ( error != PGRERROR_OK )
    {
        //std::cout << "capture error" << std::endl;
        qDebug() << "should never go here";
        exit(-1);
    }

    // convert to rgb
    rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );

    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();
    cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);

    //qDebug() << "image pixel";
    //qDebug() << image.at<cv::Vec3b>(1000, 1200)[0];

    //qDebug() << width;
    //qDebug() << height;
    //cv::Mat image = cv::Mat(height, width, CV_8UC3, rgbImage.GetData(),rowBytes);

    return image.clone();
}

bool VideoPointGrey::lastFrame()
{
    return false;
}

int VideoPointGrey::getNumberOfFrames()
{
    return 0;
}

void VideoPointGrey::disconnect()
{
    cam.Disconnect();
    connected = false;
}

void VideoPointGrey::set_resolution(int w, int h)
{
    width = w;
    height = h;
}

