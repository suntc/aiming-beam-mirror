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
        std::cout << "No camera detected" << std::endl;
        return;
    }

    error = busMgr.GetCameraFromSerialNumber(serial_top_cam,&guid1); //GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    qDebug() << "initialized";

    error = cam1.Connect(&guid1);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }

    error = cam1.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return;
    }
    //does not work
   // error = cam.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_800x600RGB, FlyCapture2::FRAMERATE_120 );
    //qDebug() << error.GetDescription();

    error = cam1.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        return;
    }

    connected = true;

    // startup second camera, if available
    if ( numCameras >= 2 )
    {
        stereoAvailable = true;
        busMgr.GetCameraFromSerialNumber(serial_side_cam,&guid2);
        cam2.Connect(&guid2);
        cam2.StartCapture();
    }
}

bool VideoPointGrey::isConnected()
{
    return connected;
}

cv::Mat VideoPointGrey::getNextFrame( )
{
    // Get the image
    error = cam1.RetrieveBuffer( &rawImage1 );

    if ( error != PGRERROR_OK )
    {
        cv::Mat m;
        return m.clone();
    }

    // convert to rgb
    rawImage1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );

    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();
    image = cv::Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes);

    return image.clone();
}

void VideoPointGrey::getNextStereoFrame(cv::Mat &f1, cv::Mat &f2)
{
    error = cam1.RetrieveBuffer( &rawImage1 );
    error = cam2.RetrieveBuffer( &rawImage2 );

    // convert to rgb
    rawImage1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );
    rawImage2.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2 );

    // convert to OpenCV Mat
    unsigned int rowBytes = (double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows();
    f1 = cv::Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes);
    f2 = cv::Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);

    return;
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
    cam1.Disconnect();
    if (stereoAvailable)
        cam2.Disconnect();

    connected = false;
}

void VideoPointGrey::set_resolution(int w, int h)
{
    width = w;
    height = h;
}

bool VideoPointGrey::isStereoAvailable()
{
    return stereoAvailable;
}

