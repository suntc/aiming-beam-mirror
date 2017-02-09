#include "videopointgreystereo.h"
#include <iostream>
#include "FlyCapture2.h"
#include <QDebug>

using namespace FlyCapture2;
using namespace std;



VideoPointGreyStereo::VideoPointGreyStereo()
{
    ready=true;

    // Connect to a camera
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        ready=false;
        return;
    }

    if ( numCameras < 1 )
    {
        cout << "No camera detected." << endl;
        ready=false;
        return;
    }
    else if ( numCameras < 2)
    {
        cout << "Only one camera detected." << endl;
        ready=false;
        return;
    }

    //error = busMgr.GetCameraFromIndex(0, &guid1);
    error = busMgr.GetCameraFromSerialNumber(serial_top_cam,&guid1);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        ready=false;
        return;
    }

    //error = busMgr.GetCameraFromIndex(1, &guid2);
    error = busMgr.GetCameraFromSerialNumber(serial_side_cam,&guid2);
    error = cam2.Connect(&guid2);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        ready=false;
        return;
    }

    error = cam1.Connect(&guid1);

    if (error != PGRERROR_OK)
    {
        PrintError(error);
        ready=false;
        return;
    }

    error = cam1.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        ready=false;
        return;
    }

    error = cam1.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        ready=false;
        return;
    }
    error = cam2.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        std::cout << "Bandwidth exceeded" << std::endl;
        ready=false;
        return;
    }
    //cam1.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_800x600RGB, FlyCapture2::FRAMERATE_60 );
    //error = cam2.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_800x600RGB, FlyCapture2::FRAMERATE_30 );
    //if (error != FlyCapture2::PGRERROR_OK)
    //            {
    //                error.PrintErrorTrace();
    //                exit(-1);
    //            }

    //cam1.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960RGB, FlyCapture2::FRAMERATE_30 );
    //cam2.SetVideoModeAndFrameRate(FlyCapture2::VIDEOMODE_1280x960RGB, FlyCapture2::FRAMERATE_30 );
    connected = true;
}

void VideoPointGreyStereo::PrintError( Error error )
{
    error.PrintErrorTrace();
}

void VideoPointGreyStereo::getNextStereoFrame( cv::Mat &f1, cv::Mat &f2)
{
    // Get the image
    error = cam1.RetrieveBuffer( &rawImage1 );
    error = cam2.RetrieveBuffer( &rawImage2 );

    if ( error != PGRERROR_OK )
    {
        std::cout << "capture error" << std::endl;
        //exit(-1);
    }
    //CameraInfo *ci;
    //cam1.GetCameraInfo(ci);
    //uint serial = ci->serialNumber;
    //qDebug()<<ci->serialNumber;
    //cam2.GetCameraInfo(ci);
    //serial = ci->serialNumber;
    //qDebug()<<ci->serialNumber;
    //exit(0);

    // convert to rgb
    rawImage1.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage1 );
    rawImage2.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage2 );
    // convert to OpenCV Mat
    unsigned int rowBytes = (unsigned int) ((double)rgbImage1.GetReceivedDataSize()/(double)rgbImage1.GetRows());
    f1 = cv::Mat(rgbImage1.GetRows(), rgbImage1.GetCols(), CV_8UC3, rgbImage1.GetData(),rowBytes);
    f2 = cv::Mat(rgbImage2.GetRows(), rgbImage2.GetCols(), CV_8UC3, rgbImage2.GetData(),rowBytes);
    //cv::resize(im_copy,im_copy,cv::Size(640,480));
    return;
}


bool VideoPointGreyStereo::lastFrame()
{
    return false;
}

int VideoPointGreyStereo::getNumberOfFrames()
{
    return 0;
}

void VideoPointGreyStereo::disconnect()
{
    cam1.Disconnect();
    cam2.Disconnect();
}

bool VideoPointGreyStereo::isConnected(int camID)
{
    return connected;
}

bool VideoPointGreyStereo::isReady()
{
    return ready;
}
