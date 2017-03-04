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

VideoPointGrey::VideoPointGrey()
{

    // get number of cameras
    status = busMgr.GetNumOfCameras(&numCameras);

    // check number of cameras to verify stereomode
    if (numCameras == 0)
    {
        connected = false;
    }
    else
    {
        connected = true;
        if (numCameras == 2)    stereoAvailable = true;

        // initialize variables to set up resolution
        // set resolution
        Format7ImageSettings fmt7ImageSettings;
        fmt7ImageSettings.mode = MODE_0;
        fmt7ImageSettings.offsetX = 0;
        fmt7ImageSettings.offsetY = 0;
        fmt7ImageSettings.width = 1280;
        fmt7ImageSettings.height = 720;
        fmt7ImageSettings.pixelFormat = PIXEL_FORMAT_MONO8; //PIXEL_FORMAT_RGB

        bool valid;
        Format7PacketInfo fmt7PacketInfo;

        // get leading (top) camera by SN
        status = busMgr.GetCameraFromSerialNumber(serial_top_cam, &guid1);
        if (status != PGRERROR_OK)
        {
            // camera not available
            cam1_available = false;
        }
        else
        {
            cam1_available = true;
            // connect to camera and initialize capture
            cam1.Connect(&guid1);

            // Validate settings
            status = cam1.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);

            // Set the settings to the camera
            status = cam1.SetFormat7Configuration(&fmt7ImageSettings,fmt7PacketInfo.recommendedBytesPerPacket );

            // Start capturing images
            cam1.StartCapture();

        }

        // get side camera by SN
        status = busMgr.GetCameraFromSerialNumber(serial_side_cam,&guid2);
        if (status != PGRERROR_OK)
        {
            // camera not available
            cam2_available = false;
        }
        else
        {
            cam2_available = true;
            // connect to camera and initialize capture
            cam2.Connect(&guid2);

            // Validate settings
            status = cam2.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);

            // Set the settings to the camera
            status = cam2.SetFormat7Configuration(&fmt7ImageSettings,fmt7PacketInfo.recommendedBytesPerPacket );

            cam2.StartCapture();
        }
    }
}

/*
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
        //std::cout << "No camera detected" << std::endl;
        return;
    }

    error = busMgr.GetCameraFromSerialNumber(serial_top_cam,&guid1); //GetCameraFromIndex(0, &guid);
    if (error != PGRERROR_OK)
    {
        //PrintError(error);
        return;
    }

    //qDebug() << "initialized";

    error = cam1.Connect(&guid1);
    if (error != PGRERROR_OK)
    {
        //PrintError(error);
        return;
    }

    error = cam1.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        //PrintError(error);
        return;
    }

    error = cam1.StartCapture();
    if ( error == PGRERROR_ISOCH_BANDWIDTH_EXCEEDED )
    {
        //std::cout << "Bandwidth exceeded" << std::endl;
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
*/
/*
bool VideoPointGrey::isConnected(int camID)
{
    return connected;
}
*/

bool VideoPointGrey::isConnected(int camID)
{
    bool isOpen = false;

    switch (camID)
    {
    case 0: // top camera
        isOpen = cam1_available;
        break;
    case 1: // side camera
        isOpen = cam2_available;
        break;
    default:
        isOpen = false;
    }

    return isOpen;
}

cv::Mat VideoPointGrey::getNextFrame( )
{
    // Get the image
    status = cam1.RetrieveBuffer( &rawImage1 );

    if ( status != PGRERROR_OK )
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
    status = cam1.RetrieveBuffer( &rawImage1 );
    status = cam2.RetrieveBuffer( &rawImage2 );

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

