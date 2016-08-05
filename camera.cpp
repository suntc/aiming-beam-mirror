#include "camera.h"
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

camera::camera()
{


}

void camera::init()
{
    // connect to the camera and assign camera handle
    if(is_InitCamera(&hCam, NULL) == IS_SUCCESS)
    {
        isConnected = true;
    }
    else
    {
        isConnected = false;
    }
}

void camera::setup()
{

    qDebug() << "Setting up";

    is_AOI(hCam, IS_AOI_IMAGE_GET_AOI, (void*)&camAOI, sizeof(camAOI));

    // Get/Set image size and location from uEye SDK
    is_FreeImageMem(hCam, imgMem, memId);
    is_AllocImageMem(hCam, camAOI.s32Width, camAOI.s32Height, 24, &imgMem, &memId);
    is_SetImageMem(hCam, imgMem, memId);

    // Start Camera
    int sho = is_CaptureVideo(hCam, IS_WAIT);
    if(sho != IS_SUCCESS)
    {
        qDebug() << "Not captured";
    }
    else
    {
        qDebug() << "Captured";
    }
}

void camera::acquire()
{
    int c = -1;
    int i = 0;
    while (c == -1)
    {
        //int frameseq = is_CameraStatus(hCam, IS_SEQUENCE_CNT, IS_GET_STATUS);
        i++;
        qDebug() << i;
        // stops live mode or cancels a hardware triggered
        // image capture in case the exposure has not yet started
        is_StopLiveVideo(hCam, IS_WAIT);

        // releases memory from previous frame
        is_FreeImageMem(hCam, imgMem, memId);

        // allocate memory
        is_AllocImageMem(hCam, camAOI.s32Width, camAOI.s32Height, 24, &imgMem, &memId);

        // active memory
        is_SetImageMem(hCam, imgMem, memId);

        // transfer image to memory
        is_CaptureVideo(hCam, IS_WAIT);

        Mat frame;
        Mat retmat(camAOI.s32Height, camAOI.s32Width, CV_8UC3, imgMem);

        cvtColor(retmat, frame, CV_BGR2RGB);
        //retmat.copyTo(frame);

        // decrease image size if it initializes at high res
        if (frame.rows != 512)
        {
            resize(frame, frame, Size (640, 512));
        }

        imshow("Camera Preview", frame);

        frame.release();
        retmat.release();

        int c0 = waitKey(0);
        if (c == -1)
        {
            c = c0;
        }
    }

    closeConnection();
}

void camera::closeConnection()
{

    // Closing and cleaning of the camera
    is_FreeImageMem(hCam, imgMem, memId);

    // close connection
    is_ExitCamera(hCam);

    // close windows
    destroyAllWindows();

}

bool camera::is_connected()
{
    return isConnected;
}

camera::~camera()
{
    // disconnecting
    qDebug() << "Destructor";

    // closes connection to the camera
    int status = is_ExitCamera(hCam);

    qDebug() << status;
}
