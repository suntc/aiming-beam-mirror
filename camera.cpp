#include "camera.h"
#include <QDebug>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;

camera::camera()
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

    char* imgMem = 0;
    int memId = 0;

    // Load and Specify Image properties from uEye SDK
    //is_LoadParameters(hCam, "/cam/set1");
    IS_RECT camAOI;
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
        int c = -1;

        while(c == -1)
        {
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

            // check frame number
            //frameseq = is_CameraStatus(hCam, IS_SEQUENCE_CNT, IS_GET_STATUS);
            //framecounter = frameseq;

            Mat frame, frame0;
            Mat retmat(camAOI.s32Height, camAOI.s32Width, CV_8UC3, imgMem);

            cvtColor(retmat, frame, CV_BGR2RGB);

            frame.copyTo(frame0);
            if (frame0.rows != 512)
            {
                resize(frame0, frame0, Size (640, 512));
            }
            //cvtColor(frame, frame, CV_RGB2GRAY);
            imshow("Camera Focus Preview", frame0);

            double dblFPS;
            is_SetFrameRate(hCam, IS_GET_FRAMERATE, &dblFPS);
            qDebug() << dblFPS;

            c = waitKey(1);

            frame.release();
            frame0.release();
            retmat.release();
        }

    }

    // Closing and cleaning of the camera
    is_FreeImageMem(hCam, imgMem, memId);

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
