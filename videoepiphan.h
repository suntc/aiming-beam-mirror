#ifndef VIDEOEPIPHAN_H
#define VIDEOEPIPHAN_H
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "videoinput.h"
#include "frmgrab.h"

typedef struct ContextOptions
{
    V2URect cropRect;           /* Crop rectangle */
    V2U_UINT32 captureFlags;    /* Capture format and flags */
    V2U_UINT32 frameCount;      /* Frame count */
    V2U_BOOL noStreaming;       /* Streaming flag */
    V2U_BOOL actionPerformed;   /* Something meaningful has been done */
} ContextOptions;

class VideoEpiphan : public VideoInput
{
public:
    VideoEpiphan();
    void setup();
    void convertFrame(V2U_GrabFrame2* fin, cv::Mat &fout);
    void start();
    void stop();
    void close();


    cv::Mat getNextFrame();
    void getNextStereoFrame(cv::Mat &f1, cv::Mat &f2);
    bool lastFrame();
    int getNumberOfFrames();
    void disconnect();
    void set_resolution(int w, int h);
    bool isStereoAvailable();
    bool isConnected(int camID);
private:
    cv::VideoCapture cap1;
    cv::VideoCapture cap2;
    int frameHeight = 720;  // default to match daVinci's 720p resolution
    int frameWidth = 1280;  // default to match daVinci's 720p resolution
    cv::Mat RGBimage;
    bool stereoAvailable;

    FrmGrabber* fg1; // leading frame grabber (left camera in stereo)
    FrmGrabber* fg2; // secondary frame grabber (right camera in stereo)

    const char* sn1 = "1";   // serial number of leading frame grabber
    const char* sn2 = "2";   // serial number of secondary frame grabber

    V2U_GrabFrame2* frame1;
    V2U_GrabFrame2* frame2;
    ContextOptions options;

    bool fg1_available = false;
    bool fg2_available = false;
};

#endif // VIDEOEPIPHAN_H

