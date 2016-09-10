#include "imageacquisition.h"
#include "segmentation.h"
#include <QDebug>
#include <boost/thread.hpp>
#include "threadwrapper.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;

imageAcquisition::imageAcquisition()
{
    startupCamera(channel, threshold);
}

void imageAcquisition::startupCamera(int ch, float thres)
{
    // startup camera
   cam = new VideoPointGrey();

   channel = ch; // channel to be displayed
   threshold = thres; // cross-correlation threshold

   // check if at least one camera is connected
   if (cam->isConnected())
   {
       ready = true;
   }
}

void imageAcquisition::startAcquisition()
{


    // initialization
    frame = cam->getNextFrame();

    Segmentation *seg = new Segmentation(frame, Point(1,1), Point(frame.cols,frame.rows), false, channel, false);

    while (thread)
    {
        if (ctrl)
        {
            // get frame
            frame = cam->getNextFrame();

            // if in acquisition, do segmentation
            if (inAcquisition)
            {
                // set segmentation threshold
                seg->thres = threshold;

                // thread
                boost::thread segmentationThread(ThreadWrapper::startSegmentationThread, seg, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);

                // wait for thread to end
                segmentationThread.join();
            }

            // show frame
            imshow("Manual focus", frame);
            waitKey(10);
            if (!ctrl)
                destroyWindow("Manual focus");


        }
    }

    //destroyAllWindows();

    // cleanup
    cam->disconnect();
    delete(cam);
    delete(seg);

}

void imageAcquisition::shutdownCamera()
{
    cam->disconnect();
    delete(cam);

}

void imageAcquisition::set_lifetime(double val, int channel)
{
    switch(channel)
    {
    case 1:
        ch1_tau = val;
        break;
    case 2:
        ch2_tau = val;
        break;
    case 3:
        ch3_tau = val;
        break;
    case 4:
        ch4_tau = val;
    }
}
