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
    Segmentation *seg = new Segmentation(frame, Point(1,1), Point(400,400), false, channel, false);

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
                boost::thread segmentationThread(ThreadWrapper::startSegmentationThread, seg, frame, 1., 1., 1., 1.);

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
