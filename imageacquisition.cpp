#include "imageacquisition.h"
#include "segmentation.h"
#include "stereosegmentation.h"
#include <QDebug>
#include <boost/thread.hpp>
#include "threadwrapper.h"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "iopath.h"
#include "videowriter_ab.h"

using namespace cv;

imageAcquisition::imageAcquisition()
{
    startupCamera(channel, threshold);
}

void imageAcquisition::startupCamera(int ch, float thres)
{

    // startup camera
   //cam = new VideoPointGrey();
    stereo_cam = new VideoPointGreyStereo();

   channel = ch; // channel to be displayed
   threshold = thres; // cross-correlation threshold

   // check if at least one camera is connected
   if (stereo_cam->isConnected())
   {
       ready = true;
   }

}

void imageAcquisition::startAcquisition()
{

    // initialization
    frame = stereo_cam->getNextFrame(0);

    //Segmentation *seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, false);
    StereoCalibration *calib = new StereoCalibration("C:/Users/Marcu Lab/Documents/AimingBeamV2/build-AimingBeam-Desktop_Qt_5_3_MSVC2013_OpenGL_64bit-Release/release/stereo_calibration.yml");
    StereoSegmentation *seg  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);

    bool init_output = false;
    VideoOutput *avi_out_augmented;
    VideoOutput *avi_out_raw;


    while (thread)
    {

        if (ctrl)
        {
            if( init_output==false)
            {
                std::string infix = subject;
                infix.append("_run");
                infix.append(std::to_string(run_number));
                std::string filename = IOPath::getDataOutputFilename(infix,"avi");
                avi_out_raw = new VideoWriter_ab(filename, frame.cols, frame.rows);

                infix.append("_augmented");
                filename = IOPath::getDataOutputFilename(infix,"avi");
                avi_out_augmented = new VideoWriter_ab(filename, frame.cols, frame.rows);
                init_output = true;
            }

            // get frame
            frame = stereo_cam->getNextFrame(0);
            Mat frame_r = stereo_cam->getNextFrame(1);
            Mat frame_l = frame.clone();
            frame_l = calib->getRectifiedIm(frame_l,0);
            frame_r = calib->getRectifiedIm(frame_r,1);


            // if in acquisition, do segmentation
            if (inAcquisition)
            {
                // add raw frame to avi export
          //      avi_out_raw->addFrame(frame);

                // set segmentation threshold
                //seg->thres = threshold;

                // thread
                //boost::thread segmentationThread(ThreadWrapper::startSegmentationThread, seg, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                boost::thread segmentationThread(ThreadWrapper::startStereoSegmentationThread, seg, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);

                // wait for thread to end
                segmentationThread.join();

                // add segmented frame to avi export
           //     avi_out_augmented->addFrame(frame);
            }

            // show frame
            imshow("Manual focus", frame);

            waitKey(10);
            if (!ctrl)
            {
                destroyWindow("Manual focus");
                delete(seg);

                //seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, false);
                seg = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
                avi_out_augmented->closeFile();
                avi_out_raw->closeFile();
                init_output=false;
            }
        }

    }

    //destroyAllWindows();

    // cleanup
    avi_out_augmented->closeFile();
    delete(avi_out_augmented);
    avi_out_raw->closeFile();
    delete(avi_out_raw);
    //cam->disconnect();
    //delete(cam);
    stereo_cam->disconnect();
    delete(stereo_cam);
    delete(seg);

}

void imageAcquisition::shutdownCamera()
{
    stereo_cam->disconnect();
    //delete(cam);
    delete(stereo_cam);
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

void imageAcquisition::set_resolution(int w, int h)
{
    width = w;
    height = h;
    //stereo_cam->set_resolution(w, h);
}
