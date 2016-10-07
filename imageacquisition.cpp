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
#include "IOTxtData.h"

using namespace cv;

imageAcquisition::imageAcquisition(bool stereomode)
{
    this->stereomode = stereomode;
    startupCamera(channel, threshold);
}

void imageAcquisition::startupCamera(int ch, float thres)
{
    // startup one or two cameras
    // and check if at least one camera is connected

    if (stereomode==false)
    {
        cam = new VideoPointGrey();
        if (cam->isConnected())
            ready = true;
    }
    else
    {
        stereo_cam = new VideoPointGreyStereo();
        if (stereo_cam->isConnected())
            ready = true;
    }
    channel = ch; // channel to be displayed
    threshold = thres; // cross-correlation threshold
}

void imageAcquisition::startAcquisition()
{
    // initialization
    //frame = stereo_cam->getNextFrame(0);

    if (!stereomode)
    {
        frame = cam->getNextFrame();
        qDebug() << "invoke mono";
        //seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
    }
    else
    {
        // to be changed XXX
        frame = stereo_cam->getNextFrame(0);
        //calib = new StereoCalibration("C:/Aiming Beam v2/Source/Calibration/stereo_calibration.yml");
        qDebug() << "invoke stereo";
        //seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
    }

    bool init_output = false;
    VideoOutput *avi_out_augmented;
    VideoOutput *avi_out_raw;

    while (thread)
    {
        if (ctrl)
        {
            if (!seg && !stereomode)
            {
                qDebug() << "build monoseg";
                seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
            }
            if (!seg_stereo && stereomode)
            {
                qDebug() << "build stereoseg";
                seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
            }
            // this should not be available for manual focus operation

            //if( init_output==false)
            if (init_output==false)
            {
                infix = subject;
                infix.append("_run");
                infix.append(std::to_string(run_number));
                std::string filename = IOPath::getDataOutputFilename(infix,"avi","videos");

                avi_out_raw = new VideoWriter_ab(filename, frame.cols, frame.rows);

                string infix0 = infix;
                infix0.append("_augmented");
                filename = IOPath::getDataOutputFilename(infix0,"avi","videos");
                avi_out_augmented = new VideoWriter_ab(filename, frame.cols, frame.rows);
                init_output = true;
            }



            // stereo pair
            Mat frame_r;
            Mat frame_l;

            // get frame
            if (!stereomode)
                frame = cam->getNextFrame();
            else
            {
                frame = stereo_cam->getNextFrame(0);

                frame_r = stereo_cam->getNextFrame(1);
                frame_l = frame.clone();

                // if stereo camera pair is used, rectify images
                frame_l = calib->getRectifiedIm(frame_l,0);
                frame_r = calib->getRectifiedIm(frame_r,1);
            }


            // if in acquisition, do segmentation
            if (inAcquisition)
            {
                // add raw frame to avi export
                avi_out_raw->addFrame(frame);
                boost::thread* segmentationThread;


                if (!stereomode)
                {
                    seg->setThreshold(threshold);
                    //boost::thread segmentationThread(ThreadWrapper::startSegmentationThread, seg, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                    //segmentationThread = new boost::thread;
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startSegmentationThread, seg, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau));
                    // wait for thread to end


                }
                else
                {

                    seg_stereo->setThreshold(threshold);
                    //boost::thread segmentationThread(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau));
                    // wait for thread to end
                    //segmentationThread.join();

                }
                // thread
                segmentationThread->join();

                // add segmented frame to avi export
                avi_out_augmented->addFrame(frame);
            }
            // show frame
            imshow("Acquisition", frame);

            int k=waitKey(10);
            if (k>=49 && k<=52 && !stereomode)
                seg->switchChannel(k-48);
            if (k>=48 && k<=52 && stereomode)
                seg_stereo->switchChannel(k-48);

            if (!ctrl)
            {

                destroyWindow("Acquisition");




                init_output=false;

                if (!stereomode)
                {

                    delete(seg);
                    qDebug() << "invoke mono 2";
                    seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
                    // write to textfile missing
                }
                else
                {
                    //std::string infix = subject;
                    //infix.append("_run");
                    //infix.append(std::to_string(run_number));
                    std::string filename = IOPath::getDataOutputFilename(infix,"txt","txt");
                    IOTxtData::writeTxtFile(filename, seg_stereo );

                    //filename = IOPath::getDataOutputFilename(infix,"jpg","figures");
                    qDebug() << "before jpg";
                    for (int i=0; i<5; i++)
                    {
                        string infix0;
                        if (i==0)
                        {
                            infix0 = infix;
                            infix0 = infix0.append("_profile");
                        }
                        else
                        {
                            infix0 = infix;
                            infix0 = infix0.append("_CH");
                            infix0 = infix0.append(to_string(i));
                        }

                        filename = IOPath::getDataOutputFilename(infix0,"jpg","figures");
                        IOTxtData::writeJpgFile_stereo(filename,seg_stereo,i);
                    }
                    qDebug() << "after jpg";
                    delete(seg_stereo);
                    qDebug() << "invoke stereo 2";
                    seg_stereo = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
                }
                // need to handle this properly for manual focussing

                avi_out_augmented->closeFile();
                avi_out_raw->closeFile();
            }
        }

    }

    //destroyAllWindows();

    // cleanup

    // need to handle this properly for manual focussing
    if (true)
    {
        avi_out_augmented->closeFile();
        delete(avi_out_augmented);
        avi_out_raw->closeFile();
        delete(avi_out_raw);

    }


    if (!stereomode)
    {
        cam->disconnect();
        delete(cam);
        delete(seg);
    }
    else
    {

        stereo_cam->disconnect();
        delete(stereo_cam);
        delete(seg_stereo);
    }



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

void imageAcquisition::set_mode(bool stereomode)
{
    this->stereomode = stereomode;
    startupCamera(channel, threshold);

    if (stereomode)
    {
        if (!calib)
        {
            // initialize calibration
            string filename = IOPath::getAppDir();
            filename.append("Calibration\\stereocalibration");
            string counter = IOPath::getCurrentCounter();
            filename.append(counter);
            filename.append(".yml");
            qDebug() << "using calibration file:";
            qDebug() << filename.c_str();
            calib = new StereoCalibration(filename);
        }

        if (!seg_stereo)
        {
            // initialize segmentation
            frame = stereo_cam->getNextFrame(0);
            qDebug() << "invoke stereo 2.5";
            //seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
        }
    }
}
