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
        //frame = readout_frame;
        //qDebug() << "invoke mono";
        //seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
    }
    else
    {
        // to be changed XXX
        frame = stereo_cam->getNextFrame(0);
        //calib = new StereoCalibration("C:/Aiming Beam v2/Source/Calibration/stereo_calibration.yml");
        //qDebug() << "invoke stereo";
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
                //qDebug() << "build monoseg";
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



            // if in acquisition, do segmentation
            if (inAcquisition)
            {
                qDebug() << "acq 1";

                // get frame
                if (!stereomode)
                {
                    //frame = cam->getNextFrame();
                    //qDebug() << "reading";
                    if (!ref_frame.empty() && !readout_frame.empty())
                    {
                        qDebug() << "acq 2";
                        //qDebug() << "before frames";
                        if (!writing)
                        {
                            qDebug() << "acq 2a";
                            ref_frame.copyTo(frame_on);
                            //frame_on = ref_frame.clone();
                            qDebug() << "acq 2o";
                            qDebug() << readout_frame.cols;
                            qDebug() << "acq1oo";
                            //frame = readout_frame.clone();
                            readout_frame.copyTo(frame);
                            qDebug() << "acq 2b";
                            frame_off = frame.clone();
                            qDebug() << "acq 2c";
                        }
                        else
                            continue;
                        //qDebug() << "after frames";
                    }
                    else
                    {
                        qDebug() << "CONTINUE";
                        continue;

                    }
                    qDebug() << "acq 3";
                    //qDebug() << "finished reading";
                }
                else
                {
                    frame = stereo_cam->getNextFrame(0);

                    frame_r = stereo_cam->getNextFrame(1);
                    frame_l = frame.clone();

                    // if stereo camera pair is used, rectify images
                    frame_l = calib->getRectifiedIm(frame_l,0);
                    frame_r = calib->getRectifiedIm(frame_r,1);
                }

                qDebug() << "acq 4";
                // add raw frame to avi export
                avi_out_raw->addFrame(frame);
                qDebug() << "acq 5";
                boost::thread* segmentationThread;

                if (!stereomode)
                {
                    qDebug() << "acq 6";
                    // set correlation threshold
                    seg->setThreshold(threshold);
                    // detection channel to be displayed
                    seg->switchChannel(channel);
                    //boost::thread segmentationThread(ThreadWrapper::startSegmentationThread, seg, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                    //segmentationThread = new boost::thread;
                    qDebug() << "acq 7";
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startSegmentationThread, seg, frame, frame_on, frame_off, ch1_tau, ch2_tau, ch3_tau, ch4_tau));
                    // wait for thread to end
                    qDebug() << "acq 8";


                }
                else
                {
                    // set correlation threshold
                    seg_stereo->setThreshold(threshold);
                    // detection channel to be displayed
                    seg_stereo->switchChannel(channel);
                    //boost::thread segmentationThread(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau));
                    // wait for thread to end
                    //segmentationThread.join();

                }
                qDebug() << "acq 9";
                // thread
                segmentationThread->join();
                qDebug() << "acq 10";
                // add segmented frame to avi export
                avi_out_augmented->addFrame(frame);
                qDebug() << "acq 11";
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
    if (true && avi_out_augmented && avi_out_raw)
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

void imageAcquisition::setAimingBeam(bool aiming_beam)
{
    aiming_beam_bool = aiming_beam;
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

void imageAcquisition::captureFrame()
{
    double thres = 0.0;
    int counter = 0;
    int counter_2 =0;
    int nframes = 20;
    std::vector<double> blues(nframes);
    std::vector<cv::Mat> images;

    while (thread)
    {
        if (inAcquisition)
            {
                // capture frame and store it in a temporary variable

                cv::Mat temp = cam->getNextFrame();
                cv::Mat tempcopy = temp.clone();
                //readout_frame = temp;

                Mat frame_lab;
                cvtColor(tempcopy, frame_lab, CV_BGR2Lab);
                extractChannel(frame_lab, frame_lab, 2);

                //const char * filenamef1 = "frame_lab.jpg";
                //cvSaveImage(filenamef1, &(IplImage(frame_lab)));

                cv::Scalar meanblueint;
                // average intensity of channel

                if (seg)
                {
                    cv::Rect area(seg->x0, seg->y0, seg->x1, seg->y1);
                    meanblueint = mean(frame_lab(area));

                    /*
                    qDebug() << "AREA";
                    qDebug() << area.x;
                    qDebug() << area.y;
                    qDebug() << area.width;
                    qDebug() << area.height;
                    */
                    double blueint = meanblueint.val[0];

                    // store blue level in image
                    blues[counter] = blueint;

                    // find min and max
                    double blue_mx = *max_element(std::begin(blues), std::end(blues));
                    double blue_mn = *min_element(std::begin(blues), std::end(blues));
                    double span = blue_mx - blue_mn;

                    // calculate thrshold to find aiming beam
                    thres = 0.25 * thres + 0.75 * ((0.5 * span) + blue_mn);

                    /*
                    qDebug() << "Start";
                    qDebug() << blueint;
                    qDebug() << blue_mx;
                    qDebug() << blue_mn;
                    qDebug() << thres;
                    */

                    writing = true;
                    if (blueint < thres - 0.1*span)
                        ref_frame = tempcopy;
                    else if (blueint > thres + 0.1*span)
                        readout_frame = tempcopy;
                    else
                    {
                        // do nothing
                    }
                    writing = false;

                    //if (counter_2 > 100 && counter_2 < 200)
                    //    images.push_back(tempcopy);

                    counter++;
                    if (counter == nframes) counter = 0;
                }
                else
                {
                    writing = true;
                    ref_frame = tempcopy;
                    readout_frame = tempcopy;
                    writing = false;
                }

                Sleep(10); // let it rest for 10 ms. otherwise it is likely to crash
            } else {
                // clear up blue vector
                std::fill(blues.begin(), blues.end(), 0);

                counter = 0;
                thres = 0;

            }


    }
/*

    vector<Mat>::iterator it = images.begin();
    VideoWriter_ab *tmp = new VideoWriter_ab("test.avi",frame.cols,frame.rows);

    while( it != images.end() )
    {
        qDebug() << "something";
        tmp->addFrame(*it);
        it++;
    }
    tmp->closeFile();
    qDebug() << "test";
*/
}

