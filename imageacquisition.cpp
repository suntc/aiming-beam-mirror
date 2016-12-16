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
#include "videoepiphan.h"
#include <ctime>


#define NO_LIFETIME -1

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
        // check USB camera
        cam_usb = new VideoPointGrey();
        //stereo_cam = new VideoPointGreyStereo();
        ready_usb = cam_usb->isConnected();
        // check frame grabber
        cam = new VideoEpiphan();
        ready_fg = cam->isConnected();

        ready = ready_fg || ready_usb;
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

    /*if (!stereomode)
    {
        if (true) //invivo)
            frame = cam->getNextFrame();
        else
            frame = cam_usb->getNextFrame();
        qDebug() << frame.cols;
        qDebug() << frame.rows;
    }
    else
    {
        // to be changed XXX
        frame = stereo_cam->getNextFrame(0);
        //calib = new StereoCalibration("C:/Aiming Beam v2/Source/Calibration/stereo_calibration.yml");
        //qDebug() << "invoke stereo";
        //seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
    }
    */
    bool init_output = false;
    clock_t timer_acquisition_start;
    VideoOutput *avi_out_augmented = nullptr;
    VideoOutput *avi_out_raw = nullptr;

    // stereo pair
    Mat frame_r;
    Mat frame_l;

    while (thread)
    {
        if (ctrl)
        {
            if (!seg && !stereomode)
            {
                //qDebug() << "here after";
                seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, scale_auto, ansi);
            }

            if (!seg_stereo && stereomode)
            {
                //seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
            }

            // initialization
            if (init_output==false)
            {
                // mark start of acquisition
                timer_acquisition_start = clock();

                infix = subject;
                infix.append("_run");
                infix.append(std::to_string(run_number));
                std::string filename = IOPath::getDataOutputFilename(infix,"avi","videos",subject);
                avi_out_raw = new VideoWriter_ab(filename, frame.cols, frame.rows);

                string infix0 = infix;
                infix0.append("_augmented");
                filename = IOPath::getDataOutputFilename(infix0,"avi","videos",subject);
                avi_out_augmented = new VideoWriter_ab(filename, frame.cols, frame.rows);

                init_output = true;
            }

            // if in acquisition, do segmentation
            if (inAcquisition)
            {

                // get frame
                if (!stereomode)
                {

                    // make sure we have something in readout and reference frame before segmentation
                    if (!ref_frame.empty() && !readout_frame.empty())
                    {
                        if (!writing)
                        {
                            frame_on = ref_frame;//.clone();
                            frame_off = readout_frame;//.clone();
                            frame = vis_frame;//.clone(); //frame_off.clone();
                        }
                        else
                            continue;
                    }
                    else
                    {
                        continue;
                    }
                }
                else
                {
                   // frame = stereo_cam->getNextFrame(0);

                   // frame_r = stereo_cam->getNextFrame(1);
                    stereo_cam->getNextFrame(frame, frame_r);
                    frame_l = frame.clone();

                    // if stereo camera pair is used, rectify images
                    frame_l = calib->getRectifiedIm(frame_l,0);
                    frame_r = calib->getRectifiedIm(frame_r,1);
                }

                // prevent repeated segmentations
                if (idx == idx_prev && ctrl)    continue;

                // add raw frame to avi export
                avi_out_raw->addFrame(frame);
                boost::thread* segmentationThread;

                if (!stereomode)
                {
                    // set correlation threshold
                    //seg->setThreshold(threshold);

                    // detection channel to be displayed
                    seg->switchChannel(channel);

                    // set ansi limit
                    seg->setAnsi(ansi);

                    // scale radius
                    seg->setRadius(radius);

                    // set autoscale
                    seg->setAutoScale(scale_auto);

                    if (!scale_auto)
                    {
                        // if not autoscale, set scale limits
                        seg->setColorScale(scale_min, scale_max);
                    }

                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startSegmentationThread, seg, frame, frame_on, frame_off, ch1_tau, ch2_tau, ch3_tau, ch4_tau, idx));
                }
                else
                {
                    // set correlation threshold
                    //seg_stereo->setThreshold(threshold);
                    // detection channel to be displayed
                    seg_stereo->switchChannel(channel);
                    //boost::thread segmentationThread(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau);
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame_l, frame_r, frame, ch1_tau, ch2_tau, ch3_tau, ch4_tau));

                }
                // clear lifetimes
                set_lifetime(NO_LIFETIME,1);
                set_lifetime(NO_LIFETIME,2);
                set_lifetime(NO_LIFETIME,3);
                set_lifetime(NO_LIFETIME,4);

                // thread
                segmentationThread->join();
                segmentationThread->detach();
                segmentationThread->~thread();

                idx_prev = idx;

                // display timer
                clock_t timer_acquisition = clock();
                double acq_timer = double(timer_acquisition - timer_acquisition_start);
                timer_display.push_back(acq_timer);

                // add segmented frame to avi export
                avi_out_augmented->addFrame(frame);
            }

            // show frame
            imshow("Acquisition", frame);

            // fullscreen
            // for now, only in vivo measurements
            //if (invivo)
            setWindowProperty("Acquisition", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

            // cleanup
            if (!ctrl)
            {
                //qDebug() << "seg g";
                destroyWindow("Acquisition");
                init_output=false;

                avi_out_augmented->closeFile();
                avi_out_raw->closeFile();

                if (!stereomode)
                {

                    std::string filename = IOPath::getDataOutputFilename(infix,"txt","txt",subject);
                    IOTxtData::writeTxtFile(filename, seg );

                    for (int i=0; i<5; i++)
                    {
                        string infix0;
                        infix0 = infix;
                        if (i == 0)
                        {
                            infix0 = infix0.append("_raw");
                        }
                        else
                        {
                            infix0 = infix0.append("_CH");
                            infix0 = infix0.append(to_string(i));
                        }

                        filename = IOPath::getDataOutputFilename(infix0,"jpg","figures",subject);
                        IOTxtData::writeJpgFile_mono(filename,seg,i);
                    }


                    string f = subject; f.append("_log_pulse_max"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,log_pulse_max); log_pulse_max.clear();

                    f = subject; f.append("_log_pulse_min"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,log_pulse_min); log_pulse_min.clear();

                    f = subject; f.append("_log_pulse_thres"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,log_pulse_thres); log_pulse_thres.clear();

                    f = subject; f.append("_log_pulse_cur"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,log_pulse_cur); log_pulse_cur.clear();

                    f = subject; f.append("_timer_frames"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,timer_frames); timer_frames.clear();

                    f = subject; f.append("_timer_display"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeLogFile(filename,timer_display); timer_display.clear();

                    f.clear();
                    filename.clear();

                    frame_on = Mat();
                    frame_off = Mat();
                    vis_frame = Mat();
                    ref_frame = Mat();
                    readout_frame = Mat();

                    delete seg;
                    seg = nullptr;

                    delete(avi_out_augmented);
                    delete(avi_out_raw);
                    //seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, scale_auto, ansi);

                }
                else
                {
                    //std::string infix = subject;
                    //infix.append("_run");
                    //infix.append(std::to_string(run_number));
                    std::string filename = IOPath::getDataOutputFilename(infix,"txt","txt",subject);
                    IOTxtData::writeTxtFile(filename, seg_stereo );

                    //filename = IOPath::getDataOutputFilename(infix,"jpg","figures",subject);
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

                        filename = IOPath::getDataOutputFilename(infix0,"jpg","figures",subject);
                        IOTxtData::writeJpgFile_stereo(filename,seg_stereo,i);
                    }
                    qDebug() << "after jpg";
                    delete(seg_stereo);
                    qDebug() << "invoke stereo 2";
                    seg_stereo = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
                }



            }
            else
            {
                // do nothing. no acquisition
            }
        }

    }

    // cleanup
    /*/qDebug() << "remove me and I'll crash on disconnect";
    /
    if (avi_out_augmented != nullptr && avi_out_raw != nullptr)
    {
        avi_out_augmented->closeFile();
        //delete(avi_out_augmented);
        avi_out_raw->closeFile();
        //delete(avi_out_raw);
    }
    */
    //qDebug() << "heer";

    if (!stereomode)
    {
        cam->disconnect();
        cam_usb->disconnect();
        delete(cam);
        delete(cam_usb);
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
    if (!stereomode)
    {
        cam->disconnect();
        delete(cam);

        cam_usb->disconnect();
        delete(cam);
    }
    else
    {
        stereo_cam->disconnect();
        delete(stereo_cam);
    }
}

void imageAcquisition::setInVivo(bool invivo)
{
    this->invivo = invivo;
}

void imageAcquisition::setIdx(int idx)
{
    //if(seg)
    //    seg->setIdx(idx);
    this->idx = idx;
}

void imageAcquisition::setRadius(double radius)
{
    this->radius = radius;
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
            //frame = stereo_cam->getNextFrame(0);
            qDebug() << "invoke stereo 2.5";
            //seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false);
        }
    }
}

void imageAcquisition::captureFrame()
{
    double thres = 0.0;
    int counter = 0;
    int nframes = 10;
    std::vector<double> blues(nframes);
    clock_t timer_off;

    while (thread)
    {
        if (inAcquisition)
        {
            // capture frame and store it in a temporary variable
            Mat temp;Mat temp2;
            if (invivo)
                temp = cam->getNextFrame();
            else
                if (stereomode)
                    cam_usb->getNextStereoFrame(temp,temp2);
                else
                    temp = cam_usb->getNextFrame();

            // check if there is an image. no image is passed if some parameters are changed through LV - image capture throws an error
            if (temp.empty())
            {
                temp.release();
                continue;
            }

            //to lab space and look at channel 2
            Mat frame_lab;
            cvtColor(temp, frame_lab, CV_BGR2Lab);
            extractChannel(frame_lab, frame_lab, 2);

            Mat frame_lab2;
            if (stereomode)
            {
                cvtColor(temp2, frame_lab2, CV_BGR2Lab);
                extractChannel(frame_lab2, frame_lab2, 2);
            }

            // check if object exists
            if (seg)
            {
                // draw rectangle to look for mean intensity.
                // eventually we need a way to lock this, so that the area is the same
                // for both frames
                cv::Rect area(seg->x0, seg->y0, seg->x1, seg->y1);
                // average intensity in the 2nd channel
                cv::Scalar meanblueint = mean(frame_lab(area));
                double blueint = meanblueint.val[0];

                // store blue level in vector
                blues[counter] = blueint;

                // find min and max in vector
                double blue_mx = *max_element(std::begin(blues), std::end(blues));
                double blue_mn = *min_element(std::begin(blues), std::end(blues));

                // amplitude
                double span = blue_mx - blue_mn;

                // calculate thrshold to find aiming beam
                double level = 0.8;
                thres = 0.1 * thres + 0.9 * ((level * span) + blue_mn);

                // lock reading while writing;
                writing = true;

                if (blueint < thres - 0.1*span)
                    ref_frame = temp.clone();
                else if (blueint > thres + 0.*span)
                    readout_frame = temp.clone();
                else
                {
                    // do nothing
                    //readout_frame = tempcopy;
                }
                vis_frame = temp.clone();
                writing = false;

                log_pulse_thres.push_back(thres);
                log_pulse_max.push_back(blue_mx);
                log_pulse_min.push_back(blue_mn);
                log_pulse_cur.push_back(blueint);


                clock_t now = clock();
                //qDebug() << (double)now;
                double elapsed_time = double(now - timer_off);
                //qDebug() << elapsed_time;
                timer_frames.push_back(elapsed_time);

                //if (counter_2 > 100 && counter_2 < 200)
                //    images.push_back(tempcopy);

                counter++;
                if (counter == nframes) counter = 0;

            }
            else
            {
                writing = true;
                ref_frame = temp.clone();
                readout_frame = temp.clone();
                vis_frame = temp.clone();
                writing = false;
            }

            temp.release();
            frame_lab.release();

            Sleep(1); // let it rest for ~10 ms. otherwise it is likely to crash
        } else {

            // clear timer
            timer_off = clock();

            // clear up blue vector
            std::fill(blues.begin(), blues.end(), 0);

            counter = 0;
            thres = 0;

            if (inFocus)   // manual focus
            {
                Mat focus_frame;
                if (invivo)
                    focus_frame = cam->getNextFrame();
                else
                    focus_frame = cam_usb->getNextFrame();

                imshow("Focus", focus_frame);

                // fullscreen
                setWindowProperty("Focus", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

                if (!inFocus)   destroyWindow("Focus");

                focus_frame.release();

            }

        }


    }
}

void imageAcquisition::setAutoScale(bool autoscale)
{
    scale_auto = autoscale;
}

void imageAcquisition::setScale(double mn, double mx)
{
    scale_max = mx;
    scale_min = mn;
}

void imageAcquisition::setAnsi(int ansi)
{
    this->ansi = ansi;
}

bool imageAcquisition::getUSBReady()
{
    frame = cam_usb->getNextFrame();
    return ready_usb;
}

bool imageAcquisition::getFGReady()
{

    frame = cam->getNextFrame();
    return ready_fg;
}

