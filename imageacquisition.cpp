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
#include <iostream>
#include "acousticfeedback.h"




#define NO_LIFETIME -1

using namespace cv;

// Class constructor
imageAcquisition::imageAcquisition(bool stereomode)
{
    this->stereomode = stereomode;
    startupCamera(channel, threshold);
    acoustic = new AcousticFeedback();

}

void imageAcquisition::startupCamera(int ch, float thres)
{
    // startup one or two cameras
    // check if leading camera is connected. Monomode can only be realized if leading camera is plugged in

    if (!cam)
    {
        // check frame grabber
        cam = new VideoEpiphan();
    }

    ready_fg = cam->isConnected(0);


    if (!cam_usb)
    {
        // check USB camera
        cam_usb = new VideoPointGrey();
    }

    ready_usb = cam_usb->isConnected(0);

    // camera(s) available?
    ready = ready_fg || ready_usb;

    channel = ch; // channel to be displayed
    threshold = thres; // cross-correlation threshold

}

void imageAcquisition::startAcquisition()
{
    // initialization
    bool init_output = false;
    clock_t timer_acquisition_start;
    VideoOutput *avi_out_augmented = nullptr;
    VideoOutput *avi_out_raw = nullptr;


    while (thread)
    {

        if (ctrl)
        {
            if (!seg && !stereomode)
            {
                seg = new Segmentation(frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, scale_auto, ansi, pentero_mode);
            }

            if (!seg_stereo && stereomode)
            {
                seg_stereo  = new StereoSegmentation(calib, frame, Point(1,1), Point(frame.cols, frame.rows), false, channel, false, scale_auto, ansi);
            }

            // initialization
            if (init_output==false)
            {

                cleanup = false;

                // mark start of acquisition
                timer_acquisition_start = clock();

                // initialize video output, raw and augmented
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

                acoustic->start_feedback();
            }

            // if in acquisition, do segmentation
            if (inAcquisition)
            {

                // make sure we have something in readout and reference frame before segmentation
                if (!ab_frame.empty() && !bg_frame.empty())
                {

                    // make sure frames are not read while they are being written. Removing this condition will make the application crash randomly
                    if (!writing)
                    {
                        frame_on = ab_frame;//.clone();
                        frame_off = bg_frame;//.clone();
                        frame = vis_frame;//.clone(); //frame_off.clone();

                        if (stereomode)
                        {

                            if (ab_frame2.empty() || bg_frame2.empty())
                                continue;

                            frame_on2 = ab_frame2;
                            frame_off2 = bg_frame2;

                            // rectify images
                            frame_on = calib->getRectifiedIm(frame_on,0);
                            frame_off = calib->getRectifiedIm(frame_off,0);
                            frame_on2 = calib->getRectifiedIm(frame_on2,1);
                            frame_off2 = calib->getRectifiedIm(frame_off2,1);
                        }
                    }
                    else
                        continue;
                }
                else
                {
                    continue;
                }


                // prevent repeated segmentations
                if (idx == idx_prev && ctrl)    continue;

                // add raw frame to avi export
                avi_out_raw->addFrame(frame);

                //boost::thread* feedbackThread;
                //feedbackThread = new boost::thread(boost::bind(ThreadWrapper::startFeedback, acoustic ));
                //segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startSegmentationThread, seg, frame, frame_on, frame_off, ch1_tau, ch2_tau, ch3_tau, ch4_tau, idx));
                //ThreadWrapper::startFeedback(acoustic);


                boost::thread* segmentationThread;

                // Segmentation frame rate
                clock_t timer_acquisition = clock();
                double acq_timer = double(timer_acquisition - timer_acquisition_start);

                if (!stereomode)
                {
                    // call setters to allow adjustment of acquisition parameters on the fly

                    // set timer. this should be passed in the constructor, but boost does not accept more than 10 arguments.
                    seg->setTimer(acq_timer);

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

                    // initialize segmentation
                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startSegmentationThread, seg, frame, frame_on, frame_off, ch1_tau, ch2_tau, ch3_tau, ch4_tau, idx));
                }
                else
                {
                    // call setters to allow adjustment of acquisition parameters on the fly

                    // set timer. this should be passed in the constructor, but boost does not accept more than 10 arguments.
                    seg_stereo->seg->setTimer(acq_timer);

                    // detection channel to be displayed
                    seg_stereo->switchChannel(channel);

                    // set ansi limit
                    seg_stereo->setAnsi(ansi);

                    // scale radius
                    seg_stereo->seg->setRadius(radius);

                    // set autoscale
                    seg_stereo->setAutoScale(scale_auto);

                    if (!scale_auto)
                    {
                        // if not autoscale, set scale limits
                        seg_stereo->setColorScale(scale_min, scale_max);
                    }

                    // start segmentation
                    std::pair <double,double> lt12;
                    lt12 = std::make_pair(ch1_tau,ch2_tau);
                    std::pair <double,double> lt34;
                    lt34 = std::make_pair(ch3_tau,ch4_tau);

                    segmentationThread = new boost::thread(boost::bind(ThreadWrapper::startStereoSegmentationThread, seg_stereo, frame, frame_on, frame_off, frame_on2, frame_off2, lt12, lt34, idx));
                }

                // TODO: need to fix this
                //clear lifetimes
                //set_lifetime(NO_LIFETIME,1);
                //set_lifetime(NO_LIFETIME,2);
                //set_lifetime(NO_LIFETIME,3);
                //set_lifetime(NO_LIFETIME,4);

                // thread
                //feedbackThread->join();
                //feedbackThread->detach();

                segmentationThread->join();
                segmentationThread->detach();
                //segmentationThread->interrupt();

                // update id, to prevent repeated segmentations
                idx_prev = idx;
                // add segmented frame to avi export
                avi_out_augmented->addFrame(frame);

            }

            namedWindow("Acquisition",WINDOW_NORMAL); //added to fix full screen issue

            // show frame, this has to be called before full screen, otherwise the y mouse click will be wrong
            imshow("Acquisition", frame);

            // fullscreen
            //setWindowProperty("Acquisition", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);//make window fullscreen

            // callback function for mouse event - looking for button click
            setMouseCallback("Acquisition", adjustArea, this);

        }
        else
        {
            // cleanup. This condition is only met when acquisition is stopped by the user
            if (!cleanup)
            {
                destroyWindow("Acquisition");
                init_output=false;

                acoustic->stop_feedback();

                // save video streams and delete corresponding pointers
                avi_out_augmented->closeFile();
                avi_out_raw->closeFile();
                delete(avi_out_augmented);
                delete(avi_out_raw);

                // log files
                std::string filename = IOPath::getDataOutputFilename(infix,"txt","txt",subject);

                // Write data files (lifetimes, segmentation coordinates and height profile)
                if (!stereomode)
                    IOTxtData::writeTxtFile(filename, seg);
                else
                    IOTxtData::writeTxtFile(filename, seg_stereo);

                // save figures
                for (int i=0; i<6; i++)
                {
                    string infix0;
                    infix0 = infix;

                    switch (i) {
                    case 0: // raw image
                        infix0 = infix0.append("_raw");
                        break;
                    case 5: // height profile
                        if (!stereomode)    continue;
                        infix0 = infix0.append("_profile");
                    default: // lifetimes
                        infix0 = infix0.append("_CH");
                        infix0 = infix0.append(to_string(i));
                    }

                    filename = IOPath::getDataOutputFilename(infix0,"jpg","figures",subject);
                    if (!stereomode)
                        IOTxtData::writeJpgFile_mono(filename,seg,i);
                    else
                        IOTxtData::writeJpgFile_stereo(filename,seg_stereo,i);

                }

                // build filename
                string f = subject; f.append("_segmentation"); f.append("_run").append(std::to_string(run_number));
                filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                IOTxtData::writeSegmentationLog(filename, timer_frames, log_pulse_thres, log_pulse_max, log_pulse_min, log_pulse_cur);

                // clear vectors
                timer_frames.clear();
                log_pulse_thres.clear();
                log_pulse_max.clear();
                log_pulse_min.clear();
                log_pulse_cur.clear();

                // log stereo data
                if (stereomode)
                {
                    f = subject; f.append("_stereo"); f.append("_run").append(std::to_string(run_number));
                    filename = IOPath::getDataOutputFilename(f,"txt","logs",subject);
                    IOTxtData::writeStereoLog(filename, seg_stereo->log_disparity_y, seg_stereo->log_synchronized);

                    // clear vectors
                    seg_stereo->log_synchronized.clear();
                    seg_stereo->log_disparity_y.clear();

                }

                f.clear();
                filename.clear();

                // clear matrices
                frame_on = Mat();
                frame_off = Mat();
                frame_on2 = Mat();
                frame_off2 = Mat();
                vis_frame = Mat();
                ab_frame = Mat();
                ab_frame2 = Mat();
                bg_frame = Mat();
                bg_frame2 = Mat();

                // restart segmentation objects
                if (!stereomode & !pentero_mode)
                {
                    delete seg;
                    seg = nullptr;
                }
                else
                {
                    delete seg_stereo;
                    seg_stereo = nullptr;
                }

                cleanup = true;
            }
        }
    }

    // clear camera and segmentation objects
    cam->disconnect();
    cam_usb->disconnect();
    delete(cam);
    delete(cam_usb);
    if (!stereomode)
        delete(seg);
    else
        delete(seg_stereo);
}

void imageAcquisition::shutdownCamera()
{
    cam->disconnect();
    delete(cam);

    cam_usb->disconnect();
    delete(cam);
}

void imageAcquisition::load_calib()
{
    startupCamera(channel, threshold);

    if (stereomode)
    {
        if (!calib)
        {

            // initialize calibration
            string filename = IOPath::getAppDir();
            if (invivo) // call frame grabber segmentation
            {
                if (firefly)
                    filename.append("Calibration\\Calibration_invivo_firefly");
                else
                    filename.append("Calibration\\Calibration_invivo_standard");
            }
            else    // call usb camera segmentation
                filename.append("Calibration\\Calibration_exvivo");

            filename.append(".yml");
            calib = new StereoCalibration(filename);

        }
    }

}

void imageAcquisition::captureFrame()
{
    double thres = 0.0;
    double thres2 = 0.0;
    int counter = 0;
    int nframes = 10;
    std::vector<double> blues(nframes);
    std::vector<double> blues2(nframes);
    clock_t timer_off;

    while (thread)
    {
        if (inAcquisition)
        {
            std::cout << "start - get images";
            // capture frame and store it in a temporary variable
            Mat temp;Mat temp2;
            if (invivo)
                //temp = cam->getNextFrame();
                if (!stereomode)
                    temp = cam->getNextFrame();
                else
                    cam->getNextStereoFrame(temp, temp2);
            else
                if (!stereomode)
                    temp = cam_usb->getNextFrame();
                else
                    cam_usb->getNextStereoFrame(temp, temp2);

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
            std::cout << "init variables";
            // check if object exists
            if (seg || seg_stereo)
            {

                // initialize variables to do segmentation.
                // also initializing vars for stereo mode that may not be necessary
                double blue_mx;
                double blue_mn;
                double blueint;

                double blue_mx2;
                double blue_mn2;
                double blueint2;

                cv::Scalar meanblueint;
                cv::Scalar meanblueint2;

                cv::Rect area;
                cv::Rect area2;

                // draw rectangle to look for mean intensity.
                // eventually we need a way to lock this, so that the area is the same
                // for both frames
                if (!stereomode)
                {

                    // x
                    area.x = seg->x0;   area.width = seg->x1;
                    // y
                    area.y = seg->y0;   area.height = seg->y1;

                }
                else    // stereomode
                {

                    // cam 1
                    area.x = seg_stereo->seg->x0;   area.width = seg_stereo->seg->x1;
                    area.y = seg_stereo->seg->y0;   area.height = seg_stereo->seg->y1;

                    // cam 2
                    area2.x = seg_stereo->x0;   area2.width = seg_stereo->x1;
                    area2.y = seg_stereo->y0;   area2.height = seg_stereo->y1;

                    // rectify image of top cam
                    frame_lab = calib->getRectifiedIm(frame_lab,0);
                    // rectify image of side cam
                    frame_lab2 = calib->getRectifiedIm(frame_lab2,1);

                }

                std::cout << "before seg";
                // average intensity in the 2nd channel, within ROI defined by area
                meanblueint = mean(frame_lab(area));
                blueint = meanblueint.val[0];

                // store blue level in vector
                blues[counter] = blueint;

                // find min and max in vector
                blue_mx = *max_element(std::begin(blues), std::end(blues));
                blue_mn = *min_element(std::begin(blues), std::end(blues));

                // amplitude
                double span = blue_mx - blue_mn;
                double span2 = 0;

                // calculate thrshold to find aiming beam
                double level = 0.8;
                thres = 0.1 * thres + 0.9 * ((level * span) + blue_mn);

                if (stereomode)
                {
                    // average intensity in the 2nd channel, within ROI defined by area
                    meanblueint2 = mean(frame_lab2(area2));
                    blueint2 = meanblueint2.val[0];

                    // blue-ish intensity in vecotr
                    blues2[counter] = blueint2;

                    // find min and max in vector
                    blue_mx2 = *max_element(std::begin(blues2), std::end(blues2));
                    blue_mn2 = *min_element(std::begin(blues2), std::end(blues2));

                    // amplitude
                    span2 = blue_mx2 - blue_mn2;

                    // calculate thrshold to find aiming beam
                    double level = 0.8;
                    thres2 = 0.1 * thres2 + 0.9 * ((level * span2) + blue_mn2);
                }

                // control vars to check synchronization in stereomode. Not used otherwise
                bool beam1_on = false;
                bool beam1_reject = false;

                // Setting on and off frames for main camera
                // lock reading while writing;
                writing = true;
                if (blueint < thres - 0.1*span) // aiming beam detected
                {
                    ab_frame = temp.clone();
                    beam1_on = true;
                }
                else if (blueint > thres + 0.*span) // aiming beam not detected
                {
                    bg_frame = temp.clone();
                }
                else    // within exclusion criteria
                {
                    beam1_reject = true;
                }
                std::cout << "after seg";
                // find aiming beam in second camera
                if (stereomode)
                {
                    is_synchronized = false;
                    if (blueint2 < thres2 - 0.1*span2)  // aiming beam detected
                    {
                        if (beam1_on)   is_synchronized = true;
                        ab_frame2 = temp2.clone();
                    }
                    else if (blueint2 > thres2 + 0.*span2)  // aiming beam not detected
                    {
                        if (!beam1_on)  is_synchronized = true;
                        bg_frame2 = temp2.clone();
                    }
                    else    // exlusion criteria
                    {
                        is_synchronized = false;

                    }

                    if(beam1_reject)    is_synchronized = false;
                    seg_stereo->set_synchronized(is_synchronized);

                }

                vis_frame = temp.clone();
                writing = false;    // unlock reading

                // write to log (1st camera) - is it worth doing for both cameras?
                log_pulse_thres.push_back(thres);
                log_pulse_max.push_back(blue_mx);
                log_pulse_min.push_back(blue_mn);
                log_pulse_cur.push_back(blueint);

                clock_t now = clock();
                double elapsed_time = double(now - timer_off);
                timer_frames.push_back(elapsed_time);

                counter++;
                if (counter == nframes) counter = 0;
            }
            else
            {
                writing = true;
                ab_frame = temp.clone();
                bg_frame = temp.clone();
                vis_frame = temp.clone();
                writing = false;

                if (stereomode)
                {
                    ab_frame2 = temp2.clone();
                    bg_frame2 = temp2.clone();
                }
            }

            temp.release();
            temp2.release();
            frame_lab.release();
            frame_lab2.release();

            //Sleep(10); // let it rest for ~10 ms. otherwise it is likely to crash
        } else {

            // clear timer
            timer_off = clock();

            // clear up blue vector
            std::fill(blues.begin(), blues.end(), 0);
            std::fill(blues2.begin(), blues2.end(), 0);

            counter = 0;
            thres = 0;

            if (inFocus)   // manual focus
            {
                Mat focus_frame;
                if (invivo)
                    focus_frame = cam->getNextFrame();
                else
                    focus_frame = cam_usb->getNextFrame();
                // fullscreen
                namedWindow("Focus",WINDOW_NORMAL);
                setWindowProperty("Focus", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
                imshow("Focus", focus_frame);

                if (!inFocus)   destroyWindow("Focus");


                focus_frame.release();

            }
            else
            {
                // do nothing
            }

        }



    }
}

void imageAcquisition::adjustArea(int event, int x, int y, int flags, void* userdata)
{
    if  ( event == EVENT_LBUTTONDOWN )  // left button is click
    {
        // following explanation of http://stackoverflow.com/questions/25748404/how-to-use-cvsetmousecallback-in-class
        imageAcquisition* acq = reinterpret_cast<imageAcquisition*>(userdata);

        if (!acq->stereomode)
            acq->seg->adjustArea(x, y);
        else
            acq->seg_stereo->seg->adjustArea(x, y);

        //delete(acq);

        //qDebug() << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
    }
}


// setters
void imageAcquisition::setInVivo(bool invivo)
{

    this->invivo = invivo;

    // load calibration files
    load_calib();
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

bool imageAcquisition::getUSBOpen(int camID)
{
    return cam_usb->isConnected(camID);
}

bool imageAcquisition::getFGOpen(int camID)
{
    return cam->isConnected(camID);
}

void imageAcquisition::setFirefly(bool firefly)
{
    this->firefly = firefly;
}

void imageAcquisition::setPentero(bool pentero_mode)
{
    this->pentero_mode = pentero_mode;
}

Mat imageAcquisition::getCurrentSegmFrame()
{
    Mat segmFrame;

    if (seg && (seg->overlay))
    {
        Mat segmFrame1 = seg->overlay->mergeOverlay(frame);
        addWeighted( segmFrame1, 0.5, frame, 0.5, 0.0, segmFrame);
    }
    return segmFrame;
}

void imageAcquisition::setDecon(LaguerreDeconvolution * deconvolution)
{
    // pass deconvolution to acoustic feedback object
    decon = deconvolution;
    if (acoustic)
        acoustic->setDecon(decon);
}
