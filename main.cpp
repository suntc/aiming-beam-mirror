#include "mainwindow.h"
#include <QApplication>
#include <QProcess>
#include <QDebug>
#include <iostream>
#include "tcp_ip.h"
#include <boost/filesystem.hpp>
//#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include "guiupdater.h"
#include "camera.h"
#include "videopointgrey.h"
#include "videoinput.h"
#include "opencv2/core.hpp"
#include "imageacquisition.h"
#include "threadwrapper.h"
#include "errorhandler.h"
#include "laguerredeconvolution.h"
#include "stereocalibration.h"
#include "iopath.h"

// Set of definitions for the state
#define OFFLINE -1
#define STANDBY 0
#define TEST_EXVIVO 1
#define TEST_INVIVO 2
#define ACQUISITION_EXVIVO 3
#define ACQUISITION_INVIVO 4

// TCP IP comm
#define TERMINATOR "\r\n"
#define SEPARATOR ":"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

/**
 * @brief buildWriteCmd
 * @param cmd
 * @param value
 *
 * Build command to be written to TCP IP, based on value (integer)
 */
void buildWriteCmd(char cmd[], int value)
{

    // integer to char
    char valueToStr[1];
    sprintf(valueToStr,"%d", value);

    // concatenate cmd & val & \r\n (for comm)
    strcat(cmd, strcat(valueToStr, "\r\n"));
}

/**
 * @brief set_ack
 * @param key
 * @param value
 * @return _cmd: full command
 *
 * Build acknowledge command to be sent to LV
 */

char* set_ack (string key, string value)
{
    string cmd = key + SEPARATOR + value + TERMINATOR;

    char cmd_[1024];
    strncpy(cmd_, cmd.c_str(), sizeof(cmd_));
    cmd.clear();
    cmd_[sizeof(cmd_) - 1] = 0;

    return cmd_;
}

void startup(GUIupdater *ui)
{

    const char *ip = "1.1.1.1";
    const char *port = "2100";
    bool init = true;

    // other vars
    string subject;     // case ID no.
    bool aiming_beam_correction = false;    // automatic correction of the aiming beam
    bool aiming_beam_value = false;  // aiming beam power (0 to 100)
    bool invivo = false;    // ex vivo false, in vivo true
    float time_resolution = 0;  // digitizer time resolution, value in ns
    int data_length = 0;    // length of the fluorescence data in each channel?
    int irf_length = 0;     // length of the IRF in each channel
    int run = 0;    // run number
    bool stereomode = false;    // stereomode on or off
    bool firefly = false;   // flag for firefly or standard camera, in vivo mode only for daVinci camera
    bool pentero = true; // flag to use fluorescence data overlay on white light images

    // lifetime boundaries & other display parameters
    bool lt_auto = true; // automatic scale
    int lt_min = 1;     // min limit
    int lt_max = 6;     // max limit
    int channel = 2;    // channel to be displayed, default 2
    int ansi = 5;       // ansi limit, as the maximum number of frames that a single pixel can be imaged

    // Unused variables in the processing, but called in the code. Keep them for now
    double threshold = 0.96;    // cross-correlation threshold
    double radius = 1.0;
    int width = 852;    // display width (number of columns)
    int height = 479;   // display height (number of rows)

    // image acquisition settings
    bool focus = false;     // if true, manual focus is on and camera is streaming
    bool acquire = false;   // if true, enter in acquisition mode

    Mat im = imread("C:/Aiming Beam v2/Release/release/pics/green_light.png", CV_LOAD_IMAGE_COLOR);

    // initialise application mode
    int mode = OFFLINE;
    int previous_mode = -2;
    ui->setMode(mode);

    // initialize image acquisition object (this is for the exvivo camera only. may need to be changed in the future)
    imageAcquisition *acq = new imageAcquisition(stereomode);
    StereoCalibration *calib;

    // look for usb cameras and frame grabber
    ui->setCameraStatus(acq->getUSBOpen(0), acq->getUSBOpen(1), acq->getFGOpen(0), acq->getFGOpen(1));

    // If none of the cameras are detected
    while (!acq->ready)
    {
        // message to user
        ui->throwError("Unable to detect cameras. Waiting...");

        // acknowledge error
        ui->setError(true);

        // shutdown camera and cleanup
        //acq->shutdownCamera();

        // wait
        Sleep(1000);

        // try to reconnect to camera
        acq->startupCamera(channel, threshold);

    }

    // at least one camera was detected. No error to be displayed
    ui->setError(false);
    ui->setCameraStatus(acq->getUSBOpen(0), acq->getUSBOpen(1), acq->getFGOpen(0), acq->getFGOpen(1));

    // check stereomode
    if (!acq->getUSBOpen(0) && !acq->getUSBOpen(1))
        ui->throwError("USB cameras not available");
    else
       if (!acq->getUSBOpen(0) || !acq->getUSBOpen(1))
            ui->throwError("Stereo mode (USB) not available");

    if (!acq->getFGOpen(0) || !acq->getFGOpen(1))
        ui->throwError("Stereo mode (frame grabber) not available");

    // initialize TCP/IP communication. It will wait here until the communication is established
    TCP_IP conn (ip, port);

    // make sure we are alive. Let LV be aware of that
    conn.write("alive\r\n");

    // check connection
    if (conn.isOpen()){

        // connected. in standby mode
        mode = STANDBY;
        ui->setMode(mode);

        // acknowledge we are ready to go
        ui->setReady(true);

        // initialize vars to manage communications
        string output;
        string key;
        string value;
        int len;

        // variables to hold IRFs & data
        vector<double> iIRF_CH1;
        vector<double> iIRF_CH2;
        vector<double> iIRF_CH3;
        vector<double> iIRF_CH4;
        vector<double> data_CH1;
        vector<double> data_CH2;
        vector<double> data_CH3;
        vector<double> data_CH4;
        vector<double> tempdata;

        // control variable to mark initialization of iIRFs
        bool iIRFs_initialized = false;
        LaguerreDeconvolution *decon = NULL;


        // measurement # within the acquisition
        int idx = 0;
        //double ch1_tau = 0.0;
        //double ch2_tau = 0.0;
        //double ch3_tau = 0.0;
        //double ch4_tau = 0.0;

        acq->thread = true; // control acquisition thread

        // start segmentation/acquisition thread, to run indefinitely
        boost::thread AcquisitionThread(ThreadWrapper::StartAcquire, acq);

        // Continuous Frame acquisition thread, also running indefinitely
        boost::thread FrameAcquisition(ThreadWrapper::StartFrameCapture, acq);

        while (true){

            // initialize deconvolution only once
            if (iIRFs_initialized==false && iIRF_CH1.size()>0 && iIRF_CH2.size()>0 && iIRF_CH3.size()>0 && iIRF_CH4.size()>0)
            {

                decon = new LaguerreDeconvolution(iIRF_CH1,iIRF_CH2,iIRF_CH3,iIRF_CH4,time_resolution);
                acq->setDecon(decon);
                iIRF_CH1.clear();
                iIRF_CH2.clear();
                iIRF_CH3.clear();
                iIRF_CH4.clear();

                iIRFs_initialized = true;
            }

            // have the read command within a thread, so the communication is always opened
            boost::thread readingThread(ThreadWrapper::StartRead, conn, &output, &len, &tempdata);



            // strip output \r\n
            output = output.substr(0, len - 2);

            // get key and value from output
            size_t f = output.find(":");

            // check if there is any value associated to the key
            if (f != std::string::npos)
            {
                // get key (string before :)
                key = output.substr(0, f);

                // get value (string after :), always as double or int
                //  +1 to account for the :
                value = output.substr(f + 1, len - f + 1);

            } else {
                // set defaults
                key = output;
                value = "";
            }

            // list of commands
            // disconnect from peer
            if (key.compare("disconnect") == 0)
            {

                // disconnecting from peer
                qDebug() << "disconnecting";

                // send acknowledgment
                conn.write("disconnect:1\r\n");

                // give it some time before disconnecting
                Sleep(500);

                // disconnect from this end
                conn.disconnect();

                // wait so that master can close connection
                Sleep(500);
                qDebug() << "disconnecting";
                break;
            }

            // setting subject id
            else if (key.compare("!subject") == 0)
            {

                subject = value;
                acq->subject = value;

                //create path
                string dir_path = IOPath::getDataDir();
                dir_path.append(subject);

                // create subject folder
                boost::filesystem::create_directory((dir_path).c_str());

                // videos folder
                string foldername = IOPath::getCategoryFolderPath("videos",subject);
                boost::filesystem::create_directory(foldername.c_str());

                // figures folder
                foldername = IOPath::getCategoryFolderPath("figures",subject);
                boost::filesystem::create_directory(foldername.c_str());

                // txt folder
                foldername = IOPath::getCategoryFolderPath("txt",subject);
                boost::filesystem::create_directory(foldername.c_str());

                // logs folder
                foldername = IOPath::getCategoryFolderPath("logs",subject);
                boost::filesystem::create_directory(foldername.c_str());

                // create data, videos, figure and logs folder
                // send acknowledgment
                //conn.write("test\r\n");
                conn.write(set_ack(key, subject));
            }

            // aiming beam correction on/off?
            else if (key.compare("!beam_adjust") == 0)
            {

                // aiming beam correction?
                aiming_beam_correction = (value.compare("1") == 0) ? true : false;
                conn.write(set_ack(key, value));
            }

            // aiming beam value
            else if (key.compare("!beam_value") == 0)
            {

                // aiming beam value
                aiming_beam_value = (value.compare("1") == 0) ? true : false;

                // set value for segmentation
                acq->setAimingBeam(aiming_beam_value);
                conn.write(set_ack(key, value));
            }

            // digitizer time resolution
            else if (key.compare("!resolution") == 0)
            {

                time_resolution = stod(value);

                // send acknowledgment
                conn.write(set_ack(key, value));

                //qDebug() << time_resolution;

            }

            // length of the fluorescence data
            else if (key.compare("!data_length") == 0)
            {

                data_length = stoi(value);

                // send acknowledgment
                conn.write(set_ack(key, value));

            }

            // IRF length
            else if (key.compare("!irf_length") == 0)
            {

                irf_length = stoi(value);

                // send acknowledgment
                conn.write(set_ack(key, value));

            }

            // in vivo/ex vivo modes
            else if (key.compare("!invivo") == 0)
            {

                // aiming beam correction?
                invivo = (value.compare("1") == 0) ? true : false;

                // set in vivo state;
                // if true, to frame grabber
                // if false, to external usb camera
                acq->setInVivo(invivo);

                if (invivo)
                {
                    // check if frame grabber is ready
                    value = acq->getFGReady() ? "1" : "0";
                }
                else
                {
                    // check if camera is ready
                    value = acq->getUSBReady() ? "1" : "0";

                }

                conn.write(set_ack(key, value));

            }

            // manual focus
            else if (key.compare("!focus") == 0)
            {
                focus = (value.compare("1") == 0) ? true : false;
                conn.write(set_ack(key, value));
            }

            // acquisition?
            else if (key.compare("!acquire") == 0)
            {
                acquire = (value.compare("1") == 0) ? true : false;
                conn.write(set_ack(key, value));

                // let user know that acquisition has started
                //Beep(1200, 1000); // 1.2kHz, 1s

            }

            // automatic lifetime boundaries
            else if (key.compare("!autocolor") == 0)
            {
                lt_auto = (value.compare("1") == 0) ? true : false;

                acq->setAutoScale(lt_auto);
                conn.write(set_ack(key, value));
            }

            // lifetime boundaries
            else if(key.compare("!color") == 0)
            {

                // get key and value from output
                size_t s = value.find(";");

                if (s != std::string::npos)
                {
                    // get min lifetime
                    lt_min = stoi(value.substr(0, s));

                    // get max lifetime
                    lt_max = stoi(value.substr(s + 1, value.length() - f + 1));

                    acq->setScale(lt_min, lt_max);

                }

                conn.write(set_ack(key, value));

            }

            // radius of segmentation
            else if (key.compare("!radius") == 0)
            {

                radius = stod(value);

                // set radius
                acq->setRadius(radius);
                // acknowledgment
                conn.write(set_ack(key, value));

            }

            // channel to be displayed
            else if (key.compare("!channel") == 0)
            {
                // string to integer
                channel = stoi(value);

                // set acquisition channel
                acq->channel = channel;

                // acknowledgment
                conn.write(set_ack(key, value));
            }

            // channel to be displayed
            else if (key.compare("!ansi") == 0)
            {
                ansi = stoi(value);
                acq->setAnsi(ansi);
                conn.write(set_ack(key, value));


            }

            // correlation threshold
            else if (key.compare("!threshold") == 0)
            {
                // string to double
                threshold = stod(value);

                // set cross-correlation threshold
                acq->threshold = threshold;

                //qDebug() << threshold;

                conn.write(set_ack(key, value));
            }

            // display resolution
            else if (key.compare("!display_res") == 0)
            {

                // low res, values are the same as in previous software (unknown reason for such specific w x h)
                if (value.compare("low") == 0)
                {
                    width = 852;
                    height = 459;
                }
                else // high res, HD standard (may need to be changed in the future)
                {
                    width = 1280;
                    height = 720;
                }

                // adjust resolution for segmentation
                acq->set_resolution(width, height);

                // acknowledge
                conn.write(set_ack(key, value));
            }

            // run number
            else if (key.compare("!run") == 0)
            {
                // set run number
                run = stoi(value);
                acq->run_number = run;

                // acknowledge tcp connection
                conn.write(set_ack(key, value));

            }

            // set for 3d mode
            else if (key.compare("!3dset") == 0)
            {
                stereomode = (value.compare("1") == 0) ? true : false;

                //acknowledge
                conn.write(set_ack(key, value));

                // toggle modes
                acq->set_mode(stereomode);

            }

            // start 3d calibration
            else if (key.compare("!3dcal") == 0)
            {
                // start 3d calibration here
                // acknowledge command
                conn.write(set_ack(key, "1"));

                acq->set_mode(true);
                calib = new StereoCalibration(acq->cam_usb, Size(9,6), 0.25); //0.5
                if (calib->isReady())
                {
                    string filename = IOPath::getAppDir();

                    if (mode == 1)
                        filename.append("Calibration\\Calibration_exvivo");
                    else if (mode == 2)
                    {
                        if (firefly)
                        {
                            filename.append("Calibration\\Calibration_invivo_firefly");
                        }
                        else
                        {
                            filename.append("Calibration\\Calibration_invivo_standard");
                        }
                    }
                    //string counter = IOPath::getandincreaseCurrentCounter();
                    //filename.append(counter);
                    filename.append(".yml");
                    //qDebug() << filename.c_str();

                    //calib->saveCalibration(filename);
                }
                else
                {
                    ui->throwError("3D calibration not ready");
                }
            }

            // query mode
            else if (key.compare("?mode") == 0)
            {

                char cmd[64] = "mode:";
                buildWriteCmd(cmd, mode);

                // send acknowledgment
                conn.write(cmd);

            }

            // channel 1 IRF (instrument response function)
            else if (key.compare("&irf1") == 0)
            {
                iIRF_CH1 = tempdata;
                //qDebug() << iIRF_CH1.size();
                conn.write(set_ack(key, "1"));

            }

            // channel 2 IRF (instrument response function)
            else if (key.compare("&irf2") == 0)
            {
                iIRF_CH2 = tempdata;
                //qDebug() << iIRF_CH2.size();
                conn.write(set_ack(key, "1"));

            }

            // channel 3 IRF (instrument response function)
            else if (key.compare("&irf3") == 0)
            {
                iIRF_CH3 = tempdata;
                //qDebug() << iIRF_CH3.size();
                conn.write(set_ack(key, "1"));

            }

            // channel 4 IRF (instrument response function)
            else if (key.compare("&irf4") == 0)
            {
                iIRF_CH4 = tempdata;
                //qDebug() << iIRF_CH4.size();
                conn.write(set_ack(key, "1"));

            }

            // channel 1 data
            else if (key.compare("%dat1") == 0)
            {
                data_CH1 = tempdata;
                double lifetime = decon->getLifetime(data_CH1,1);
                data_CH1.clear();
                acq->set_lifetime(lifetime,1);
                conn.write(set_ack(key, std::to_string(lifetime)));
            }

            // channel 2 data
            else if (key.compare("%dat2") == 0)
            {
                data_CH2 = tempdata;
                double lifetime = decon->getLifetime(data_CH2,2);
                data_CH2.clear();
                acq->set_lifetime(lifetime,2);
                conn.write(set_ack(key, std::to_string(lifetime)));
            }

            // channel 3 data
            else if (key.compare("%dat3") == 0)
            {
                data_CH3 = tempdata;
                double lifetime = decon->getLifetime(data_CH3,3);
                data_CH3.clear();
                acq->set_lifetime(lifetime,3);
                conn.write(set_ack(key, std::to_string(lifetime)));
            }

            // channel 4 data
            else if (key.compare("%dat4") == 0)
            {
                data_CH4 = tempdata;
                double lifetime = decon->getLifetime(data_CH4,4);
                data_CH4.clear();
                acq->set_lifetime(lifetime,4);
                conn.write(set_ack(key, std::to_string(lifetime)));
            }

            // data point id
            else if (key.compare("!idx") == 0)
            {
                idx = stoi(value);
                acq->setIdx(idx);
                conn.write(set_ack(key, value));
            }

            // calibration file for invivo, firefly or standard camera?
            else if (key.compare("!firefly") == 0)
            {
                firefly = (value.compare("1") == 0) ? true : false;

                // point to different calibration file
                acq->setFirefly(firefly);

                //acknowledge
                conn.write(set_ack(key, value));

            }

            else if (key.compare("!pentero") == 0)
            {
                pentero = (value.compare("1") == 0) ? true : false;

                // use white light overlay
                acq->setPentero(pentero);

                // acknowledge
                conn.write(set_ack(key, value));
            }

            else
            {

                // something unexpected was received. Are we still connected?
                if (!init)
                {

                    // make sure we are still connected
                    conn.write("alive\r\n");


                    if (!conn.isOpen())
                    {
                        //qDebug() << "To disconnect";

                        // disconnect
                        conn.disconnect();

                        // out of the loop
                        break;
                    }
                }

                // the buffer always needs to cleared upon initialization
                // Therefore, we use this var not to allow the command alive to be sent during init
                init = false;

            }

            //if(stereomode==true && acq->stereomode==false)
            //    acq->set_mode(true);
            // set acquisition loop condition
            /*
            if (acquire)
            {
                acq->ctrl = acquire;
                acq->inAcquisition = acquire;   // differentiate between focusing and acquisition

            }
            else
            {
                if (acq->cleanup)
                {
                    acq->inAcquisition = false;
                    acq->ctrl = false;
                }
                else
                {
                    acq->ctrl = acquire;
                    acq->inAcquisition = acquire;
                }
            }
            */
            //acq->ctrl = acquire;
            acq->inAcquisition = acquire;
            acq->inFocus = focus;

            // check current working mode
            mode = (focus) ?
                        ((invivo) ? TEST_INVIVO : TEST_EXVIVO)
                      :
                        (acquire) ?
                            ((invivo) ? ACQUISITION_INVIVO : ACQUISITION_EXVIVO)
                          :
                            STANDBY;
            if (mode != previous_mode)
                ui->setMode(mode);

            if (mode == STANDBY && previous_mode != STANDBY)
            {
                // display green LED image, if ready to start acquisition
                // in Pentero mode show current segmentation, if available
                Mat cursegIm;
                if (pentero)
                {
                    cursegIm = acq->getCurrentSegmFrame();
                }

                if (cursegIm.cols>0)
                {
                    imshow("Ready", cursegIm);
                }
                else
                    imshow("Ready", im);

                //fullscreen. comment to facilitate debugging
                setWindowProperty("Ready", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
                cursegIm.release();

                //im.release();
            }
            else if (mode != STANDBY)
            {
                destroyWindow("Ready");
            }

            previous_mode = mode;

            //clear up thread
            readingThread.join();
            readingThread.detach();



        }

        // make sure the thread is not working forever
        acq->thread = false;

        AcquisitionThread.join();
        AcquisitionThread.detach();

        FrameAcquisition.join();
        FrameAcquisition.detach();


    } else {
        ui->throwError("Connection not OK");
    }

    // restart the application instead of having a recursive function. this clears up memory as well
    // threads are not being terminated if program is stopped using the close button (top right)
    qApp->quit();
    QProcess::startDetached(qApp->arguments()[0], qApp->arguments());
    //startup(ui);

}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // create UI
    MainWindow w;
    w.show();

    // initialization
    w.ready(false);
    w.error(false);
    w.init();
    // send updater as argument of the thread in order to be able to update the UI from within the thread
    // For some weird reason, QT does not allow the UI to be updated from outside the source class, without using SIGNALS and SLOTS
    boost::thread startupThread(startup, w.updater);

    return a.exec();
}
