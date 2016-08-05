#include "mainwindow.h"
#include <QApplication>
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
#include "imageacquisition.h";

#define OFFLINE -1
#define STANDBY 0
#define TEST_EXVIVO 1
#define TEST_INVIVO 2
#define ACQUISITION_EXVIVO 3
#define ACQUISITION_INVIVO 4

#define STARTUP 0
#define INIT_CAMERA 1
#define INIT_IRF 2
#define IDLE 3
#define BUSY 4
#define COMPLETE 5
#define SERROR 6
#define ABORT 7

#define TERMINATOR "\r\n"
#define SEPARATOR ":"

using namespace std;
using namespace boost::filesystem;
using namespace cv;

void StartRead(TCP_IP &obj, string *output)
{
    if (obj.isOpen())
    {
        obj.read(output);
    }
}

void StartAcquire(imageAcquisition *acq)
{
    acq->startAcquisition();
}

void buildWriteCmd(char cmd[], int value)
{

    // integer to char
    char valueToStr[1];
    sprintf(valueToStr,"%d", value);

    // concatenate cmd & val & \r\n (for comm)
    strcat(cmd, strcat(valueToStr, "\r\n"));
}

char* set_ack (string key, string value)
{
    string cmd = key + SEPARATOR + value + TERMINATOR;

    char cmd_[1024];
    strncpy(cmd_, cmd.c_str(), sizeof(cmd_));
    cmd_[sizeof(cmd_) - 1] = 0;

    return cmd_;
}

void startup(GUIupdater *ui)
{


    const char *ip = "1.1.1.1";
    const char *port = "2100";
    bool init = true;

    // initialise application mode
    int mode = OFFLINE;
    ui->setMode(mode);

    qDebug() << "Warming up...";

    // initialize TCP/IP communication
    TCP_IP conn (ip, port);

    // make sure we are alive
    conn.write("alive\r\n");

    // check connection
    if (conn.isOpen()){

        // connected. in standby mode
        mode = STANDBY;
        ui->setMode(mode);

        // initialize vars to manage communications
        string output;
        string key;
        string value;

        // other vars
        string subject;     // case ID no.
        bool aiming_beam_correction = false;    // automatic correction of the aiming beam
        int aiming_beam_value = 0;  // aiming beam power (0 to 100)
        bool invivo = false;    // ex vivo false, in vivo true

        float time_resolution = 0;  // digitizer time resolution, value in ns
        int data_length = 0;    // length of the fluorescence data in each channel?
        int irf_length = 0;     // length of the IRF in each channel

        bool focus = false;     // if true, manual focus is on and camera is streaming
        bool acquire = false;

        // startup camera
        //VideoInput *cam = new VideoPointGrey();

        // initialize object
        imageAcquisition *acq = new imageAcquisition();
        acq->thread = true; // control acquisition thread
        boost::thread AcquisitionThread(StartAcquire, acq);
        while (true){

            // have the read command within a thread, so the communication is always opened
            boost::thread readingThread(StartRead, conn, &output);

            // strip output \r\n
            output = output.substr(0, output.length() - 2);

            // get key and value from output
            size_t f = output.find(":");

            if (f != std::string::npos)
            {
                // get key (string before :)
                key = output.substr(0, f);

                // get value (string after :), always as double or int
                //  +1 to account for the :
                value = output.substr(f + 1, output.length() - f + 1);

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
                //qDebug() << output;

                // send acknowledgment
                conn.write("disconnect:1\r\n");

                // give it some time before disconnecting
                Sleep(500);

                // disconnect from this end
                conn.disconnect();

                // wait so that master can close connection
                Sleep(500);

                break;
            }



            // setting subject id
            else if (key.compare("!subject") == 0)
            {

                subject = value;

                // send acknowledgment
                conn.write("test\r\n");
                //conn.write(set_ack(key, subject));

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
                aiming_beam_value = stoi(value);
                conn.write(set_ack(key, value));
            }

            // digitizer time resolution
            else if (key.compare("!resolution") == 0)
            {

                time_resolution = stod(value);

                // send acknowledgment
                conn.write(set_ack(key, value));

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

            }

            // query mode
            else if (key.compare("?mode") == 0)
            {

                char cmd[64] = "mode:";
                buildWriteCmd(cmd, mode);

                // send acknowledgment
                conn.write(cmd);

                // read all parameters. for debug purposes
                //qDebug() << subject.c_str();
                //qDebug() << time_resolution;
                //qDebug() << aiming_beam_value;
                //qDebug() << irf_length;
                //qDebug() << data_length;
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
                        qDebug() << "To disconnect";

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

            // set acquisition loop condition
            acq->ctrl = focus;

            // check current working mode
            mode = (focus) ?
                        ((invivo) ? TEST_INVIVO : TEST_EXVIVO)
                      :
                        STANDBY;
            ui->setMode(mode);

            readingThread.join();

        }

        // make sure the thread is not working forever
        acq->thread = false;
        AcquisitionThread.join();
        AcquisitionThread.detach();

    } else {
        qDebug() << "Connection not OK";
    }

    startup(ui);
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    // create UI
    MainWindow w;
    w.show();

    // initialization
    w.init();


    // make things working
    // send updater as argument of the thread in order to be able to update the UI from within the thread
    // For some weird reason, QT does not allow the UI to be updated from outside the source class, without using SIGNALS and SLOTS
    boost::thread startupThread(startup, w.updater);

    return a.exec();
}
