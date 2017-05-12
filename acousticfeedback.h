#ifndef ACOUSTICFEEDBACK_H
#define ACOUSTICFEEDBACK_H
//#include <windows.h>
//#include <Windows.h>
#include <chrono>
#include <thread>
#include <QObject>
#include <QtMultimedia/QSound>
#include "laguerredeconvolution.h"

class AcousticFeedback: public QObject
{
Q_OBJECT
public slots:
    void feedback();
public:
    AcousticFeedback();
    //~AcousticFeedback();
    void start_feedback();
    void stop_feedback();
    bool running = true;
    double threshold_low_signal = 0.14;
    LaguerreDeconvolution * decon = NULL;
    void setDecon(LaguerreDeconvolution * decon);

private:
    void feedback_start(unsigned int interval);
};

#endif // ACOUSTICFEEDBACK_H
