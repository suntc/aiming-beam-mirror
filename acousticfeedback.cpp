#include "acousticfeedback.h"
#include <Windows.h>

AcousticFeedback::AcousticFeedback()
{

}

void AcousticFeedback::feedback_start(unsigned int interval)
{
    // start acoustic feedback

    running = true;
    AcousticFeedback* self = this;

    // start feedback in new thread so that it doesnt impact the main thread
    std::thread([interval, self]() {
        while (self->running)
        {
            self->feedback();
            std::this_thread::sleep_for(std::chrono::milliseconds(interval));
        }
    }).detach();
}

void AcousticFeedback::stop_feedback()
{
    // stop thread creation
    running = false;
}

void AcousticFeedback::start_feedback()
{
    feedback_start(600);
}

void AcousticFeedback::feedback()
{
    // make sure that deconvolution object exists
    if (decon)
    {
       double ind = decon->getAcousticIndicator();
       if (ind<threshold_low_signal)
           Beep(700,300);
    }
}

void AcousticFeedback::setDecon(LaguerreDeconvolution * decon)
{
    // set pointer to deconvolution object
    this->decon = decon;
}

