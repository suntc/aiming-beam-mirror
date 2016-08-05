#include "imageacquisition.h"
#include <QDebug>

imageAcquisition::imageAcquisition()
{


}

void imageAcquisition::startAcquisition()
{
    // startup camera
    VideoInput *cam = new VideoPointGrey();

    while (thread)
    {
        if (ctrl)
        {

            frame = cam->getNextFrame();
            imshow("Manual focus", frame);
            waitKey(10);
            if (!ctrl)
                destroyWindow("Manual focus");
        }
    }

    //destroyAllWindows();

    // close camera
    cam->disconnect();
    delete(cam);

}
