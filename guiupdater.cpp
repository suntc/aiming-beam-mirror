#include "guiupdater.h"
#include <QDebug>

void GUIupdater::setMode(int mode)
{
    emit requestNewMode(mode);
}

void GUIupdater::setReady(bool status)
{
    emit requestReady(status);
}

void GUIupdater::setError(bool status)
{
    emit requestError(status);
}

void GUIupdater::throwError(std::string msg)
{
    emit outputError(msg);
}

void GUIupdater::setCameraStatus(bool usb_1, bool usb_2, bool fg_1, bool fg_2)
{
    emit cameraStatus(usb_1, usb_2, fg_1, fg_2);
}
