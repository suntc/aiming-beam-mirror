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
