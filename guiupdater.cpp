#include "guiupdater.h"
#include <QDebug>

void GUIupdater::setMode(int mode)
{
    emit requestNewMode(mode);
}
