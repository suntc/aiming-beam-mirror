#ifndef CAMERA_H
#define CAMERA_H

#include <C:\Program Files\IDS\uEye\Develop\include\uEye.h>

class camera
{
private:
    HIDS hCam = 0;
    bool isConnected = 0;
public:
    camera();
    ~camera();
    void setup();
    //void acquire(IS_RECT camAOI, char* imgMem, int memId);
    bool is_connected();
};

#endif // CAMERA_H
