#ifndef GUIUPDATER_H
#define GUIUPDATER_H
#include <QObject>

class GUIupdater : public QObject {
    Q_OBJECT

public:
    explicit GUIupdater(QObject *parent = 0) : QObject(parent) {}
    void setMode(int mode);
    void setReady(bool status);
    void setError(bool status);
    void throwError(std::string msg);
    void setCameraStatus(bool usb_1, bool usb_2, bool fg_1, bool fg_2);

private:
    Q_DISABLE_COPY(GUIupdater)

signals:
    void requestNewMode(int);
    void requestReady(bool);
    void requestError(bool);
    void outputError(std::string);
    void cameraStatus(bool, bool, bool, bool);
};
#endif // GUIUPDATER_H
