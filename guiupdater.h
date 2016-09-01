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


private:
    Q_DISABLE_COPY(GUIupdater)

signals:
    void requestNewMode(int);
    void requestReady(bool);
    void requestError(bool);
    void outputError(std::string);
};
#endif // GUIUPDATER_H
