#ifndef GUIUPDATER_H
#define GUIUPDATER_H
#include <QObject>

class GUIupdater : public QObject {
    Q_OBJECT

public:
    explicit GUIupdater(QObject *parent = 0) : QObject(parent) {}
    void setMode(int mode);

private:
    Q_DISABLE_COPY(GUIupdater)

signals:
    void requestNewMode(int);
};
#endif // GUIUPDATER_H
