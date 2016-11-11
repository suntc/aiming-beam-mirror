#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "guiupdater.h"
#include <QMainWindow>

// needed to use string as argument in signal-slot thingy
Q_DECLARE_METATYPE(std::string)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    void init();
    void standby(bool status);
    void test_exvivo(bool status);
    void acquisition_exvivo(bool status);
    void test_invivo(bool status);
    void acquisition_invivo(bool status);
    void offline(bool status);
    void closeEvent(QCloseEvent *event);
    GUIupdater *updater = new GUIupdater();
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void setMode(int mode);
    void ready(bool status);
    void error(bool status);
    void log(std::string msg);


};

#endif // MAINWINDOW_H
