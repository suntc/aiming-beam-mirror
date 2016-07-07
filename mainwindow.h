#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "guiupdater.h"

#include <QMainWindow>

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
    GUIupdater *updater = new GUIupdater();
    ~MainWindow();

private:
    Ui::MainWindow *ui;

public slots:
    void setMode(int mode);

};

#endif // MAINWINDOW_H
