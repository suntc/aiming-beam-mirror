#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>
#include <QCloseEvent>
#include <QMessageBox>
#include <boost/filesystem.hpp>
#include <QProcess>
#include <iostream>
#include <QtMultimedia/QSound>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    // Needed to use string as outputError argument. check QMetaType docs
    qRegisterMetaType<std::string>();

    QObject::connect(updater, SIGNAL(requestNewMode(int)), this, SLOT(setMode(int)));
    QObject::connect(updater, SIGNAL(requestReady(bool)), this, SLOT(ready(bool)));
    QObject::connect(updater, SIGNAL(requestError(bool)), this, SLOT(error(bool)));
    QObject::connect(updater, SIGNAL(outputError(std::string)), this, SLOT(log(std::string)));
    QObject::connect(updater, SIGNAL(cameraStatus(bool,bool,bool,bool)), this, SLOT(camera(bool, bool, bool, bool)));
}

void MainWindow::init()
{

    // initialize as offline
    offline(true);

    // initialize cameras UI indicators
    camera(false, false, false, false);
}

void MainWindow::setMode(int mode)
{
    // set all to false
    offline(false);
    standby(false);
    test_exvivo(false);
    acquisition_exvivo(false);
    test_invivo(false);
    acquisition_invivo(false);

    switch (mode)
    {
    case -1:    // offline
        offline(true);
        break;
    case 0:     // standby
        standby(true);
        break;
    case 1:    // test
        test_exvivo(true);
        break;
    case 2:    // test
        test_invivo(true);
        break;
    case 3:    // acquisition ex vivo
        acquisition_exvivo(true);
        break;
    case 4:    // acquisition in vivo
        acquisition_invivo(true);
        break;
    }

}

void MainWindow::offline(bool status)
{
    if (status)
    {
        ui->offline_btn->setStyleSheet("background-color: green");
        ui->offline_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->offline_btn->setStyleSheet("background-color: red");
        ui->offline_label->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::camera(bool usb_1, bool usb_2, bool fg_1, bool fg_2)
{
    // top usb camera
    QString ss = (usb_1) ? "green" : "red";
    ui->USB_1_ready->setStyleSheet("background-color: " + ss);

    // side usb camera
    ss = (usb_2) ? "green" : "red";
    ui->USB_2_ready->setStyleSheet("background-color: " + ss);

    // frame grabber 1
    ss = (fg_1) ? "green" : "red";
    ui->FG_1_ready->setStyleSheet("background-color: " + ss);

    // frame grabber 2
    ss = (fg_2) ? "green" : "red";
    ui->FG_2_ready->setStyleSheet("background-color: " + ss);

}

void MainWindow::standby(bool status)
{
    if (status)
    {
        ui->standby_btn->setStyleSheet("background-color: green");
        ui->standby_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->standby_btn->setStyleSheet("background-color: red");
        ui->standby_label->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::test_exvivo(bool status)
{
    if (status)
    {
        ui->test_exvivo_btn->setStyleSheet("background-color: green");
        ui->test_exvivo_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->test_exvivo_btn->setStyleSheet("background-color: red");
        ui->test_exvivo_label->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::acquisition_exvivo(bool status)
{
    if (status)
    {
        ui->acquisition_exvivo_btn->setStyleSheet("background-color: green");
        ui->acquisition_exvivo_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->acquisition_exvivo_btn->setStyleSheet("background-color: red");
        ui->acquisition_exvivo_label->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::test_invivo(bool status)
{
    if (status)
    {
        ui->test_invivo_btn->setStyleSheet("background-color: green");
        ui->test_invivo_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->test_invivo_btn->setStyleSheet("background-color: red");
        ui->test_invivo_label->setStyleSheet("font-weight:normal");
    }
}


void MainWindow::acquisition_invivo(bool status)
{
    if (status)
    {
        ui->acquisition_invivo_btn->setStyleSheet("background-color: green");
        ui->acquisition_invivo_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->acquisition_invivo_btn->setStyleSheet("background-color: red");
        ui->acquisition_invivo_label->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::ready(bool status)
{
    if (status)
    {
        ui->ready_btn->setStyleSheet("background-color:green;");
        ui->ready_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->ready_btn->setStyleSheet("background-color:rgb(197,197,197);");
        ui->ready_btn->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::error(bool status)
{
    if (status)
    {
        ui->error_btn->setStyleSheet("background-color:red;");
        ui->error_label->setStyleSheet("font-weight:bold");
    }
    else
    {
        ui->error_btn->setStyleSheet("background-color:rgb(197,197,197);");
        ui->error_btn->setStyleSheet("font-weight:normal");
    }
}

void MainWindow::log(std::string msg)
{
    ui->log_text->appendPlainText(QString::fromStdString(msg));
}


void MainWindow::closeEvent (QCloseEvent *event)
{
    QMessageBox::StandardButton resBtn = QMessageBox::question( this, "Aiming beam",
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
        event->accept();
        qApp->quit();
    }
}


MainWindow::~MainWindow()
{
    delete ui;
}
