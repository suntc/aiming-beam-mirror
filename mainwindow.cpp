#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    QObject::connect(updater, SIGNAL(requestNewMode(int)), this, SLOT(setMode(int)));

}

void MainWindow::init()
{

    // initialize as offline
    offline(true);
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

MainWindow::~MainWindow()
{
    delete ui;
}
