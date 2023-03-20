//by Tianci Zhang

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "operation.h"
#include "globalvariable.h"
#include <QMessageBox>
#include "TP_nowaitingExt.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::ReceiveData(int in_info)
{

}


void MainWindow::on_actionTest_triggered()
{
    double weight = 1;
//    uint index = 1;
    TP_nowaitingExt alg(weight);
    alg.Experiment_homogeneous();
}



