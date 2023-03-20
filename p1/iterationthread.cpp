#include "iterationthread.h"
#include "GeneticAlgorithm.h"
#include "operation.h"
#include "globalvariable.h"
#include <QMessageBox>

IterationThread::IterationThread(QObject *parent) :
    QThread(parent)
{
}

void IterationThread::run()
{
    bool flag = false;
    while(true){
        g_mutex.lock();
        if(b_ga_clicked){
            flag = true;
            b_ga_clicked = false;           
        }
        g_mutex.unlock();
        if(flag){
            flag = false;
            ExperimentGA();
            emit SendData(1);
        }
    }
}
