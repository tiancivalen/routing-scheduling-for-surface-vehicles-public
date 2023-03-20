#ifndef ITERATIONTHREAD_H
#define ITERATIONTHREAD_H

#include <QThread>

class IterationThread : public QThread
{
    Q_OBJECT
public:
    explicit IterationThread(QObject *parent = 0);
    virtual void run();
signals:
    void SendData(int in_info);
public slots:
    
};

#endif // ITERATIONTHREAD_H
