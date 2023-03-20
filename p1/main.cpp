//by Tianci Zhang

#include "mainwindow.h"
#include <QApplication>
#include <QTextCodec>
#include "globalvariable.h"
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
    QTextCodec::setCodecForCStrings(codec);
    MainWindow w;
    w.show();
    g_config.Read("..\\config.txt");//read in the simulation configuration file
    g_model.Initialize(g_config.flag_ors);//called once for fixed airport model
    return a.exec();

}

