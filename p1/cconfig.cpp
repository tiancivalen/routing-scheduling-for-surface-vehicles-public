//by Tianci Zhang

#include "cconfig.h"
#include <QFile>
#include <QString>
#include <QDebug>
CConfig::CConfig()
{
    flag_ors = 0;//default: ors
    flag_heuristic = sd;//default: static shortest path distance
}

void CConfig::Read(QString in_file)
{
    QFile file(in_file);
    if(!file.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open config.txt";
        return;
    }
    QString line;
    QString ident;
    QString valu;
    while(!file.atEnd()){
        line = file.readLine();

        if(line.isEmpty()){
            continue;
        }
        else if(line.at(0) == '#'){
            continue;
        }
        else{
            qDebug() << line;
            ident = line.section('=',0,0);
            valu = line.section('=',1,1);

            if(ident == "ors"){
                if(valu == "no"){
                    flag_ors = 1;
                }
                else{
                    flag_ors = 0;
                }
            }
            else if(ident == "heuristic"){
                if(valu == "sd"){
                    flag_heuristic = sd;
                }
                else if(valu == "euclidean"){
                    flag_heuristic = euclidean;
                }
                else{
                    flag_heuristic = zero;
                }
            }
            else{
                qDebug() << QString("Undefined: %1").arg(ident);
            }
        }
    }
    file.close();
}
