#include "StaticPath.h"
#include "globalvariable.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
void StaticPath()
{

    //读入plansequence文件，并确定每个航空器的最短路径长度，并将路径长度写入文件

    QFile planfile(g_model.file_dir+"sequenceplan.txt");
    if(!planfile.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "failed to open sequenceplan.txt";
    }
    QFile sdfile("..\\data_out\\shortest_path_length.txt");
    if(!sdfile.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << "failed to create shortest_path_length.txt";
    }
    QTextStream ts(&sdfile);
    uint startnd;
    uint endnd;
    QString strline;
    while(!planfile.atEnd()){
        strline = planfile.readLine();
        if(strline.isEmpty())
            continue;
        else if(strline.at(0) == '#')
            continue;
        else{
            ts << strline.section('\t',0,0)<< '\t';
            startnd = (uint)strline.section('\t',3,3).toInt();
            endnd = (uint)strline.section('\t',5,5).toInt();
            ts << g_model.matrix_nn_sd[startnd][endnd] << '\n';
        }
    }
    planfile.close();
    sdfile.close();

    qDebug() << "shortest path for each aircraft is calculated.";
}
