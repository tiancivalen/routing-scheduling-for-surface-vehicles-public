#include "FileOperation.h"

#include <QDir>
void GetFileNames(QString in_filepath, QString in_filter_suffix, QString in_filter_name, std::vector<QString>& out_filenames)
{
    QDir dir(in_filepath);
    QStringList strlist;
    strlist << in_filter_suffix;
    QFileInfoList fil = dir.entryInfoList(strlist);
    QString str;
    foreach(QFileInfo fileinfo, fil){
        str = fileinfo.fileName();
        if(str.contains(in_filter_name)){
            out_filenames.push_back(str);
        }
    }
}
