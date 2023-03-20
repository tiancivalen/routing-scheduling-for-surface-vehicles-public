#ifndef FILEOPERATION_H
#define FILEOPERATION_H
#include <vector>
#include <QString>
void GetFileNames(QString in_filepath, QString in_filter_suffix, QString in_filter_name, std::vector<QString>& out_filenames);
#endif // FILEOPERATION_H
