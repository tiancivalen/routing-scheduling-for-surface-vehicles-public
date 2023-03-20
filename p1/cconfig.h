//by Tianci Zhang

#ifndef CCONFIG_H
#define CCONFIG_H
#include <qglobal.h>
enum HeuFlag{sd,euclidean,zero};
class CConfig
{
public:
    CConfig();
    void Read(QString in_file);
public:
    uint flag_ors;
    HeuFlag flag_heuristic;
};

#endif // CCONFIG_H
