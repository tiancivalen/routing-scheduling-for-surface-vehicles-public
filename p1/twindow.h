//by Tianci Zhang

#ifndef TWINDOW_H
#define TWINDOW_H
#include <QtGlobal>
#include "common.h"
#include "twindow_r.h"
class twindow : public twindow_r
{
public:
    ~twindow();
    twindow(double ts=0, double te=0);
    OccVector& GetOccVector();
    InOutVector& GetInOutVector();
public:
    double tstart;
    double tend;
    OccVector m_occ_vector;
    InOutVector m_inout_vector;
};

#endif // TWINDOW_H
