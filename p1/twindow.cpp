//by Tianci Zhang

#include "twindow.h"


twindow::~twindow()
{
    for(uint i=0;i<m_occ_vector.size();i++){
        delete m_occ_vector.at(i);
    }
    m_occ_vector.clear();
}

twindow::twindow(double ts, double te)
{
    tstart = ts;
    tend = te;
    m_occ_vector.clear();
    m_inout_vector.clear();
}

OccVector& twindow::GetOccVector()
{

    return m_occ_vector;
}

InOutVector& twindow::GetInOutVector()
{
    return m_inout_vector;
}
