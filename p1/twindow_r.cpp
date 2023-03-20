//by Tianci Zhang

#include "twindow_r.h"

twindow_r::twindow_r()
{
    //m_occ_vector_robust.clear();
}

twindow_r::~twindow_r()
{
    for(uint i=0;i<m_occ_vector_robust.size();i++){
        delete m_occ_vector_robust.at(i);
    }
    m_occ_vector_robust.clear();
}
