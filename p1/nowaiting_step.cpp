//by Tianci Zhang

#include "nowaiting_step.h"
#include <QDebug>

nowaiting_step::nowaiting_step()
{
    m_entw.tstart = 0;
    m_entw.tend = 0;
    m_prestep = NULL;
    m_cost = 0;
    m_startstep = NULL;
}

nowaiting_step::nowaiting_step(Basicstep in_bs, double in_time)
{
    m_bs = in_bs;
    m_entrytime = in_time;
    m_prestep = NULL;
    m_cost = 0;
    m_startstep = NULL;
}

nowaiting_step::nowaiting_step(Basicstep in_bs, twindow in_entw, double in_len, double in_sp)
{
    m_bs = in_bs;
    m_entw = in_entw;
    m_length = in_len;
    m_speed = in_sp;
    m_prestep = NULL;
    m_cost = 0;
    m_startstep = NULL;
}

nowaiting_step::nowaiting_step(nowaiting_step* in_pns)
{
    m_bs = in_pns->m_bs;
    m_entw = in_pns->m_entw;
    m_length = in_pns->m_length;
    m_speed= in_pns->m_speed;
    m_cost = in_pns->m_cost;
    m_entrytime = in_pns->m_entrytime;
    m_preExitTW = in_pns->m_preExitTW;
}

bool nowaiting_step::Dominate(nowaiting_step* prs)
{
    if(prs->m_bs != m_bs){
        qDebug() << QString("Attention: The base steps are not comparable.");
        return false;
    }
    if((m_entw.tstart < prs->m_entw.tstart+1e-3 && m_entw.tend > prs->m_entw.tend-1e-3))
    {
        if(m_cost<(prs->m_cost+1e-3))
//        if(this->m_startstep == prs->m_startstep)
            return true;
        else
            return false;
    }
    else{
        return false;
    }
}
