//by Tianci Zhang

#ifndef NOWAITING_STEP_H
#define NOWAITING_STEP_H
#include "base_step.h"
#include "twindow.h"

class nowaiting_step : public base_step
{
public:
    nowaiting_step();
    nowaiting_step(Basicstep in_bs, double in_time);
    nowaiting_step(Basicstep in_bs, twindow in_entw, double in_len=0, double in_sp=0);
    nowaiting_step(nowaiting_step* in_pns);
    bool Dominate(nowaiting_step* pbs);

public:
    twindow m_entw;
    double m_length;
    double m_speed;
    // 19-08-2017 added
    double m_cost;
    nowaiting_step* m_startstep;
    twindow m_preExitTW;
};

#endif // NOWAITING_STEP_H
