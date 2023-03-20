//by Tianci Zhang

#ifndef BASE_STEP_H
#define BASE_STEP_H
#include "common.h"

class base_step
{
public:
    base_step();
    base_step(Basicstep in_bs);
    base_step(Basicstep in_bs, double in_time);
    base_step(uint in_rgnid, uint in_ndid, uint in_awid, double in_time);
    ~base_step();
public:
    Basicstep m_bs;
    double m_entrytime;
    base_step* m_prestep;//previous base_step along the path
};

#endif // BASE_STEP_H
