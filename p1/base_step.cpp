//by Tianci Zhang

#include "base_step.h"
#include "utils.h"


base_step::base_step()
{
}

base_step::base_step(Basicstep in_bs)
{
    m_bs =in_bs;
}

base_step::base_step(Basicstep in_bs, double in_time)
{
    m_bs = in_bs;
    m_entrytime = in_time;
}

base_step::base_step(uint in_rgnid, uint in_ndid, uint in_awid, double in_time)
{
    m_bs = TernaryToUnary(in_rgnid, in_ndid, in_awid);
    m_entrytime = in_time;
}

base_step::~base_step()
{

}
