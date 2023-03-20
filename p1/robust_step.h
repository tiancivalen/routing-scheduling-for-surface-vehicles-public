#ifndef ROBUST_STEP_H
#define ROBUST_STEP_H
#include "base_step.h"


class robust_step : public base_step
{
public:
    inline robust_step(Basicstep in_bs, double in_tenter, double in_ioa,
                       double in_cost_time,double in_cost_delay)
    {
        m_bs = in_bs;
        m_entrytime = in_tenter;
        m_IOA = in_ioa;
        m_cost_time = in_cost_time;
        m_cost_delay = in_cost_delay;
    }
    //为了向后兼容:
    inline robust_step(Basicstep in_bs,double in_tenter,double in_ioa,
                       double in_cost)
    {
        m_bs = in_bs;
        m_entrytime = in_tenter;
        m_IOA = in_ioa;
        m_cost = in_cost;
    }

    bool Dominate(robust_step* prs);//判断当前对象是否dominate输入的对象
public:
    double m_IOA;//IOA的长度
    double m_cost_time;//从起始位置到达当前位置的总time
    double m_cost_delay;//从起始位置到达当前位置的总delay
    double m_cost;
};

typedef  std::vector<robust_step*>  rPath;//robust path structure
typedef  std::vector<robust_step*>  RobustStepVector;

#endif // ROBUST_STEP_H
