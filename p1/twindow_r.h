//by Tianci Zhang

#ifndef TIMEWINDOW_R_H
#define TIMEWINDOW_R_H
#include <QtGlobal>
#include <vector>

class rOccVariable
{
public:
    inline rOccVariable(double in_time=0, uint in_count=0, double in_ioa=0, int in_action=0, rOccVariable* in_pair = 0, int in_id=-1)
    {
        m_time = in_time;
        m_ioa = in_ioa;
        m_count = in_count;
        m_action = in_action;
        m_pair = in_pair;
        m_id = in_id;
    }
public:
    double m_time;//NTOA,占用发生变化的时刻
    uint m_count;//占用/释放动作发生后的占用个数
    double m_ioa;//对应的IOA
    int m_action;//1:in（占用）,-1:out（释放）,0:Unknown（未知动作）
    rOccVariable* m_pair;//与当前对象成对出现的另一个rOccVariable的指针
    int m_id;
};
typedef std::vector<rOccVariable*>  rOccVector;



class twindow_r
{
public:
    twindow_r();
    ~twindow_r();

public:
    rOccVector m_occ_vector_robust;
};

#endif // TIMEWINDOW_R_H
