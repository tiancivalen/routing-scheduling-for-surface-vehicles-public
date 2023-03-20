//by Tianci Zhang

#ifndef COMMON_H
#define COMMON_H
#include <map>
#include <vector>
#include <QtGlobal>
class    twindow;
class    cnode;
class    base_step;


typedef  quint64                        Basicstep;//note region-node-aw as a single number
typedef  std::vector<base_step*>        Path;
typedef  std::vector<base_step*>        Closelist;
typedef  std::vector<base_step*>        OpenlistForExpand;//扩展过程中使用

typedef  std::map<uint, double>         WindowTimeTable;//aw and t-enter list 扩展过程中使用
typedef  std::vector<twindow*>          TWVector;// time window vector
typedef  std::pair<double, int>         OccVariable;//占用发生变化的时刻和变化后的占用个数
typedef  std::vector<OccVariable*>      OccVector;
typedef  std::vector<char>              InOutVector;
typedef  std::vector<double>            ComputationTimeVector;
typedef  std::vector<int>               StepLengthVector;
typedef  std::vector<cnode*>            NodeVector;
typedef  std::vector<int>               IterationCountVector;

typedef struct s_occupancy
{
    int m_id;
    int m_direction;//占用方向，只对lane有意义
    double m_time_start;
    double m_time_end;
    uint m_nd_in;//进入节点
    uint m_nd_out;//脱离节点
} occ_info;
typedef  std::vector<occ_info>          OccInfoVector;



#endif // COMMON_H
