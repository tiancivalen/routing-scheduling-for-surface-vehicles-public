//by Tianci Zhang

#ifndef TP_ROBUST_GLOBALVARIABLES_H
#define TP_ROBUST_GLOBALVARIABLES_H
#include <QtGlobal>
#include "vehicle.h"

extern double g_ioa;
extern const double g_w;//平衡运行时间和延误的权重
extern const double g_w_delay;
extern double g_k;//IOA相对运行时间的百分比
extern const double g_len_plane;//飞机机身长度
extern const double g_sep;//最小安全间隔

extern double g_ratio_density;//密度缩放因数
extern double g_pathnumber;//指定的path个数
extern bool   g_flag_startdelay;
extern double g_ratio_range;//dominate判定时用到的那个比例因子
extern double g_frequency;//起降频率（单位:架次/分钟）

const uint TAXI = 0;
const uint LAND = -1;
const uint TAKEOFF = 1;


extern Vehicles g_vehs4comp;

extern uint g_WTSType;


#endif // TP_ROBUST_GLOBALVARIABLES_H
