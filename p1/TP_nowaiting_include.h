//by Tianci Zhang

#ifndef TP_NOWAITING_INCLUDE_H
#define TP_NOWAITING_INCLUDE_H

#include "nowaiting_step.h"

class Vehicle;
typedef  std::map<uint, twindow>  AW_ENTW_table_nowaiting;//available time window index and entry time window list 扩展过程中使用
typedef  std::vector<nowaiting_step*>  NowaitingStepVector;


//const uint TAXI = 0;
//const uint LAND = -1;
//const uint TAKEOFF = 1;

extern Vehicle* g_pveh;

#endif // TP_NOWAITING_INCLUDE_H
