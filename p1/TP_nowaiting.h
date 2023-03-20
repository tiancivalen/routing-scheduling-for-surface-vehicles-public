// taxi planning with MTT (on certain regions)

#ifndef TP_ROBUST_NOWAITING_H
#define TP_ROBUST_NOWAITING_H
#include "twindow.h"
#include "cregioncollect.h"
#include "TP_nowaiting_include.h"

//declarations & definitions:



////OccVariable和OccVector的nowaiting版本
//class OccVariable_nowaiting
//{
//public:
//    inline OccVariable_nowaiting(twindow in_entw, uint in_count=0, int in_action=0, OccVariable_nowaiting* in_pair = 0, int in_id=-1)
//    {
//        m_entw = in_entw;
//        m_count = in_count;
//        m_action = in_action;
//        m_pair = in_pair;
//        m_id = in_id;
//    }
//public:
//    twindow m_entw;//NTOA,占用发生变化的时刻
//    uint m_count;//占用/释放动作发生后的占用个数
//    int m_action;//1:in（占用）,-1:out（释放）,0:Unknown（未知动作）
//    OccVariable_nowaiting* m_pair;//与当前对象成对出现的另一个rOccVariable的指针
//    int m_id;//占用航空器的id
//};
//typedef std::vector<OccVariable_nowaiting*>  OccVector_nowaiting;



// functions:

void Experiment_nowaiting();
int SequencePlan_nowaiting();          //Robust顺序规划方法
int Plan_nowaiting(Vehicle *veh);      //Robust TP 算法
int Plan_nowaiting_loopless(Vehicle *veh);
void Expand_nowaiting(nowaiting_step *in_bs);
void Expand_nowaiting_loopless(nowaiting_step *in_bs);
bool Closed_nowaiting(Basicstep in_bs, NowaitingStepVector& rsv);//这里为了实现multipath，允许同一个basestep对应的多个robuststep存在
bool Opened_nowaiting(Basicstep in_bs, std::vector<uint> &rsv);
void genTBST_nowaiting(Vehicle* pveh);//根据nowaiting_step信息生成time-based surface trajectory
void GetMinHeuristicCostOfOpenlist_nowaiting(nowaiting_step *&out_bs);//
void GetMinHeuristicCostOfOpenlist_nowaiting_deepfirst(nowaiting_step *&out_bs);//

void printResult_nowaiting(QString filename);
void UpdateTimeWindowVector_nowaiting(Path &in_path, CRegionCollect &rct);
bool Overlapped_nowaiting(twindow* in_ptw, TWVector& in_twv, uint& out_count, AW_ENTW_table_nowaiting& out_wtt);
bool ExitWindow_Intersection_nowaiting(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
bool EntryWindow_Line_nowaiting(TWVector& in_twv, twindow* in_ew, uchar in_cap, double in_speed, twindow &out_entw, uint& out_awid);
bool ExitWindow_Line_nowaiting(twindow in_entw,twindow* in_aw, uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed, uchar hold);
void RemoveTimewindow_Line_nowaiting(TWVector& twv, twindow* in_ptw);
void UpdateOccupation_nowaiting(TWVector& twv, nowaiting_step* pb_s, nowaiting_step* pb_e, uint in_cap);
void RemoveTimewindow_Inter_nowaiting(TWVector& twv, twindow* in_ptw);
bool EntryWindow_Intersection_nowaiting(twindow* in_ptw1, twindow* in_ptw2, twindow& out_entw);

void PathFeasibilityCheck_nowaiting();
void CheckOccupationVector_nowaiting(CRegionCollect &rct);
void ConflictDetection_nowaiting();
void ExtractOccupancyInfo_nowaiting();
void CheckOccupancyInfo_nowaiting(CRegionCollect& rct);
void SimutaneousExchangeDetection_nowaiting();
void ClearOccupancyInfo_nowaiting(CRegionCollect& rct);
void HoldDetection_nowaiting();

double getSpeed_nowaiting(uint rgnid, uint action, const int isTurn);//0表示正常滑行，1表示起飞，-1表示着陆
uint getAction_nowaiting(uint srgnid, uint sndid, uint ergnid);//根据起、止位置信息判断执行的动作类型

void printPathDetail_nowaiting(Path& rpath, QString fdir);

#endif // TP_ROBUST_NOWAITING_H
