#ifndef TP_NOWAITING_ORIG_H
#define TP_NOWAITING_ORIG_H

#include "cregioncollect.h"
#include "TP_nowaiting_include.h"


void Experiment_nowaiting_orig();
int SequencePlan_nowaiting_orig();          //Robust顺序规划方法
int Plan_nowaiting_orig(Vehicle *veh);      //Robust TP 算法
void Expand_nowaiting_orig(nowaiting_step *in_bs);
void Expand_nowaiting_loopless_orig(nowaiting_step *in_bs);
bool Closed_nowaiting_orig(Basicstep in_bs);//这里为了实现multipath，允许同一个basestep对应的多个robuststep存在
bool Opened_nowaiting_orig(Basicstep in_bs, nowaiting_step*& out_pbs);
void GetMinHeuristicCostOfOpenlist_nowaiting_orig(nowaiting_step *&out_bs);//
void printResult_nowaiting_orig(QString filename);
void UpdateTimeWindowVector_nowaiting_orig(Path &in_path, CRegionCollect &rct);
bool Overlapped_nowaiting_orig(twindow* in_ptw, TWVector& in_twv, uint& out_count, WindowTimeTable& out_wtt);
bool ExitWindow_Intersection_nowaiting_orig(double in_tenter, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
bool ExitWindow_Line_nowaiting_orig(double in_tenter,twindow* in_aw, uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed, uchar hold);
void RemoveTimewindow_Line_nowaiting_orig(TWVector& twv, twindow* in_ptw);
void UpdateOccupation_nowaiting_orig(TWVector& twv, nowaiting_step* pb_s, nowaiting_step* pb_e, uint in_cap);
void RemoveTimewindow_Inter_nowaiting_orig(TWVector& twv, twindow* in_ptw);

bool EnterTime_Intersection_nowaiting_orig(twindow* in_ptw1, twindow* in_ptw2, double& out_tenter);
bool EnterTime_Line_nowaiting_orig(TWVector& in_twv, twindow* in_ew, uchar in_cap, double in_speed, double &out_tenter, uint& out_awid);

void PathFeasibilityCheck_nowaiting_orig();
void CheckOccupationVector_nowaiting_orig(CRegionCollect &rct);
void ConflictDetection_nowaiting_orig();
void ExtractOccupancyInfo_nowaiting_orig();
void CheckOccupancyInfo_nowaiting_orig(CRegionCollect& rct);
void SimutaneousExchangeDetection_nowaiting_orig();
void ClearOccupancyInfo_nowaiting_orig(CRegionCollect& rct);
void HoldDetection_nowaiting_orig();

double getSpeed_nowaiting_orig(uint rgnid, uint action, const int in_isTurn);//0表示正常滑行，1表示起飞，-1表示着陆
uint getAction_nowaiting_orig(uint srgnid, uint sndid, uint ergnid);//根据起、止位置信息判断执行的动作类型

void printPathDetail_nowaiting_orig(Path& rpath, QString fdir);


int Plan_nowaiting_orig(Vehicle *veh, Path& in_path);
int Plan_nowaiting_loopless_orig(Vehicle *veh, Path& in_path);

void printPathDetail_nowaiting_orig(Path& rpath, QString fdir, QString fname);

void updateCost_orig(nowaiting_step *pns, double weight=0);//

int Plan_nowaiting_orig_homogeneous(Vehicle *veh, Path& in_path);
void Expand_nowaiting_orig_homogeneous(nowaiting_step *in_bs);
bool EnterTime_Line_nowaiting_orig(TWVector& in_twv, twindow* in_ew, double &tenter, uint& out_awid);
bool ExitWindow_Line_nowaiting_orig(double in_tenter, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
void savePartialSolutionPaths(QString filename, std::vector<base_step *> &nsv);
void saveTimeWindows(QString filename, CRegionCollect &rct);

//for revision
int Plan_nowaiting_orig_homogeneous_runwayBlockTime(Vehicle *veh, Path& in_path);
void Expand_nowaiting_orig_homogeneous_runwayBlockTime(nowaiting_step *in_bs);
bool ExitWindow_Intersection_nowaiting_orig_runwayBlockTime(double in_time, double in_nnlength,uint in_act, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
void printPathDetail_nowaiting_orig_runwayBlockTime(Path& rpath, QString fdir, QString fname);

#endif // TP_NOWAITING_ORIG_H
