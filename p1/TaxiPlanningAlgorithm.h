#ifndef TAXIPLANNINGALGORITHM_H
#define TAXIPLANNINGALGORITHM_H
#include <QString>
#include "cregioncollect.h"
#include "twindow.h"
#include "cnode.h"

class Vehicle;

void ParsePlanFile(QString planfilename);
int  SequencePlan();//the basic sequential planning
void Heuristics(uint in_target_node);
double StraightLineDistance(cnode* ndfrom, cnode* ndto);//line distance
double rad(double degree);
int  Plan(Path& out_path, Vehicle *veh);
void UpdateTimeWindowVector(Path& in_path, CRegionCollect &rct);
void ResetGlobalVariables();
bool EnterTime_Line(TWVector& in_twv, twindow* in_ew, uchar in_cap, double &out_tenter, uint& out_awid);

bool Overlapped(twindow* in_ptw,TWVector& in_twv, uint& out_count, WindowTimeTable& out_wtt);
bool EnterTime_Intersection(twindow* in_ew, twindow* in_aw, double &out_tenter);
void GetMinHeuristicCostOfOpenlist(base_step *&out_bs);

uint GetRegionIndex(Basicstep in_bs);
uint GetNodeIndex(Basicstep in_bs);
uint GetAWindowIndex(Basicstep in_bs);
quint64 TernaryToUnary(uint n1, uint n2, uint n3);//zone id, node id, aw id
void TernaryToUnary(uint n1, uint n2, uint n3, quint64& out_bs);

void Expand(base_step *in_bs);
bool ExitWindow_Intersection(double in_time, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed);
bool ExitWindow_Line(const double in_tenter, twindow* in_aw, uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed);
bool Closed(Basicstep in_bs);
bool Opened(Basicstep in_bs, base_step*& out_b_s);

void RemoveTimewindow_Line(TWVector& twv, twindow* in_ptw);
void UpdateOccupation(TWVector& twv, twindow* in_ptw, uint in_cap);
void RemoveTimewindow_Inter(TWVector& twv, twindow* in_ptw);

void TimeWindowSlimming(double in_cur_time);


#endif // TAXIPLANNINGALGORITHM_H
