#ifndef TP_ROBUST_MULTIPLE_H
#define TP_ROBUST_MULTIPLE_H

#include <QString>
#include "cregioncollect.h"
#include "twindow.h"
#include "cnode.h"
#include "base_step.h"
#include "robust_step.h"

class Vehicle;


void Experiment_Robust_multiple();
int SequencePlan_robust_multiple();                          //Robust顺序规划方法
int Plan_robust_multiple(Vehicle *veh);      //Robust TP 算法
void Expand_robust_multiple(robust_step* in_bs);
void pathSelect(Vehicle* veh);//模拟管制员从候选path集合中选取其中一个的动作
bool Closed_robust_multiple(Basicstep in_bs, RobustStepVector& rsv);//这里为了实现multipath，允许同一个basestep对应的多个robuststep存在
bool Opened_robust_multiple(Basicstep in_bs, std::vector<uint> &rsv);
void adjustArrivalRunwayOccupancy_multiple(Vehicle* pveh);//因为跑道上降落时速度很快不允许停止等待，所以这里要调整进港航空器的跑道占用时间窗，将跑道上的等待时间移至air buffer中

void GetMinHeuristicCostOfOpenlist_robust_multiple(robust_step *&out_bs);//
double calCost_multiple(double time, double delay);//根据距离、速度和运行时间计算（当前滑行步骤的）cost

void printResult_multiple(QString filename);

void printAllPaths(Vehicle* veh, QString fdir);//打印所有路径的详细信息到文件
void printAbstractPathsInfo(Vehicle* veh, QString fdir);//打印所有路径的摘要信息到文件

// for compare
void Experiment_Robust_multiple4comp();
void ParsePlanFile4comp(QString planfilename);
int SequencePlan_robust_multiple4comp();
void printResult4comp(QString filename);

#endif // TP_ROBUST_MULTIPLE_H
