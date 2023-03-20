#ifndef TP_ROBUST_H
#define TP_ROBUST_H

/****2015年12月23日新增（RobustTP）****/

#include <QString>
#include "cregioncollect.h"
#include "twindow.h"
#include "cnode.h"
#include "base_step.h"
#include "robust_step.h"



class Vehicle;


// main interface
void Experiment_Robust();
int SequencePlan_robust();                          //Robust顺序规划方法
int Plan_robust(rPath& out_path, Vehicle *veh);      //Robust TP 算法
////int SequencePlan_robust_standholding();//考虑stand holding时的顺序规划方法
////int Plan_robust_reverse(rPath& out_path, Vehicle *veh);//用于departing aircraft，反向搜索


//for result analysis
void printParam(QString filename);//将程序的参数设置情况输出到文件
void printResult(QString filename);


//auxiliary functions
void Expand_robust(robust_step* in_bs);
bool Opened_robust(Basicstep in_bs, robust_step*& out_b_s);
void UpdateTimeWindowVector_robust(rPath& in_path, CRegionCollect &rct);
void SavePathToFile_robust(uint id,rPath& rpath, QString fname);
void ConflictDetection_robust();
bool Overlapped_robust(twindow* in_ptw,double in_ioa, TWVector& in_twv, uint& out_count, WindowTimeTable& out_wtt);
bool ExitWindow_Intersection_robust(double in_time, double in_ioa, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed);
bool EnterTime_Intersection_robust(twindow* in_ptw1, double in_ioa, twindow* in_ptw2, double& out_tmin);
bool ExitWindow_Line_robust(const double in_tenter,double in_ioa, twindow* in_aw, uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed);
bool EnterTime_Line_robust(TWVector& in_twv, twindow* in_ew, double in_ioa, uchar in_cap, double in_speed, double &out_tenter, uint& out_awid);
double ntoa(double ioa, double ioa_start);//根据IOA和TS(IOA)计算NTOA
double ioaStart(double ioa, double ntoa);//根据IOA和NTOA计算IOA的起始时刻，即TS(IOA)
double ioaEnd(double ioa, double ntoa);
void RemoveTimewindow_Line_robust(TWVector& twv, twindow* in_ptw);
void UpdateOccupation_robust(TWVector& twv, robust_step *pb_s, robust_step *pb_e, uint in_cap);
void RemoveTimewindow_Inter_robust(TWVector& twv, twindow* in_ptw);
double getSpeed(uint rgnid, uint action);//0表示正常滑行，1表示起飞，-1表示着陆
uint getAction(uint srgnid, uint sndid, uint ergnid);//根据起、止位置信息判断执行的动作类型
double getIOA(double in_ioa, double in_len, double in_speed, double k);//根据长度、速度和比例因子计算IOA长度
void adjustArrivalRunwayOccupancy(Vehicle* pveh);//因为跑道上降落时速度很快不允许停止等待，所以这里要调整进港航空器的跑道占用时间窗，将跑道上的等待时间移至air buffer中
void GetMinHeuristicCostOfOpenlist_robust(robust_step *&out_bs);//
double calCost(double len, double speed, double duration);//根据距离、速度和运行时间计算（当前滑行步骤的）cost

//for stand holding of departing aircraft
////double CalcRunwayArrivalTime(Vehicle* pveh);

//postprocessing
void CheckOccupationVector_robust(CRegionCollect &rct);
void ConflictDetection_robust();
void SimutaneousExchangeDetection_robust();
void ExtractOccupancyInfo_robust();
void CheckOccupancyInfo_robust(CRegionCollect& rct);
void ClearOccupancyInfo_robust(CRegionCollect& rct);
void PathFeasibilityCheck();//检查已经分配的路径是否符合滑行道的物理结构
void printPathDetail(rPath &rpath, QString fdir);

//最后的内存清理
void releaseMemory();

#endif // TP_ROBUST_H
