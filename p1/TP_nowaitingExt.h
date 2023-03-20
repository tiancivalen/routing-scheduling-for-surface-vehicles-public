//taxi planning with MTT (on certain regions), extended with waiting time penalisation
//by Tianci Zhang

#ifndef TP_ROBUST_NOWAITINGEXT_H
#define TP_ROBUST_NOWAITINGEXT_H
#include "twindow.h"
#include "cregioncollect.h"
#include "TP_nowaiting_include.h"

class TP_nowaitingExt
{
public:
    double m_weight;

public:
    TP_nowaitingExt(double weight=0);

private:
    bool Closed(Basicstep in_bs, NowaitingStepVector& rsv);
    bool Opened(Basicstep in_bs, std::vector<uint> &rsv);
    void genTBST(Vehicle* pveh);
    void GetMinHeuristicCostOfOpenlist(nowaiting_step *&out_bs);//
    void GetMinHeuristicCostOfOpenlist_deepfirst(nowaiting_step *&out_bs);//

    void printResult(QString filename);
    bool Overlapped(twindow* in_ptw, TWVector& in_twv, uint& out_count, AW_ENTW_table_nowaiting& out_wtt);
    bool ExitWindow_Intersection(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
    //bool EntryWindow_Line(TWVector& in_twv, twindow* in_ew, uchar in_cap, double in_speed, twindow &out_entw, uint& out_awid);
    //bool ExitWindow_Line(twindow in_entw,twindow* in_aw, uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed, uchar hold);
    void RemoveTimewindow_Line(TWVector& twv, twindow* in_ptw);
    void UpdateOccupation(TWVector& twv, nowaiting_step* pb_s, nowaiting_step* pb_e, uint in_cap);
    void RemoveTimewindow_Inter(TWVector& twv, twindow* in_ptw);
    bool EntryWindow_Intersection(twindow* in_ptw1, twindow* in_ptw2, twindow& out_entw);

    void PathFeasibilityCheck();
    void SimutaneousExchangeDetection();
    void ClearOccupancyInfo(CRegionCollect& rct);
    void HoldDetection();

    double getSpeed(uint rgnid, uint action, const int in_isTurn);
    uint getAction(uint srgnid, uint sndid, uint ergnid);

    void printPathDetail(Path& rpath, QString fdir);

    void updateCost(nowaiting_step* pns);


    void TimeWindowSlimming(double in_cur_time);
    void ParsePlanFile(QString planfilename);
    void ResetGlobalVariables();
    void Heuristics(uint in_target_node);
    quint64 TernaryToUnary(uint n1, uint n2, uint n3);
    void TernaryToUnary(uint n1, uint n2, uint n3, quint64& out_bs);
    uint GetRegionIndex(Basicstep in_bs);
    uint GetNodeIndex(Basicstep in_bs);
    uint GetAWindowIndex(Basicstep in_bs);
    double StraightLineDistance(cnode* ndfrom, cnode* ndto);
    double rad(double degree);


    // added since 11-Oct-2017
public:

    bool EntryWindow_Line(TWVector& in_twv, twindow* in_ew, twindow &out_entw, uint& out_awid);
    bool ExitWindow_Line(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable);
    void UpdateTimeWindowVector_homogeneous(Path &in_path, CRegionCollect &rct);
    void ConflictDetection_homogeneous();
    void ExtractOccupancyInfo_homogeneous();
    void CheckOccupancyInfo_homogeneous(CRegionCollect& rct);

    void Expand_loopless_homogeneous(nowaiting_step *in_bs);
    int Plan_homogeneous(Vehicle *veh);
    int SequencePlan_homogeneous();
    void Experiment_homogeneous();

    void printPathDetail(Path& rpath, QString fdir, QString fname);


};

#endif // TP_ROBUST_NOWAITING_H
