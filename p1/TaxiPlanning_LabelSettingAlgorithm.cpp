#include "TaxiPlanning_LabelSettingAlgorithm.h"
#include "globalvariable.h"
#include "twindow.h"
#include "TaxiPlanningAlgorithm.h"
#include <algorithm>
#include <assert.h>
#include "common.h"
#include "operation.h"
#include <QDebug>
#include <QElapsedTimer>
#include <QCoreApplication>
#include "vehicle.h"
#include <QFile>
#include <QTextStream>

#define __DEBUG_AVG_TIMEWINDOW__

#define __DEBUG_FILE__
#define __DEBUG_COMPUTETIME__
//#define __DEBUG_COMPARE_LS__

/****全局变量****/
CLabelCollect g_LabColl;
CLSNodeCollect g_LSNodeColl;
v_PathLS     g_vector_path_ls;

CLabel::CLabel(uint in_id, double in_ts, double in_te, CLabel* in_pre)
{
    this->m_nodeid = in_id;
    this->ts = in_ts;
    this->te = in_te;
    this->pPreLabel = in_pre;
}

bool CLabel::Dominate(CLabel* in_pLab)
{
    assert(this->m_nodeid == in_pLab->m_nodeid);    //用于比较的两个CLabel必须属于同一个node
    if(this->m_zoneid != in_pLab->m_zoneid){    //若所在区域不同，也不认为是dominated的
        return false;
    }
    else if((this->ts <= in_pLab->ts) && (this->te >= in_pLab->te)){
        return true;
    }
//    if((this->ts <= in_pLab->ts) && (this->te >= in_pLab->te)){
//            return true;
//    }
    else{
        return false;
    }
}

void CLabelCollect::Initialize()
{
    this->m_H.clear();
    this->m_C.clear();
}

void CLabelCollect::Reset()
{
    this->m_H.clear();
    this->m_C.clear();
}

CLabel* CLabelCollect::GetMin()
{
    CLabel* pout;
    pout = this->m_H.at(0);
    double cost = pout->ts + g_array_h[pout->m_nodeid]/g_speed;
    double tmptime;
    double tmpcost;
    uint index = 0;
    CLabel* ptmp;
    uint ndid;
    for(uint i=1; i < this->m_H.size();i++){
        ptmp = this->m_H.at(i);
        ndid = ptmp->m_nodeid;
        tmptime = ptmp->ts;
        tmpcost = tmptime + g_array_h[ndid]/g_speed;
        if(cost > tmpcost){
            pout = this->m_H.at(i);
            cost = tmpcost;
            index = i;
        }
    }
    this->m_H.erase(this->m_H.begin()+index);   //将它从H中取出,并加入C
    this->m_C.push_back(pout);
    return pout;
}

void CLSNodeCollect::Initialize()
{
    this->m_LSNodeVector.clear();
    this->m_count = g_model.m_ndct.m_count_node;
    for(uint i=0; i< this->m_count; i++){
        CLSNode* plsn = new CLSNode;
        plsn->m_id = i;
        this->m_LSNodeVector.push_back(plsn);
    }
}

void CLSNodeCollect::Reset()
{
    CLSNode* plsn;
    for(uint i=0; i<this->m_count; i++){
        plsn = this->m_LSNodeVector.at(i);
        plsn->m_LabelVector.clear();
    }
}

CLSNode* CLSNodeCollect::GetLSNodebyID(uint in_nodeid)
{
    assert(in_nodeid < this->m_LSNodeVector.size());
    return this->m_LSNodeVector.at(in_nodeid);
}

uint GetZoneIDBetweenTwoNodes(uint in_nd1, uint in_nd2)
{
    cnode* pnd1 = g_model.m_ndct.m_array_node[in_nd1];
    cnode* pnd2 = g_model.m_ndct.m_array_node[in_nd2];
    uint z11 = pnd1->m_nbzone[0];
    uint z12 = pnd1->m_nbzone[1];
    uint z21 = pnd2->m_nbzone[0];
    uint z22 = pnd2->m_nbzone[1];
    if((z11 == z21) || (z11==z22)){
        return z11;
    }
    else{
        assert((z12==z21) || (z12 == z22));
        return z12;
    }
}

uint GetZoneIDbyTwoNodes(uint in_nd1, uint in_nd2)
{
    cnode* pnd1 = g_model.m_ndct.m_array_node[in_nd1];
    cnode* pnd2 = g_model.m_ndct.m_array_node[in_nd2];
    uint z11 = pnd1->m_nbzone[0];
    uint z12 = pnd1->m_nbzone[1];
    uint z21 = pnd2->m_nbzone[0];
    uint z22 = pnd2->m_nbzone[1];
    uint comz;
    if((z11 == z21) || (z11==z22)){
        comz = z11;
    }
    else{
        assert((z12==z21) || (z12 == z22));
        comz = z12;
    }
    return g_model.m_ndct.m_array_node[in_nd2]->GetAnotherZoneId(comz);
}

void ResetGlobalVariables_LS()
{
    g_iterationcount = 0;   //扩展次数
    g_accesscount = 0;      //可达性判定的次数
    g_LabColl.Reset();      //
    g_LSNodeColl.Reset();   //
}

void UpdateTimeWindowVector_LS(Chain_LS& in_path, CRegionCollect &rct)
{
    twindow tw;
    uint rngid;
    uint ndid;
    CRegion* prgn;
    char dir;
    assert(in_path.size() > 1);
    CLabel* pb_s1;
    CLabel* pb_s2;
    //double delta = 0.2;       //用于防止同时资源交换现象的出现

    uint n = in_path.size();    //for debug
    assert(n>2);
    pb_s2 = in_path.at(n-1);
    tw.tend = pb_s2->ts;        //因为从最后一个Label对应节点进入的区域为stand或buffer，暂时不考虑其占用冲突
    for(int i = n-2; i>=0; i--){ //backward
        tw.tstart = pb_s2->tin_from_pre;
        pb_s1 = in_path.at(i);
        rngid = GetZoneIDBetweenTwoNodes(pb_s1->m_nodeid,pb_s2->m_nodeid);
        ndid = pb_s1->m_nodeid;   //进入区域的node号
        prgn = rct.GetRegion(rngid);
        if(prgn->m_type == Line){

            //if(prgn->m_id == 47 && fabs(tw.tstart - 365558.90)<0.01){
            //    int x = 1;
            //}
            dir =prgn->GetDirection(ndid);
            RemoveTimewindow_Line(prgn->GetTWVector(1-dir),&tw);
            UpdateOccupation(prgn->GetTWVector(dir), &tw, prgn->m_num_capacity);

        }
        else{//对Runway类似Inter进行处理（独占）
            RemoveTimewindow_Inter(prgn->GetTWVector(), &tw);
        }

        tw.tend = tw.tstart;
        pb_s2 = pb_s1;
        //stand和buffer暂不处理
    }
}

int  SequencePlan_LS()
{
    std::vector<uint> v_dif_id;
    ReadDiffFile(v_dif_id, "..\\Analysis_compare\\Pudong_without_ors\\diff_drt_ls.txt");
    //ReadDiffFile(v_dif_id, "..\\Analysis_compare\\Lukou\\diff_drt_ls.txt");
    //ReadDiffFile(v_dif_id, "..\\Analysis_compare\\Heathrow\\diff_drt_ls.txt");
    uint cur_dif_index = 0;

    g_LabColl.Initialize();     //初始化Label Collect对象
    g_LSNodeColl.Initialize();  //初始化LSNode Collect对象
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;             //current agent's planning priority
    Vehicle* veh;
    for(uint i=0;i<g_vehs.size();i++){
        Chain_LS chain;           //定义路径变量
        veh = g_vehs.at(i);
        g_veh_id = veh->m_id;

//        if(g_priority%2000 == 0){//每两千次循环清理一次时间窗
//            TimeWindowSlimming(veh->m_start_time);
//        }

#ifdef __DEBUG_AVG_TIMEWINDOW__
           double avg_count = ComputeAverageTimeWindow();
           g_avg_timewindows_vector.push_back(avg_count);   //平均每个区域包含的时间窗个数
#endif

        double t1 = GetHighPrecisionCurrentTime();          //获取系统当前时间
        int returnvalue = Plan_LS(chain, veh);                   //为当前车辆规划路径
        double t2 = GetHighPrecisionCurrentTime();          //获取系统当前时间
        if(returnvalue == 0){   //如果成功找到路径：
            Path_LS path;
            Convert(chain,path);//将chain_ls类型转换为path_ls类型
            g_vector_path_ls.push_back(path);               //将vehicle的path存入全局变量

#ifdef  __DEBUG_COMPARE_LS__
            if((cur_dif_index < v_dif_id.size()) && (veh->m_id == v_dif_id.at(cur_dif_index))){
                SavePathToFile_LS(veh->m_id, path,QString("..\\data_out\\LS_path_of_vehicle%1.txt").arg(veh->m_id));
                ResetGlobalVariables_LS();
                ResetGlobalVariables(); //当前航空器规划完毕后复位必要的全局变量：
                Path p;
                Plan(p, veh);
                SavePathToFile(veh->m_id, p, QString("..\\data_out\\DRT_path_of_vehicle%1.txt").arg(veh->m_id));
                cur_dif_index++;
            }
#endif

            UpdateTimeWindowVector_LS(chain, g_model.m_rct);    //（成功为当前规划航空器找到可行路径后）更新区域的时间窗约束
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);    //打印路径规划完成的提示信息
            g_flag_failure.push_back(0);    //表示成功找到路径
//根据需要，记录需要的运算结果和数据//
#ifdef __DEBUG_FILE__
//            if((veh->m_id == 6123)){
//                SavePathToFile_LS(veh->m_id, path, QString("..\\data_out\\ORIGINAL_LS_path_of_vehicle%1.txt").arg(veh->m_id));
//            }
//            if(veh->m_id == 6122){
//                QString file = QString("..\\data_out\\ORIGINAL_LS_twindowaftertarget_%1.txt").arg(veh->m_id);
//                PrintTimeWindows(file, g_model.m_rct);//for debug
//                QString file2 = QString("..\\data_out\\ORIGINAL_LS_occvectoraftertarget_%1.txt").arg(veh->m_id);
//                PrintOccupationVector(file2,g_model.m_rct);
//            }
#endif

#ifdef __DEBUG_COMPUTETIME__
            ctvector.push_back(t2-t1);                      //求解时间
            g_starting_wait_vector.push_back(path.at(0).m_time - veh->m_start_time);  //在起始位置等待的时间
            g_accesscount_vector.push_back(g_accesscount);  //可达性判定的次数
            itr_cnt_vector.push_back(g_iterationcount);     //扩展次数
#endif

#ifdef __DEBUG_PATHSTEPCOUNT__
            slvector.push_back(path.size());                //路由步数
#endif

        }
        else{   //如果没有找到路径
            Path_LS path;
            g_vector_path_ls.push_back(path);
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
            g_flag_failure.push_back(1);    //表示未能找到可行路径
#ifdef __DEBUG_COMPUTETIME__
            ctvector.push_back(t2-t1);
            g_starting_wait_vector.push_back(-1);
            g_accesscount_vector.push_back(g_accesscount);
            itr_cnt_vector.push_back(g_iterationcount);
#endif
#ifdef __DEBUG_PATHSTEPCOUNT__
            slvector.push_back(-1);//路由步数
#endif
        }

        ResetGlobalVariables_LS();  //当前航空器规划完毕后复位必要的全局变量
        g_priority++;               //denote agent's planning priority
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){       //线程休眠一段时间以降低CPU占用率
            QCoreApplication::processEvents();
        }
    }


/******************全部规划完毕后：***********************************/
    CheckOccupationVector(g_model.m_rct);   //检查占用向量中是否存在错误
    ConflictDetection_LS();                    //检查是否存在冲突
    SimutaneousExchangeDetection();         //检测是否存在同时资源交换现象
    ResetTimeWindows(g_model.m_rct);        //复位时间窗约束
    delete[] g_array_h;                     //释放启发值数组
/******************************************************************/

    return 0;
}

int Plan_LS(Chain_LS &out_path, Vehicle *veh)
{
    int status = 1;             //0表示搜索成功，1表示失败

    double exp_time = 0;
    double t1, t2;

    g_start_region = veh->m_start_region;
    g_start_node = veh->m_start_node;
    g_start_time = veh->m_start_time;
    g_end_region = veh->m_end_region;
    g_end_node = veh->m_end_node;
    Heuristics(g_end_node);     //启发值更新
    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
            ->GetAnotherZoneId(g_start_region);         //第一个滑行道区域编号
    twindow ew(g_start_time, c_timemax);
    CLabel* pLab = new CLabel(g_start_node,ew.tstart,ew.tend,0);    //根据输入信息创建第一个Label
    pLab->tin_from_pre = 0; //???
    pLab->m_zoneid = veh->m_start_region;
    g_LabColl.m_H.push_back(pLab);                      //将Label加入H
    g_LSNodeColl.GetLSNodebyID(g_start_node)->m_LabelVector.push_back(pLab);
    CLabel* pLabMin;
    t1 = GetHighPrecisionCurrentTime(); //为了计算“扩展过程”的耗时
    while(!g_LabColl.m_H.empty()){
        g_iterationcount++;                             //扩展次数加1

        uint n = g_LabColl.m_H.size();


        pLabMin = g_LabColl.GetMin();                   //从H中取出key值最小的Label
        //qDebug() << g_iterationcount << ": n=" << n << "  node:" << pLabMin->m_nodeid;;
        //g_LabColl.m_C.push_back(pLabMin);               //将其放入C中
        if(pLabMin->m_nodeid == g_end_node){
            Chain_LS path;
            path.push_back(pLabMin);
            do{
                pLabMin = pLabMin->pPreLabel;
                path.push_back(pLabMin);
            }while(pLabMin->m_nodeid != g_start_node);
            uint n = path.size();
            for(uint i=0; i<n;i++){
                out_path.push_back(path.at(n - 1 - i)); //调正顺序
            }
            status =  0;        //denote path found
            break;
        }
        //确定即将进入的区域编号
        uint zone_next;
        if(pLabMin->pPreLabel == 0){
            zone_next = first_taxiway_region;
        }
        else{
            //zone_next = GetZoneIDbyTwoNodes(pLabMin->pPreLabel->m_nodeid,
            //                                pLabMin->m_nodeid); //注意两个node参数的顺序
            zone_next = g_model.m_ndct.m_array_node[pLabMin->m_nodeid]->GetAnotherZoneId(pLabMin->m_zoneid);
        }
        if(zone_next == 1000){
            continue;
        }
        CRegion* prgn = g_model.m_rct.GetRegion(zone_next);
        if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
            continue;
        }   //prgn若是buffer或stand类型直接返回，因为这两种类型的区域没有其他出口


        uint id_neighbornode;

        for (uint i=0; i<prgn->m_count_node; i++){
            id_neighbornode = prgn->m_vector_node.at(i);
            if(id_neighbornode == pLabMin->m_nodeid){
                continue;
            }
            else if(g_model.matrix_nn_conn[pLabMin->m_nodeid][id_neighbornode] == 0){
                continue;
            }
            else{
                //根据区域的不同类型做不同处理
                if(prgn->m_type == Line){   //lane类型
                    twindow ew(pLabMin->ts, pLabMin->te);   //将Label的interval转换为ew类型
                    int dir = prgn->GetDirection(pLabMin->m_nodeid);
                    double tenter;
                    uint awid;
                    if(EnterTime_Line(prgn->GetTWVector(dir),&ew,prgn->m_num_capacity,tenter,awid)){
                        double nnlength = g_model.matrix_nn_dis[pLabMin->m_nodeid][id_neighbornode];
                        double speed = g_speed;
                        twindow ew_next;

                        if(ExitWindow_Line(tenter,prgn->GetTWVector(dir).at(awid),prgn->m_num_capacity,nnlength,&ew_next,speed)){
                            v_Label v_lab;                  //用于保存所有被新的Label dominate的现有Label
                            CLabel* pLabNext = new CLabel(id_neighbornode,ew_next.tstart,ew_next.tend,pLabMin);
                            pLabNext->tin_from_pre = tenter;
                            pLabNext->m_zoneid = zone_next;
                            uint flag = 0;
                            foreach(CLabel* pLabTmp, g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector){
                                if(pLabTmp->Dominate(pLabNext)){
                                    delete pLabNext;
                                    pLabNext = 0;
                                    flag = 1;               //表示pLabNext被dominate
                                    break;
                                }
                                else if(pLabNext->Dominate(pLabTmp)){
                                    v_lab.push_back(pLabTmp);
                                }
                            }
                            if(flag != 1){
                                if(!v_lab.empty()){
                                    foreach(CLabel* pLabDom, v_lab){    //将每个被pLabNext donimate的Label从H和L(id_neighbornode)中清除，然后释放内存
                                        g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.erase(remove(g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.begin(),
                                                                                                                g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.end(),pLabDom),g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.end());
                                        g_LabColl.m_H.erase(remove(g_LabColl.m_H.begin(),
                                                                   g_LabColl.m_H.end(),pLabDom),g_LabColl.m_H.end());
                                        delete pLabDom;
                                    }
                                }
                                if(pLabNext != 0){  //如果没被dominate，将其加入H和L(node)
                                    g_LabColl.m_H.push_back(pLabNext);
                                    g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.push_back(pLabNext);
                                }
                            }
                        }
                    }
                }
                else{//intersection或runway类型
                    twindow ew(pLabMin->ts, pLabMin->te);   //将Label的interval转换为ew类型
                    uint paircount = 0;
                    WindowTimeTable wtt;
                    if(Overlapped(&ew, prgn->GetTWVector(), paircount, wtt)){
                        double nnlength = g_model.matrix_nn_dis[pLabMin->m_nodeid][id_neighbornode];
                        double speed = g_speed;
                        if(prgn->m_type == Runway){
                            speed = speed*5;
                        }
                        WindowTimeTable::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            uint awid = itr->first;
                            double tenter = itr->second;
                            twindow ew_next;
                            if(ExitWindow_Intersection(tenter, nnlength, prgn->GetTWVector().at(awid), &ew_next, speed)){
                                v_Label v_lab;      //用于保存所有被新的Label dominate的现有Label
                                CLabel* pLabNext = new CLabel(id_neighbornode,ew_next.tstart,ew_next.tend,pLabMin);
                                pLabNext->tin_from_pre = tenter;
                                pLabNext->m_zoneid = zone_next;
                                uint flag = 0;
                                foreach(CLabel* pLabTmp, g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector){
                                    if(pLabTmp->Dominate(pLabNext)){
                                        delete pLabNext;
                                        pLabNext = 0;
                                        flag = 1;   //表示pLabNext被dominate
                                        break;
                                    }
                                    else if(pLabNext->Dominate(pLabTmp)){
                                        v_lab.push_back(pLabTmp);
                                    }
                                }
                                if(flag != 1){
                                    if(!v_lab.empty()){
                                        foreach(CLabel* pLabDom, v_lab){    //将每个被pLabNext donimate的Label从H和L(id_neighbornode)中清除，然后释放内存
                                            g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.erase(remove(g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.begin(),
                                                                                                                    g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.end(),pLabDom),g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.end());
                                            g_LabColl.m_H.erase(remove(g_LabColl.m_H.begin(),
                                                                       g_LabColl.m_H.end(),pLabDom),g_LabColl.m_H.end());
                                            delete pLabDom;
                                        }
                                    }
                                    if(pLabNext != 0){  //如果没被dominate，将其加入H和L(node)
                                        g_LabColl.m_H.push_back(pLabNext);
                                        g_LSNodeColl.GetLSNodebyID(id_neighbornode)->m_LabelVector.push_back(pLabNext);
                                    }
                                }
                            }
                        }

                    }
                }
            }
        }
    }
    t2 = GetHighPrecisionCurrentTime();     //为了计算“扩展过程”的耗时
    exp_time = t2-t1;                       //扩展过程耗时
    g_expandtime_vector.push_back(exp_time);//将扩展过程耗时加入记录

    return status;

}

void PrintTaxiTimeAndDistance_LS(QString in_timefile, QString in_disfile)
{
    QFile file_time(in_timefile);
    if(!file_time.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << QString("创建文件%1失败").arg(in_timefile);
        return;
    }
    QTextStream ts_time(&file_time);
    QFile file_dis(in_disfile);
    if(!file_dis.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << QString("创建文件%1失败").arg(in_disfile);
        return;
    }
    QTextStream ts_dis(&file_dis);
    double time;
    double dis;
    CPathStep pbs;
    CPathStep pbs_2;
    uint nd;
    uint nd2;
    Path_LS path;
    for(uint i=0; i<g_vector_path_ls.size();i++){
        path = g_vector_path_ls.at(i);
        if(path.size()>0){   //为了增加对比实验而引入的判断
            pbs = path.at(0);
            pbs_2 = path.at(path.size()-1);
            time = pbs_2.m_time - pbs.m_time;
            ts_time << i+1 << '\t' << time << '\n';
            assert(path.size() > 2);//通常步骤数肯定大于2
            dis = 0;//距离清零
            for(uint j=0; j<path.size()-1; j++){
                pbs = path.at(j);
                pbs_2 = path.at(j+1);
                nd = pbs.m_nodeid;
                nd2 = pbs_2.m_nodeid;
                dis += g_model.matrix_nn_dis[nd][nd2];
            }
            ts_dis << i+1 << '\t' << dis << '\n';
        }
        else{   //为了增加对比实验而引入的处理(未能找到可行路径)
            ts_time << i+1 << '\t' << -1 << '\n';
            ts_dis << i+1 << '\t' << -1 << '\n';
        }

    }
    file_time.close();
    file_dis.close();
}

void Convert(Chain_LS& in_chain, Path_LS& out_path)
{
    CLabel* pb_s1;
    CLabel* pb_s2;
    uint n = in_chain.size();    //for debug
    assert(n>2);
    pb_s1 = in_chain.at(0);
    uint nd1 = pb_s1->m_nodeid;
    pb_s2 = in_chain.at(1);
    double tstart = pb_s2->tin_from_pre;
    CPathStep ps;
    ps.m_nodeid = nd1;
    ps.m_time = tstart;
    out_path.push_back(ps);

    for(uint i = 2; i<n; i++){
        pb_s1 = pb_s2;
        nd1 = pb_s1->m_nodeid;
        pb_s2 = in_chain.at(i);
        tstart = pb_s2->tin_from_pre;
        CPathStep ps;
        ps.m_nodeid = nd1;
        ps.m_time = tstart;
        out_path.push_back(ps);
    }

    CPathStep ps_end;
    ps_end.m_nodeid = pb_s2->m_nodeid;
    ps_end.m_time = pb_s2->ts;  //
    out_path.push_back(ps_end);
}

void SavePathToFile_LS(uint &in_id, Path_LS& in_path, QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("error, failed to create path info file");
        return;
    }
    else{
        QTextStream ts(&file);
        //Path::iterator itr;
        //Basicstep bs;
        QString strline;
        /*for(itr = in_path.begin(); itr!=in_path.end();itr++){
            bs = itr->second;
            strline = QString("%1 %2 %3").arg(itr->first)
                    .arg(GetNodeIndex(bs)).arg(GetRegionIndex(bs));
            ts << strline << '\n';
        }*/
        CPathStep pb_s;
        uint startzone = g_vehs.at(in_id-1)->m_start_region;
        uint nextzone;
        for(uint i=0; i<in_path.size();i++){
            pb_s = in_path.at(i);
            nextzone = g_model.m_ndct.m_array_node[pb_s.m_nodeid]->GetAnotherZoneId(startzone);
            strline = QString("%1 %2 %3").arg(pb_s.m_time,0,'f',3)
                    .arg(pb_s.m_nodeid).arg(nextzone);
            ts << strline << '\n';
            startzone = nextzone;
        }
        file.close();
    }
}

void ExtractOccupancyInfo_LS()
{
    double t1;
    double t2;
    uint zone1;
    uint zone2;
    uint in_zone;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    Vehicle* pveh;
    CPathStep pbs;
    CRegion* prgn;
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        in_zone = pveh->m_start_region;
        Path_LS path = g_vector_path_ls.at(i);
        //由于增加实验对比而引入的判断
        if(path.size() == 0){
            continue;
        }
        //------------------------
        pbs = path.at(0);
        t1 = pbs.m_time;
        ndin = pbs.m_nodeid;
        zone1 = in_zone;
        for(uint j=1; j<path.size();j++){
            pbs = path.at(j);
            t2 = pbs.m_time;
            zone2 = g_model.m_ndct.m_array_node[ndin]->GetAnotherZoneId(zone1);
            ndout = pbs.m_nodeid;
            occ_info occinfo;
            occinfo.m_id = pveh->m_id;
            occinfo.m_time_start = t1;
            occinfo.m_time_end = t2;
            occinfo.m_nd_in = ndin;
            occinfo.m_nd_out = ndout;
            prgn = g_model.m_rct.GetRegion(zone2);
            if(prgn->m_type == Line){
                occinfo.m_direction = (ndin < ndout) ? 0 : 1;//占用方向
            }
            prgn->m_occinfo_vector.push_back(occinfo);
            zone1 = zone2;
            t1 = t2;
            ndin = ndout;
        }
    }
}

void ConflictDetection_LS()
{
    ExtractOccupancyInfo_LS();
    CheckOccupancyInfo(g_model.m_rct);
    ClearOccupancyInfo(g_model.m_rct);//检测完后清空占用信息，以允许再次检测
    qDebug() << "conflict detection finished.";
}

void PrintFinishTime_LS(QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << QString("创建文件%1失败").arg(filename);
        return;
    }
    QTextStream ts(&file);
    ts.setRealNumberNotation(QTextStream::FixedNotation);
    ts.setRealNumberPrecision(3);
    uint size;

    for(uint i=0; i<g_vector_path_ls.size(); i++){
        Path_LS path = g_vector_path_ls.at(i);
        size = path.size();
        CPathStep pbs = path.at(size-1);
        ts << i+1 << '\t'
           << pbs.m_time << '\n';
    }
    file.close();
}
