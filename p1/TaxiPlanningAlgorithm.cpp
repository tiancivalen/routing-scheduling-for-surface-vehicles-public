#include "TaxiPlanningAlgorithm.h"
#include "globalvariable.h"
#include <QFile>
#include <FileOperation.h>
#include <QDebug>
#include <math.h>
#include "operation.h"
#include <assert.h>
#include "base_step.h"
#include <QElapsedTimer>
#include <QCoreApplication>
#include "TP_robust.h"
#include "TP_robust_globalvariables.h"

#define __DEBUG_FILE__
#define __DEBUG_COMPUTETIME__
#define __DEBUG_PATHSTEPCOUNT__
#define __DEBUD_ITERATIONTIMES__
//#define __DEBUG_SUCCESSIONCHECK__
//#define __DEBUG_OCCUCHECK__
#define __DEBUG_AVG_TIMEWINDOW__
//#define __DEBUG_COMPARE_BS__


void ParsePlanFile(QString planfilename)
{
    g_vehs.clear(); //复位全局变量
    g_objcount = 0;

    QFile file(planfilename);
    if(!file.open(QFile::ReadOnly | QFile::Text)){
        qDebug("error, faied to open the plan file, please check");
        return;
    }
    else{
        QString strline;
        bool b;
        Vehicle* veh;
        g_vehs.clear();
        g_objcount = 0;
        while(!file.atEnd()){
            strline = file.readLine();
            if(strline.isEmpty()){//skip the empty line
                continue;
            }
            else if(strline.at(0) == '#'){//skip the comment line
                continue;
            }
            else{
                veh = new Vehicle;
                veh->m_id = strline.section('\t',0,0).toInt(&b);
                assert(b);
                veh->m_start_time = g_ratio_density*(strline.section('\t',1,1).toDouble(&b));
                //veh->m_start_time = (uint)strline.section('\t',1,1).toDouble(&b)/2; //divide by 3
                assert(b);
                veh->m_start_region = (uint)strline.section('\t',2,2).toInt(&b);
                assert(b);
                CRegion* prgn = g_model.m_rct.GetRegion(veh->m_start_region);
                if(prgn->m_type == Buffer){
                    veh->m_type = ARRIV;
                }
                else if(prgn->m_type == Stand){
                    veh->m_type = DEPAR;
                }
                else{
                    qDebug() << QString("Error: unknown vehicle type. Please check.");
                }
                veh->m_start_node = (uint)strline.section('\t',3,3).toInt(&b);
                assert(b);
                veh->m_end_region = (uint)strline.section('\t',4,4).toInt(&b);
                assert(b);
                veh->m_end_node = (uint)strline.section('\t', 5, 5).toInt(&b);
                assert(b);
                g_vehs.push_back(veh);
                g_objcount++;
            }
        }
        file.close();
    }
}



int SequencePlan()
{

    int returnvalue;
    //初始化启发值数组
    g_array_h = new double[g_model.m_ndct.m_count_node];
    g_priority = 1;//current agent's planning priority
    Vehicle* veh;
    //uint num = g_vehs.size();
    uint num = 1000;
    for(uint i=0;i<num;i++){
        veh = g_vehs.at(i);
        g_veh_id = veh->m_id;

//        if(g_priority%2000 == 0){//每两千次循环清理一次时间窗
//            TimeWindowSlimming(veh->m_start_time);
//        }

#ifdef __DEBUG_AVG_TIMEWINDOW__
           double avg_count = ComputeAverageTimeWindow();
           g_avg_timewindows_vector.push_back(avg_count);//平均每个区域包含的时间窗个数
#endif

        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan(veh->m_path, veh);
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue == 0){//如果成功找到路径：
            //更新时间窗
            UpdateTimeWindowVector(veh->m_path, g_model.m_rct);//（成功为当前规划航空器找到可行路径后）更新区域的时间窗约束
            //打印路径规划完成的提示信息
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);

//根据需要，记录需要的运算结果和数据//
#ifdef __DEBUG_FILE__
            //if((veh->m_id == 18)){
                //SavePathToFile(veh->m_id, veh->m_path,QString("..\\data_out\\ORIGINAL_DRT_path_of_vehicle%1.txt").arg(veh->m_id));
            //}
//            if(veh->m_id == 14884){
                //QString file = QString("..\\data_out\\ORIGINAL_DRT_twindowaftertarget_%1.txt").arg(veh->m_id);
                //PrintTimeWindows(file, g_model.m_rct);//for debug
//                QString file2 = QString("..\\data_out\\ORIGINAL_DRT_occvectoraftertarget_%1.txt").arg(veh->m_id);
//                PrintOccupationVector(file2,g_model.m_rct);
//            }


#endif

#ifdef __DEBUG_COMPUTETIME__
            ctvector.push_back(t2-t1);//求解时间
            g_starting_wait_vector.push_back(veh->m_path.at(0)->m_entrytime - veh->m_start_time);//在起始位置等待的时间
            g_accesscount_vector.push_back(g_accesscount);//可达性判定的次数
            itr_cnt_vector.push_back(g_iterationcount);//扩展次数

#endif

#ifdef __DEBUG_PATHSTEPCOUNT__
            slvector.push_back(veh->m_path.size());//路由步数
#endif


        }
        else{//如果没有找到路径
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }


        //当前航空器规划完毕后复位必要的全局变量：
        ResetGlobalVariables();
        //denote agent's planning priority
        g_priority++;
        //线程休眠一段时间以降低CPU占用率
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){
            QCoreApplication::processEvents();
        }
    }
    //全部规划完毕后：
    CheckOccupationVector(g_model.m_rct);//检查占用向量中是否存在错误
    ConflictDetection();//检查是否存在冲突
    SimutaneousExchangeDetection();//检测是否存在同时资源交换现象
    ResetTimeWindows(g_model.m_rct);//复位时间窗约束
    delete[] g_array_h;//释放启发值数组
    return 0;
}


void Heuristics(uint in_target_node)
{
    if(g_config.flag_heuristic == sd){
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            g_array_h[i] = g_model.matrix_nn_sd[i][in_target_node];//以静态最短路的长度为启发值,启发值要比真实值小
        }
    }
    else if(g_config.flag_heuristic == euclidean){
        cnode* ndtarget;
        cnode* pnd;
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            ndtarget = g_model.m_ndct.m_array_node[in_target_node];
            pnd = g_model.m_ndct.m_array_node[i];
            g_array_h[i] = StraightLineDistance(pnd, ndtarget);//启发值要比真实值小
        }
    }
    else{//不使用启发值
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            g_array_h[i] = 0;
        }
    }

}

double StraightLineDistance(cnode* ndfrom, cnode* ndto)
{
    double radLat1 = rad(ndfrom->m_y);
    double radLat2 = rad(ndto->m_y);
    double a = radLat1 - radLat2;
    double b = rad(ndfrom->m_x) - rad(ndto->m_x);
    double s = 2*asin(sqrt(pow(sin(a/2),2) + cos(radLat1)*cos(radLat2)*pow(sin(b/2),2)));
    s = s*EARTH_RADIUS;
    s = s*1000;//m
    return s;
}

double rad(double degree)
{
    return degree * PI / 180.0;
}

int Plan(Path& out_path, Vehicle* veh)
{
    double exp_time = 0;
    double t1, t2;

    g_start_region = veh->m_start_region;
    g_start_node = veh->m_start_node;
    g_start_time = veh->m_start_time;
    g_end_region = veh->m_end_region;
    g_end_node = veh->m_end_node;
    Heuristics(g_end_node);//启发值更新
    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
            ->GetAnotherZoneId(g_start_region);//第一个滑行道区域编号

    Basicstep bs;
    double tenter;
    uint rgnid;
    uint ndid;
    uint awid;
    ndid = g_start_node;
    rgnid = first_taxiway_region;
    twindow ew(g_start_time, c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the first taxiway zone

    /*tackle the first taxiway zone*/
    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
        WindowTimeTable wtt;
        uint num;
        if(!Overlapped(&ew, prgn->GetTWVector(),num,wtt)){//
            QString info = QString("error, the entertime of target %1 is not properly set").arg(veh->m_id);
            qDebug(info.toAscii());
            return 1;
        }
        else{
            WindowTimeTable::iterator itr;
            //uint awid;
            for(itr=wtt.begin();itr!=wtt.end();itr++){
                awid = itr->first;
                tenter = itr->second;
                bs = TernaryToUnary(rgnid, ndid, awid);
                base_step* b_s = new base_step(bs, tenter);
                openlistexp.push_back(b_s);
                base_step* b_s_pre = new base_step(TernaryToUnary(g_start_region,0,0), 0);
                closelist.push_back(b_s_pre);
                b_s->m_prestep = b_s_pre;
            }
        }
    }

    else if(prgn->m_type == Line){//if the first taxiway zone is a lane (this case should only appear for arrival aircrafts)
        qDebug() << QString("the first taxiway shouldn't be a lane");
    }


    /*do iterative search until: 1.the openlist is empty  2.the target bs is chosen for expansion*/
    uint curregion;//current region (for expansion)
    uint curnode;//current node (for expansion)
    int status = 1;
    base_step* pb_s;
    while(!openlistexp.empty()){        
        g_iterationcount++;//迭代次数增加1

        uint n = openlistexp.size();


        GetMinHeuristicCostOfOpenlist(pb_s);//get least cost bs from open
        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
        if(g_veh_id == 13 && curnode == 128 && curregion == 154 ){
            int xx = 0;
        }
        //qDebug() << g_iterationcount << ": n=" << n << "  node:" << curnode << "  zone:" << curregion;

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found
            Path path;
            path.push_back(pb_s);
            do{
                pb_s = pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                out_path.push_back(path.at(n - 1 - i));//调正顺序
            }
            status =  0;//denote path found

            g_expandtime_vector.push_back(exp_time);//将扩展过程所用的时间保存
            break;//退出while
        }
        else{
            t1 = GetHighPrecisionCurrentTime();
            Expand(pb_s);//preregion is used to prevent doubling back
            t2 = GetHighPrecisionCurrentTime();
            exp_time += t2 - t1;
        }
    }
#if defined(__DEBUG_SUCCESSIONCHECK__)
    CheckSuccessionRelation();
#endif
    return status;
}

void UpdateTimeWindowVector(Path& in_path, CRegionCollect &rct)
{
    twindow tw;
    //Path::iterator itr;
    //Path::iterator itrnext;
    //Path::iterator itrend = in_path.end();

    uint rngid;
    //uint awid;
    uint ndid;
    CRegion* prgn;
    char dir;
    assert(in_path.size() > 1);
    base_step* pb_s1;
    base_step* pb_s2;
    //double delta = 0.2;//用于防止同时资源交换现象的出现

    uint n = in_path.size();//for debug
    for(uint i = 0; i<n-1; i++){
        pb_s1 = in_path.at(i);
        pb_s2 = in_path.at(i+1);
        tw.tstart = pb_s1->m_entrytime;
        tw.tend = pb_s2->m_entrytime;
        rngid = GetRegionIndex(pb_s1->m_bs);
        ndid = GetNodeIndex(pb_s1->m_bs);
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
        //stand和buffer暂不处理
    }

}

void ResetGlobalVariables()
{
//    for(uint i=0; i<closelist.size();i++){
//        delete closelist.at(i);
//    }
    closelist.clear();
//    for(uint i=0; i<openlistexp.size(); i++){
//        delete openlistexp.at(i);
//    }
    openlistexp.clear();
//    for(uint i=0; i<g_tmp_basestepvector.size(); i++){
//        delete g_tmp_basestepvector.at(i);
//    }
    g_tmp_basestepvector.clear();

    g_iterationcount = 0;
    g_accesscount = 0;
}

bool EnterTime_Line(TWVector &in_twv, twindow* in_ew, uchar in_cap,double &out_tenter, uint& out_awid)
{
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    double tenter;
    uint index;
    OccVariable* poccv;
    OccVariable* poccpre;
    int status = 0;//denote special status of the in-out vector: when there is an in and an out appear at same time
    for(uint i=0; i<twnum; i++){
        ptwtmp = in_twv.at(i);
        //分析可知，区域控制规则决定了唯一可能的情形如下
        if((in_ew->tstart > ptwtmp->tstart) && (in_ew->tstart < ptwtmp->tend)){
            out_awid = i;
            tenter = in_ew->tstart;
            //确定当前航空器在ORS中的位置
            for(index = 0; index < ptwtmp->GetOccVector().size();index++){
                poccv = ptwtmp->GetOccVector().at(index);
                if(poccv->first > tenter ){
                    if(index < (ptwtmp->GetOccVector().size()-1)){
                        if(fabs(poccv->first - ptwtmp->GetOccVector().at(index+1)->first) < 1e-3){//如果下一个in-out动作的时刻与前一个动作的相同
                            index += 1;
                            status = 1;
                        }
                    }
                    break;
                }
            }
            //在ORS的开头或末尾
            if((index == ptwtmp->GetOccVector().size()) || (index == 0)){
                out_tenter = tenter;
                return true;
            }
            //一般情况
            else{
                if(status == 0){
                    poccpre = ptwtmp->GetOccVector().at(index-1);
                }
                else{
                    assert(index > 1);//这种情况下index至少=2
                    poccpre = ptwtmp->GetOccVector().at(index-2);
                }


                if(poccpre->second < in_cap){
                    out_tenter = tenter;
                    return true;
                }
                else{
                    /*if(g_priority == 88){
                        qDebug("...");
                    }*/

                    assert(poccpre->second == in_cap);
                    for(; index<ptwtmp->GetOccVector().size();index++){
                        poccv = ptwtmp->GetOccVector().at(index);
                        if(poccv->second < in_cap){
                            tenter = poccv->first;
                            if(tenter < in_ew->tend){
                                out_tenter = tenter;
                                return true;
                            }
                        }
                    }
                }
            }
            break;
        }

    }
    return false;

}

quint64 TernaryToUnary(uint n1, uint n2, uint n3)
{
    quint64 tmp = n1;
    tmp = (tmp << 22);
    tmp += n2;
    tmp = (tmp << 20);
    tmp += n3;
    return tmp;
}

bool Overlapped(twindow* in_ptw,TWVector& in_twv, uint& out_count, WindowTimeTable& out_wtt)
{
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    out_count = 0;
    double tmin = 0;
    if(twnum == 0){
        return false;
    }
    else{
        for(uint i=0; i<twnum; i++){
            ptwtmp = in_twv[i];
            if(EnterTime_Intersection(in_ptw, ptwtmp, tmin)){
                out_count++;
                out_wtt.insert(WindowTimeTable::value_type(i, tmin));
            }
        }
        return (out_count > 0);
    }
}

bool EnterTime_Intersection(twindow* in_ptw1, twindow* in_ptw2, double& out_tmin)
{
    if((in_ptw1->tstart >= in_ptw2->tstart) && (in_ptw1->tstart <= in_ptw2->tend)){
        out_tmin = in_ptw1->tstart;
        return true;
    }
    else if((in_ptw2->tstart > in_ptw1->tstart)&&(in_ptw2->tstart < in_ptw1->tend)){
        out_tmin = in_ptw2->tstart;
        return true;
    }
    else return false;

}

void GetMinHeuristicCostOfOpenlist(base_step*& out_bs)
{
/*    BSSuccession::iterator itr = bssuccession.begin();
    for(uint i=0; i< bssuccession.size(); i++){
        uint t1 = step_entertimelist[itr->second];
        uint t2 = step_entertimelist[itr->first];
        if( t1 > t2){
            int x= 0;
        }
        itr++;
    }
 */
    out_bs = openlistexp.at(0);
    double cost = out_bs->m_entrytime+g_array_h[GetNodeIndex(out_bs->m_bs)]/g_speed;
    double tmptime;
    double tmpcost;
    uint index = 0;
    base_step* bs;
    uint ndid;
    for(uint i=0; i < openlistexp.size();i++){
        bs = openlistexp.at(i);
        ndid = GetNodeIndex(bs->m_bs);
        tmptime = bs->m_entrytime;
        tmpcost = tmptime + g_array_h[ndid]/g_speed;
        if(cost > tmpcost){
            out_bs = openlistexp.at(i);
            cost = out_bs->m_entrytime + g_array_h[GetNodeIndex(out_bs->m_bs)]/g_speed;
            index = i;
        }
    }
    openlistexp.erase(openlistexp.begin()+index);//将它从openlist中取出,并加入closelist
    closelist.push_back(out_bs);
/*
    itr = bssuccession.begin();
    for(uint i=0; i< bssuccession.size(); i++){
        uint t1 = step_entertimelist[itr->second];
        uint t2 = step_entertimelist[itr->first];
        if( t1 > t2){
            int x= 0;
        }
        itr++;
    }
    */
}

uint GetRegionIndex(Basicstep in_bs)
{
    uint rindex = (uint)(in_bs >> 42);
    return rindex;
}

uint GetNodeIndex(Basicstep in_bs)
{
    uint nindex = (uint)(in_bs >> 20);
    nindex = nindex & uint(0x3fffff);
    return nindex;

}

uint GetAWindowIndex(Basicstep in_bs)
{
    uint windex = (uint)in_bs;
    windex = windex & uint(0xfffff);
    return windex;
}

void Expand(base_step* in_bs)
{
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
    uint curawid = GetAWindowIndex(in_bs->m_bs);//current free time window id
    //if(currid == 48 && fabs(in_bs->m_entrytime-366126.91)<0.01){
    //    int x = 1;
    //}
    //如果expand遇到buffer或stand，表明某条路已经到了尽头，但并非计划文件的目标位置，因此直接返回
    CRegion* prgn = g_model.m_rct.GetRegion(currid);//pointer to current region
    assert(prgn->m_type != Line);//按照动态路由算法的原理，这里不应当出现Line类型的扩展基准
    if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
        return;
    }//（因此，expand事实上只对Inter和Runway类型的扩展基准进行扩展）

    assert((prgn->m_type == Inter) || (prgn->m_type == Runway));
    //if(!((prgn->m_type == Inter) || (prgn->m_type == Runway))){
    //    int x = 0;
    //}
    double avg_speed;//TODO:这里是否可以考虑增加转弯和直行时平均速度存在的差异？
    if(prgn->m_type == Inter){
        avg_speed = g_speed;
    }
    else{
        //avg_speed = g_speed * 5;//假定跑道上滑行的速度是普通滑行速度的5倍
        avg_speed = g_speed;
    }
    twindow* paw;
    paw = prgn->GetTWVector().at(curawid);
    CRegion* pnb;// pointer to neighbor region
    double nnlength;//length to the neighbor node
    twindow* pew = new twindow;
    uint ndcount = prgn->m_count_node;//number of nodes contained in current region
    uint nd_id;//node id
    uint neighbor_zone_id;
    char dir_nb;
    twindow* pnbaw = 0;
    Basicstep curbs;
    double tenter;
    cnode* pnode ;
    g_accesscount += ndcount-1; //分枝个数等于进行可达性判定的次数，完成对所有分枝的可达性判定等于完成一次扩展
    //对当前区域包含的每个节点ndid（除当前步骤对应的节点外），判定由当前区域和ndid确定的相邻区域的可达性
    for(uint i=0; i<ndcount; i++){
        nd_id = prgn->m_vector_node.at(i);
        pnode = g_model.m_ndct.m_array_node[nd_id];
        if(nd_id == curnodeid){
            continue;
        }
        else{
            curbs = in_bs->m_bs;//initialize current bs as the input bs for expansion
            neighbor_zone_id = pnode->GetAnotherZoneId(currid);
            if(neighbor_zone_id == 1000){
                continue;
            }
            pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//pointer to the neighbor region
            //assert(g_model.matrix_nn_conn[nd_id][curnodeid] == 1);
            if(g_model.matrix_nn_conn[curnodeid][nd_id] == 0 ){//若从curnodeid到nd_id不可行（无滑行路线或该方向的滑行被禁止），则退出本次循环
                continue;
            }
            nnlength = g_model.matrix_nn_dis[curnodeid][nd_id];//length between current node and the neighbor node
            uint paircount = 0;
            WindowTimeTable wtt;
            //这里的扩展基准只可能是Inter或Runway型，所以:
            if(ExitWindow_Intersection(in_bs->m_entrytime, nnlength, paw, pew, avg_speed)){

                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){

                    uint awid_line;
                    dir_nb = pnb->GetDirection(nd_id);
                    if(!EnterTime_Line(pnb->GetTWVector(dir_nb), pew, pnb->GetCapacity(), tenter, awid_line)){
                        continue;//if the neighbor lane is not accessible, continue
                    }
                    else{//若该路段可以进入，需要进一步确定是否可脱离
                        //if(pnb->m_id == 48 && fabs(tenter-366126.91)<0.01){
                        //    int x = 1;
                        //}
                        pnbaw = pnb->GetTWVector(dir_nb).at(awid_line);
                        assert(pnb->m_count_node == 2);
                        uint next_node_id;
                        if(pnb->m_vector_node.at(0) == nd_id){
                            next_node_id = pnb->m_vector_node.at(1);
                        }
                        else{
                            next_node_id = pnb->m_vector_node.at(0);
                        }
                        nnlength = g_model.matrix_nn_dis[nd_id][next_node_id];//length to the other node of the lane
                        if(!ExitWindow_Line(tenter, pnbaw, pnb->GetCapacity(), nnlength, pew, g_speed)){
                            continue;//if the lane is not exit-able, continue
                        }
                        else{
                            uint nextinterid = g_model.m_ndct.m_array_node[next_node_id]->GetAnotherZoneId(neighbor_zone_id);//another neighbor intersection's id of the lane( assumes that a lane connects with an intersection at either end)
                            if((nextinterid == 1000) || (g_model.matrix_nn_conn[nd_id][next_node_id] == 0)){//若nd_id的另一个相邻区域不存在，或当前路段不可用
                                continue;
                            }
                            else{//至此，表明在当前路段区域可脱离。与该路段相邻的下一区域为Inter或Runway:
                                TernaryToUnary(neighbor_zone_id, nd_id, awid_line, curbs);//update the curbs
                                CRegion* pnb_next = g_model.m_rct.GetRegion(nextinterid);//指向下一顶点
                                if(!Overlapped(pew, pnb_next->GetTWVector(), paircount, wtt)){
                                    continue;
                                }
                                else{
                                    base_step* b_s_tmp;
                                    base_step* b_s_lane;
                                    Basicstep bs;
                                    uint awid;
                                    volatile double entertime;
                                    WindowTimeTable::iterator itr;
                                    for(itr=wtt.begin();itr!=wtt.end();itr++){
                                        awid = itr->first;
                                        entertime = itr->second;
                                        TernaryToUnary(nextinterid, next_node_id, awid, bs);
                                        if(Closed(bs)){
                                            continue;
                                        }
                                        else if(Opened(bs, b_s_tmp)){//判断是否open，并确定相应的base_step指针
                                            if(b_s_tmp->m_entrytime > entertime){
                                                //for the lane
                                                b_s_lane = new base_step(curbs, tenter);
                                                g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存
                                                b_s_lane->m_prestep = in_bs;
                                                //for the intersection
                                                b_s_tmp->m_entrytime = entertime;
                                                b_s_tmp->m_prestep = b_s_lane;

                                            }

                                        }
                                        else{
                                            b_s_tmp = new base_step(bs, entertime);
                                            openlistexp.push_back(b_s_tmp);//add bs to open
                                            b_s_lane = new base_step(curbs, tenter);
                                            g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存

                                            b_s_lane->m_prestep = in_bs;
                                            b_s_tmp->m_prestep = b_s_lane;

                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                /*if current neighbor is an inter or a runway*/
                else if((pnb->m_type == Inter) || (pnb->m_type == Runway)){
                    if(!Overlapped(pew, pnb->GetTWVector(), paircount, wtt)){
                        continue;
                    }
                    else{
                        base_step* b_s_tmp;
                        Basicstep bs;
                        uint awid;
                        double entertime;
                        WindowTimeTable::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            awid = itr->first;
                            entertime = itr->second;
                            TernaryToUnary(neighbor_zone_id, nd_id, awid, bs);
                            if(Closed(bs)){
                                continue;
                            }
                            else if(Opened(bs, b_s_tmp)){

                                if(b_s_tmp->m_entrytime > entertime){
                                    b_s_tmp->m_entrytime = entertime;
                                    b_s_tmp->m_prestep = in_bs;
                                }

                            }
                            else{
                                b_s_tmp = new base_step(bs, entertime);
                                b_s_tmp->m_prestep = in_bs;
                                openlistexp.push_back(b_s_tmp);
                            }
                        }
                    }
                }
                /*else if current neighbor is a stand or a buffer*/
                else{//stand和air-buffer内的冲突情况暂不考虑，所以：
                    base_step* b_s_tmp;
                    Basicstep bs;
                    double entertime = pew->tstart;
                    TernaryToUnary(neighbor_zone_id, nd_id, 0, bs);
                    if(Closed(bs)){
                        continue;
                    }
                    else if(Opened(bs, b_s_tmp)){
                        if(b_s_tmp->m_entrytime > entertime){
                            b_s_tmp->m_entrytime = entertime;
                            b_s_tmp->m_prestep = in_bs;
                        }

                    }
                    else{
                        b_s_tmp = new base_step(bs, entertime);
                        b_s_tmp->m_prestep = in_bs;
                        openlistexp.push_back(b_s_tmp);
                    }
                }
            }
        }
    }
    delete pew;
}

bool ExitWindow_Intersection(double in_time, double in_nnlength, twindow* in_paw,
        twindow* out_pew, double in_speed)
{
    double tend = in_time + in_nnlength/in_speed;
    if(tend > in_paw->tend){
        return false;
    }
    else{
        if(tend < in_paw->tstart){//TODO:这里应当比较in_time和in_paw->tstart吧？
            qDebug("error, the enter time is not among the aw");
            return false;
        }
        out_pew->tstart = tend;
        out_pew->tend = in_paw->tend;
        return true;
    }
}

bool ExitWindow_Line(const double in_tenter, twindow* in_aw, uchar in_cap, double in_nnlength,
                     twindow* out_pew, double in_speed)
{
    /*if the occvector is empty, it's straightforward*/
    if(in_aw->GetOccVector().size() == 0){
        out_pew->tstart = in_nnlength/in_speed + in_tenter;
        out_pew->tend = in_aw->tend;
        if(out_pew->tend > out_pew->tstart){
            return true;
        }
        else{
            return false;
        }
    }
    /*else, */
    double tl;
    double tr;
    OccVariable* pocc;
    uchar precount = 0;
    uint out = 0;
    uint i;
    char status = 1;
    for(i=0;i<in_aw->GetOccVector().size();i++){
        pocc = in_aw->GetOccVector().at(i);
        if(pocc->first > in_tenter){
            if(i == 0){
                precount = 0;
            }
            else{
                pocc = in_aw->GetOccVector().at(i-1);
                precount = pocc->second;//当前区域中的航空器个数
            }
            break;
        }
    }
    if(i == 0){//若当前规划航空器先于之前进入该路段的所有航空器进入该路段
        tl = in_nnlength/in_speed + in_tenter;
        for(;i<in_aw->GetOccVector().size();i++){
            if(in_aw->GetInOutVector().at(i) != 1){
                tr = in_aw->GetOccVector().at(i)->first;//此时，当前航空器要先于其他航空器脱离该路段
                break;
            }
            if(in_aw->GetOccVector().at(i)->second == in_cap){//若在第一个out动作之前遇到满容量时间窗
                tr = in_aw->GetOccVector().at(i)->first;
                break;
            }
        }
        if(tl < tr){
            out_pew->tstart = tl;
            out_pew->tend = tr;
            return true;
        }
        else{
            return false;
        }
    }
    else if(i == in_aw->GetOccVector().size()){//if the entry time lie out of all the occpancy records, it's also straightforward
        out_pew->tstart = in_nnlength/in_speed + in_tenter;
        out_pew->tend = in_aw->tend;
        if(out_pew->tend > out_pew->tstart){
            return true;
        }
        else{
            return false;
        }
    }
    else{//一般的情况
        tl = in_nnlength/in_speed + in_tenter;
        tr = in_aw->tend;

        for(;i<in_aw->GetOccVector().size();i++){
            if(in_aw->GetInOutVector().at(i) != 1){
                out++;
            }
            switch(status){
            case 1://还有先前进入该路段的航空器未离开该路段
                if(in_aw->GetOccVector().at(i)->second == in_cap){
                    return false;
                }
                if(out == precount){
                    //----------2016-1-2 revised (because a bug is found here)
                    if(precount > 0){
                        if(tl < in_aw->GetOccVector().at(i)->first){
                            tl = in_aw->GetOccVector().at(i)->first;
                        }
                    }
                    status = 2;
                    //------------------------------------------------------
                }
                break;
            case 2://确定了先前进入该路段的航空器中最晚离开该路段的脱离时刻
                if(out == (precount+1)){
                    status = 3;
                    tr = in_aw->GetOccVector().at(i)->first;
                    if(tl < tr){
                        out_pew->tstart = tl;
                        out_pew->tend = tr;
                        return true;
                    }
                    else{
                        return false;
                    }
                }
                else if(in_aw->GetOccVector().at(i)->second == in_cap){
                    return false;
                }
                break;

            }

        }

    }
    assert(status != 1);
    //if(status == 1){
    //    int x = 0;
    //}
    if(status == 2){
        if(tl < tr){
            out_pew->tstart = tl;
            out_pew->tend = tr;
            return true;
        }
        else{
            return false;
        }
    }
}

void TernaryToUnary(uint n1, uint n2, uint n3, quint64& out_bs)
{
    out_bs = n1;
    out_bs = (out_bs << 22);
    out_bs += n2;
    out_bs = (out_bs << 20);
    out_bs += n3;
}

bool Closed(Basicstep in_bs)
{
    base_step* pb_s;
    for(uint i=0; i<closelist.size();i++){
        pb_s = closelist.at(i);
        if(pb_s->m_bs == in_bs){
            return true;
        }
    }
    return false;
}

bool Opened(Basicstep in_bs, base_step*& out_b_s)
{
    base_step* pb_s;
    for(uint i=0; i<openlistexp.size();i++){
        pb_s = openlistexp.at(i);
        if(pb_s->m_bs == in_bs){
            out_b_s = pb_s;
            return true;
        }
    }
    return false;
}

void RemoveTimewindow_Line(TWVector& twv, twindow* in_ptw)
{
    twindow* ptw;
    twindow* pnewtw;
    double tstart = in_ptw->tstart;
    double tend = in_ptw->tend;
    double tmp;
    uint index_aw;
    uint i;
    //if(fabs(in_ptw->tend-368143.26)<0.01){
    //    int stop=1;
    //}
    //if((fabs(in_ptw->tstart-366150.37)<0.01) && (fabs(in_ptw->tend-366216.7)<0.1)){
    //    int stop = 0;
    //}
    //确定in_ptw所在时间窗
    for(i=0; i<twv.size(); i++){
        ptw = twv.at(i);
        if(tstart < ptw->tend){
            index_aw = i;
            break;
        }
    }
    assert(i != twv.size());
    if(i < twv.size()-1){
        ptw = twv.at(i+1);
        assert(tend < ptw->tstart);
        //if(tend > ptw->tstart){
        //    int x = 0;
        //}
    }
    ptw = twv.at(index_aw);
    //从该时间窗中去除in_ptw
    if(tend <= ptw->tstart){//若in_ptw恰好落在时间窗之间的空白处，直接返回
        return;
    }
    else if(tend >= ptw->tend){//若in_ptw长度超过了当前time window
        tmp = tstart;
        tstart = ptw->tend;
        ptw->tend = tmp;
        if((ptw->tend - ptw->tstart) < c_twlengthmin){
            twv.erase(twv.begin()+i);
            delete ptw;
        }
    }
    else if(tstart <= ptw->tstart){
        ptw->tstart = tend;
        if((ptw->tend - ptw->tstart) < c_twlengthmin){
            twv.erase(twv.begin()+i);
            delete ptw;
        }
    }
    else{
        double gaps = (tstart-ptw->tstart);
        double gape = (ptw->tend -  tend);
        if(gaps < c_twlengthmin){
            if(gape >= c_twlengthmin){
                ptw->tstart = tend;
            }
            else{//gape < c_twlengthmin
                twv.erase(twv.begin()+i);
                delete ptw;
            }
        }
        else{
            if(gape >= c_twlengthmin){

                pnewtw = new twindow;
                pnewtw->tstart = tend;
                pnewtw->tend = ptw->tend;
                twv.insert(twv.begin()+i+1, pnewtw);

                ptw->tend = tstart;
                //更新两个新time window的ORS
                //判断in-out向量信息是否正确
                //int y;
                //if(ptw->GetOccVector().size()%2 != 0){
                //    y = ptw->GetOccVector().size();
                //}
                if(ptw->GetOccVector().size() > 0){
                    OccVariable* pocc;
                    char io_tmp;
                    uint index;
                    uint loop;
                    for(loop=0; loop<ptw->GetOccVector().size();loop++){
                        pocc = ptw->GetOccVector().at(loop);
                        if(pocc->first > ptw->tend){
                            index = loop;
                            break;
                        }
                    }

                    if(loop!=ptw->GetOccVector().size()){
                        //if(loop%2 != 0){
                        //    int error = 1;
                        //}
                        for(uint i = index ;i<ptw->GetOccVector().size();i++){
                            pocc = ptw->GetOccVector().at(i);
                            io_tmp = ptw->GetInOutVector().at(i);
                            pnewtw->GetOccVector().push_back(new OccVariable(pocc->first,pocc->second));
                            pnewtw->GetInOutVector().push_back(io_tmp);

                        }
                        for(uint i=index; i<ptw->GetOccVector().size();i++){
                            delete ptw->GetOccVector().at(i);
                        }
                        ptw->GetOccVector().erase(ptw->GetOccVector().begin()+index, ptw->GetOccVector().end());
                        ptw->GetInOutVector().erase(ptw->GetInOutVector().begin()+index, ptw->GetInOutVector().end());
                    }
                    //判断in-out向量信息是否正确
                    //if(ptw->GetOccVector().size()%2 != 0){
                    //    int z = 0;
                    //}
                    //if(pnewtw->GetOccVector().size()%2 !=0){
                    //    int x = 0;
                    //}

                }

            }
            else{
                ptw->tend = tstart;
            }
        }
    }
}

void UpdateOccupation(TWVector& twv, twindow* in_ptw, uint in_cap)
{
    twindow* ptw;
    uint i;
    for(i=0;i<twv.size();i++){
        ptw = twv.at(i);
        if(in_ptw->tstart < ptw->tend){//确定了占用发生时刻所在的时间窗
            break;
        }
    }
    assert(i!= twv.size());
    OccVector& occv = ptw->GetOccVector();
    InOutVector& inoutv = ptw->GetInOutVector();

    //if the occvector is empty, it's straightforward
    if((occv.size() == 0)){
        OccVariable* newocc1 = new OccVariable(in_ptw->tstart, 1);
        occv.push_back(newocc1);
        OccVariable* newocc2 = new OccVariable(in_ptw->tend, 0);
        occv.push_back(newocc2);
        inoutv.push_back(1);
        inoutv.push_back(-1);
        return;
    }
    //否则，occv.size()应该为偶数
    assert( occv.size()%2 == 0);//otherwise, it's a little complicated

    uint indexstart=0;
    uint indexend=0;
    uint flag_start = 0;
    uint flag_end = 0;
    uint curcap;
    OccVariable* poc;
    if(occv.at(0)->first >= in_ptw->tend){//若当前航空器先于其他航空器进入该路段
        OccVariable* newocc1 = new OccVariable(in_ptw->tstart, 1);
        OccVariable* newocc2 = new OccVariable(in_ptw->tend, 0);
        occv.insert(occv.begin(),newocc1);
        occv.insert(occv.begin()+1,newocc2);
        inoutv.insert(inoutv.begin(),1);
        inoutv.insert(inoutv.begin()+1,-1);
        return;
    }
    else if(occv.at(occv.size()-1)->first <= in_ptw->tstart){//若当前航空器在先前所有航空器离开该路段后进入路段
        OccVariable* newocc1 = new OccVariable(in_ptw->tstart, 1);
        OccVariable* newocc2 = new OccVariable(in_ptw->tend, 0);
        occv.push_back(newocc1);
        occv.push_back(newocc2);
        inoutv.push_back(1);
        inoutv.push_back(-1);
        return;
    }
    else{//一般情况
        for(i=0;i<occv.size();i++){
            poc = occv.at(i);
            if(flag_start == 0){
                if(poc->first > in_ptw->tstart){
                    indexstart = i;
                    flag_start = 1;
                }
            }
            if(flag_end == 0){
                if(poc->first > in_ptw->tend){
                    indexend = i;
                    flag_end = 1;
                    break;//flag_end=1表明已经确定了indexstart和indexend，可以中断for循环
                }
            }
        }
        assert(flag_start == 1);
        if(flag_end == 0){//若当前航空器是最后一个脱离该路段的，将indexend指向向量末尾
            indexend = occv.size();

            for(uint j = indexstart; j<=indexend-1; j++){
                poc = occv.at(j);
                poc->second = poc->second+1;
                //if(poc->second > in_cap){
                //    int x = 0;
                //}
            }
            if(indexstart == 0){
                curcap = 0;
            }
            else{
                poc = occv.at(indexstart-1);
                curcap = poc->second;
            }

            OccVariable* newocc1 = new OccVariable(in_ptw->tstart, curcap+1);
            occv.insert(occv.begin()+indexstart, newocc1);
            inoutv.insert(inoutv.begin()+indexstart, 1);
            OccVariable* newocc2 = new OccVariable(in_ptw->tend, 0);
            occv.push_back(newocc2);
            inoutv.push_back(-1);
        }
        else{
            if(indexstart == indexend){
                assert(indexstart>0);
                poc = occv.at(indexstart-1);
                curcap = poc->second;
                OccVariable* newocc1 = new OccVariable(in_ptw->tstart, curcap+1);
                OccVariable* newocc2 = new OccVariable(in_ptw->tend, curcap);
                occv.insert(occv.begin()+indexstart,newocc1);
                occv.insert(occv.begin()+indexstart+1,newocc2);
                inoutv.insert(inoutv.begin()+indexstart,1);
                inoutv.insert(inoutv.begin()+indexstart+1,-1);
            }
            else{
                assert(indexend > indexstart);
                for(uint j = indexstart; j<=indexend-1; j++){
                    poc = occv.at(j);
                    poc->second = poc->second+1;
                    //if(poc->second > in_cap){
                    //    int x = 0;
                    //}
                }
                if(indexstart == 0){
                    curcap = 0;
                }
                else{
                    poc = occv.at(indexstart-1);
                    curcap = poc->second;
                }

                OccVariable* newocc1 = new OccVariable(in_ptw->tstart, curcap+1);
                poc = occv.at(indexend-1);
                curcap = poc->second;
                OccVariable* newocc2 = new OccVariable(in_ptw->tend, curcap-1);
                occv.insert(occv.begin()+indexstart, newocc1);
                inoutv.insert(inoutv.begin()+indexstart, 1);
                occv.insert(occv.begin()+indexend+1, newocc2);//因为插入了一个新元素，indexend指向的位置被后移了，需要加1.
                inoutv.insert(inoutv.begin()+indexend+1, -1);

            }
        }
    }
    assert(occv.size()%2 == 0);
}

void RemoveTimewindow_Inter(TWVector& twv,  twindow* in_ptw)
{
    twindow* ptw;
    double gapstart;
    double gapend;
    uint i;//used to denote the index of free time window in which current agent occupies the intersection
   // ptw = twv.at(in_awid);//摒弃不用了，因为可用时间窗序号可能会受前面更新过程的影响而变化
    for(i=0;i<twv.size();i++){
        ptw = twv.at(i);
        if(in_ptw->tstart < ptw->tend){
            break;
        }
    }
    assert(i!= twv.size());
    gapstart = in_ptw->tstart - ptw->tstart;
    gapend = ptw->tend - in_ptw->tend;
    if((gapstart < 0)||(gapend < 0)){
        qDebug("error, the aw and ow don't match");
        qDebug() << QString("gapstart=") <<gapstart;
        qDebug() << QString("gapend=") <<gapend;

    }
    else{
        if(gapstart > c_twlengthmin){

            if(gapend > c_twlengthmin){//remove the occupancy window in the middle
                twindow* twnew = new twindow(in_ptw->tend, ptw->tend);
                twv.insert(twv.begin()+i+1, twnew);
                ptw->tend = in_ptw->tstart;
            }
            else{
                ptw->tend = in_ptw->tstart;//remove the occupancy window at the end
            }
        }
        else if(gapend>c_twlengthmin){
            ptw->tstart = in_ptw->tend;//remove the occupancy window at the beginning
        }
        else{
            twv.erase(twv.begin()+i);//remove the original free time window completely
        }
    }

}

void TimeWindowSlimming(double in_cur_time)
{
    CRegion* prgn;
    twindow* ptw;
    uint index = 0;
    for(uint i=0; i<g_model.m_rct.m_count_region; i++){
        prgn = g_model.m_rct.GetRegion(i);
        if(prgn->m_type == Line){//路段
            TWVector& twv = prgn->GetTWVector(0);//正向
            //确定需要删除的元素所在范围，注意时间窗向量中的时间窗是按照时间先后存储的
            for(index=0; index<twv.size(); index++){
                ptw = twv.at(index);
                if(ptw->tend > in_cur_time){
                    if(index > 0){
                        twv.erase(twv.begin(),twv.begin()+index-1);
                    }
                    break;
                }
            }
            TWVector& twv_r = prgn->GetTWVector(1);//反向
            for(index=0; index<twv_r.size(); index++){
                ptw = twv_r.at(index);
                if(ptw->tend > in_cur_time){
                    if(index > 0){
                        twv_r.erase(twv_r.begin(),twv_r.begin()+index-1);
                    }
                    break;
                }
            }

        }
        else{//其他类型的区域
            TWVector& twv = prgn->GetTWVector();
            for(index=0; index<twv.size(); index++){
                ptw = twv.at(index);
                if(ptw->tend > in_cur_time){
                    if(index > 0){
                        twv.erase(twv.begin(),twv.begin()+index-1);
                    }
                    break;
                }
            }
        }
    }
}


