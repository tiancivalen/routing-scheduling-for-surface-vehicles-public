#include "TP_nowaiting.h"
#include "TaxiPlanningAlgorithm.h"
#include "globalvariable.h"
#include "assert.h"
#include <QElapsedTimer>
#include "operation.h"
#include <math.h>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include "TP_robust_globalvariables.h"

extern double g_frequency;


void Experiment_nowaiting()
{
    //printParam(QString("..\\data_out\\参数设置.txt"));
    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件
    SequencePlan_nowaiting();
    QString str_freq = QString("_%1").arg(g_frequency);
    //PrintComputeTime("..\\data_out\\computetime_nowaiting"+str_freq+".txt", ctvector);
    //PrintIterationCount("..\\data_out\\iterationcount_nowaiting"+str_freq+".txt",itr_cnt_vector);
}

int SequencePlan_nowaiting()
{
    int returnvalue;
    QString fdir = QString("..\\data_out");
    QFile file(QString("%1\\ComputationTime.txt").arg(fdir));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug()<< "Failed to create the computational time recording file.";
        return -1;
    }
    QTextStream textstm(&file);

    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        if((i+1)%10==0){
            textstm << "\n";
        }

        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
//        if(g_veh_id == 6){
//            int x = 0;
//        }

        //----------每若干次循环清理一次时间窗-----------
        if(g_priority%50 == 0){
            TimeWindowSlimming(veh->m_start_time);
        }
        //------------------------------------------



        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_nowaiting(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        textstm << QString("%1 ").arg(t2-t1,0,'f',1);//单位：毫秒
        if(returnvalue == 0){

            genTBST_nowaiting(veh);
            ctvector.push_back(t2-t1);//calclation time recoding
            itr_cnt_vector.push_back(g_iterationcount);//记录迭代次数
            UpdateTimeWindowVector_nowaiting(veh->m_path,g_model.m_rct);
            printPathDetail_nowaiting(veh->m_path,fdir);
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);

            //if(g_veh_id == 54){
                //printPathDetail(veh->m_rPath,fdir);
                //SavePathToFile_robust(veh->m_id, veh->m_rPath,QString("..\\data_out\\RobustDRT_path_of_vehicle%1.txt").arg(veh->m_id));
            //}
//            if(g_veh_id == 9){
//                QString file = QString("..\\data_out\\Robust_twindowaftertarget_%1.txt").arg(veh->m_id);
//                PrintTimeWindows(file, g_model.m_rct);//for debug
//            }
        }
        else{
            ctvector.push_back(-1);
            itr_cnt_vector.push_back(-1);//记录迭代次数
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }

        //itr_cnt_vector.push_back(g_iterationcount);
        ResetGlobalVariables();//当前航空器规划完毕后复位必要的全局变量：
        g_priority++;//denote agent's planning priority

        // ------------ 线程休眠一段时间以降低CPU占用率 ---------
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){
            QCoreApplication::processEvents();
        }
        // -------------------------------------------------

    }
    //全部规划完毕后：
    PathFeasibilityCheck_nowaiting();//检测路径的物理可行性
    CheckOccupationVector_nowaiting(g_model.m_rct);//检查路段的占用向量中是否存在错误（路段容量、FIFO冲突）
    ConflictDetection_nowaiting();//检查是否存在（交叉口和路段对头）冲突
    HoldDetection_nowaiting();
    ResetTimeWindows(g_model.m_rct);//复位时间窗约束
    delete[] g_array_h;//释放启发值数组
    qDebug() << QString("------------Planning completed-----------");

    file.close();

    return 0;
}

int Plan_nowaiting(Vehicle *veh)
{
    double exp_time = 0;
    double t1, t2;
//    if(veh->m_id == 12){
//        int xx = 0;
//    }
    g_start_region = veh->m_start_region;
    g_start_node = veh->m_start_node;
    g_start_time = veh->m_start_time;
    g_end_region = veh->m_end_region;
    g_end_node = veh->m_end_node;

    // ---------- 启发值更新 ----------
    Heuristics(g_end_node);
    // ------------------------------


    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
            ->GetAnotherZoneId(g_start_region);//第一个滑行道区域编号

    Basicstep bs;
    twindow entw;//entry time window
    //double ioa = g_ioa;//length of the initial IOA
    //double k = 0.1;//按ioa/travel time=0.1
    uint rgnid;
    uint ndid;
    uint awid;
    ndid = g_start_node;
    rgnid = first_taxiway_region;
    //twindow ew(ioaStart(ioa,g_start_time), c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
    double delta = 5;//机身脱离当前区域需要的时间
    twindow ew(g_start_time, c_timemax-delta);//initialize the exit window from the start zone
    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the next zone to be visited, i.e., the first taxiway zone
    /*tackle the first taxiway zone*/
    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
        AW_ENTW_table_nowaiting wtt;
        uint num;
        if(!Overlapped_nowaiting(&ew,prgn->GetTWVector(),num,wtt)){//
            QString info = QString("error, the entry time window of target %1 is not properly set").arg(veh->m_id);
            qDebug(info.toAscii());
            return 1;
        }
        else{
            AW_ENTW_table_nowaiting::iterator itr;
            for(itr=wtt.begin();itr!=wtt.end();itr++){
                awid = itr->first;
                entw = itr->second;//
                entw.tend = fmin(entw.tstart+c_waitmax,entw.tend);//注意这里的处理
                bs = TernaryToUnary(rgnid, ndid, awid);
                nowaiting_step* b_s = new nowaiting_step(bs, entw);//
                openlistexp.push_back(b_s);//这里不考虑dominate的问题，直接加入openlist
                nowaiting_step* b_s_pre = new nowaiting_step(TernaryToUnary(g_start_region,0,0),twindow(0,0));
                b_s->m_prestep = b_s_pre;
                closelist.push_back(b_s_pre);//
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
    nowaiting_step* pb_s;

    g_iterationcount = 0;//迭代次数归零
    while(!openlistexp.empty()){

        g_iterationcount++;//迭代次数增加1

        uint n = openlistexp.size();


        GetMinHeuristicCostOfOpenlist_nowaiting(pb_s);//get least cost bs from open

        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
        if(g_veh_id == 171){
            int xx = 0;
        }

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found

            Path path;
            path.push_back(pb_s);
            do{
                pb_s = (nowaiting_step*)pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                veh->m_path.push_back(path.at(n - 1 - i));//调正顺序
            }
            //veh->m_SinglePathLenVector.push_back(n);//记录step个数

            status =  0;//denote path found

            break;

        }
        else{
            //t1 = GetHighPrecisionCurrentTime();
            Expand_nowaiting(pb_s);//preregion is used to prevent doubling back
            //t2 = GetHighPrecisionCurrentTime();
            //exp_time += t2 - t1;
        }

    }

    return status;
}

int Plan_nowaiting_loopless(Vehicle *veh)
{
    double exp_time = 0;
    double t1, t2;
//    if(veh->m_id == 12){
//        int xx = 0;
//    }
    g_start_region = veh->m_start_region;
    g_start_node = veh->m_start_node;
    g_start_time = veh->m_start_time;
    g_end_region = veh->m_end_region;
    g_end_node = veh->m_end_node;

    // ---------- 启发值更新 ----------
    Heuristics(g_end_node);
    // ------------------------------


    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
            ->GetAnotherZoneId(g_start_region);//第一个滑行道区域编号

    Basicstep bs;
    twindow entw;//entry time window
    //double ioa = g_ioa;//length of the initial IOA
    //double k = 0.1;//按ioa/travel time=0.1
    uint rgnid;
    uint ndid;
    uint awid;
    ndid = g_start_node;
    rgnid = first_taxiway_region;
    //twindow ew(ioaStart(ioa,g_start_time), c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
    double delta = 5;//机身脱离当前区域需要的时间
    twindow ew(g_start_time, c_timemax-delta);//initialize the exit window from the start zone
    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the next zone to be visited, i.e., the first taxiway zone
    /*tackle the first taxiway zone*/
    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
        AW_ENTW_table_nowaiting wtt;
        uint num;
        if(!Overlapped_nowaiting(&ew,prgn->GetTWVector(),num,wtt)){//
            QString info = QString("error, the entry time window of target %1 is not properly set").arg(veh->m_id);
            qDebug(info.toAscii());
            return 1;
        }
        else{
            AW_ENTW_table_nowaiting::iterator itr;
            for(itr=wtt.begin();itr!=wtt.end();itr++){
                awid = itr->first;
                entw = itr->second;//
                entw.tend = fmin(entw.tstart+c_waitmax,entw.tend);//注意这里的处理
                bs = TernaryToUnary(rgnid, ndid, awid);
                nowaiting_step* b_s = new nowaiting_step(bs, entw);//
                openlistexp.push_back(b_s);//这里不考虑dominate的问题，直接加入openlist
                nowaiting_step* b_s_pre = new nowaiting_step(TernaryToUnary(g_start_region,0,0),twindow(0,0));
                b_s->m_prestep = b_s_pre;
                closelist.push_back(b_s_pre);//
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
    nowaiting_step* pb_s;

    t1 = GetHighPrecisionCurrentTime();//当前时间

    g_iterationcount = 0;//迭代次数归零
    while(!openlistexp.empty()){
// added 27-08-2017 by TZ
        t2 = GetHighPrecisionCurrentTime();
//        if((t2-t1)/1000>10){
//            status = 2;//2表示计算超时
//            return status;
//        }
// end added
        g_iterationcount++;//迭代次数增加1

        uint n = openlistexp.size();


        GetMinHeuristicCostOfOpenlist_nowaiting(pb_s);//get least cost bs from open
        //GetMinHeuristicCostOfOpenlist_nowaiting_deepfirst(pb_s);

        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
        if(g_veh_id == 171){
            int xx = 0;
        }

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found

            Path path;
            path.push_back(pb_s);
            do{
                pb_s = (nowaiting_step*)pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                veh->m_path.push_back(path.at(n - 1 - i));//调正顺序
            }
            //veh->m_SinglePathLenVector.push_back(n);//记录step个数

            status =  0;//denote path found

            break;

        }
        else{
            //t1 = GetHighPrecisionCurrentTime();
            Expand_nowaiting_loopless(pb_s);//preregion is used to prevent doubling back
            //t2 = GetHighPrecisionCurrentTime();
            //exp_time += t2 - t1;
        }

    }

    return status;
}

void Expand_nowaiting(nowaiting_step* in_bs)
{
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
//    if(curnodeid==161&& currid == 86&& g_veh_id == 54){
//        int x = 0;
//    }
    uint curawid = GetAWindowIndex(in_bs->m_bs);//current free time window id

//    if (g_veh_id == 1){
//        qDebug() << QString("region %1, node %2, entw [%3, %4], cost %5").arg(currid).arg(curnodeid).arg(in_bs->m_entw.tstart).arg(in_bs->m_entw.tend)
//                    .arg(in_bs->m_cost);

//    }


    CRegion* prgn = g_model.m_rct.GetRegion(currid);//pointer to current region
    assert(prgn->m_type != Line);//按照动态路由算法的原理，这里不应当出现Line类型的扩展基准
    if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
        return;
    }//（因此，expand事实上只对Inter和Runway类型的扩展基准进行扩展）
    assert((prgn->m_type == Inter) || (prgn->m_type == Runway));

    double avg_speed;//TODO:这里是否可以考虑增加转弯和直行时平均速度存在的差异？
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
    twindow entw;
    cnode* pnode ;
    g_accesscount += ndcount-1; //分枝个数等于进行可达性判定的次数，完成对所有分枝的可达性判定等于完成一次扩展
    //double ioa_first;
    //double ioa_second;
    uint act;//
    int isTurn;
    nowaiting_step* bs_pre;
    nowaiting_step* b_s_lane;
    bool flag_lane = false;
    bool flag_reachable = true;
    //对当前区域包含的每个节点ndid（除当前步骤对应的节点外），判定由当前区域和ndid确定的相邻区域的可达性
    for(uint i=0; i<ndcount; i++){
        nd_id = prgn->m_vector_node.at(i);
        pnode = g_model.m_ndct.m_array_node[nd_id];
        if(nd_id == curnodeid){
            continue;
        }
        else{
            bs_pre = in_bs;
            //curbs = in_bs->m_bs;//initialize current bs as the input bs for expansion
            neighbor_zone_id = pnode->GetAnotherZoneId(currid);
            if(neighbor_zone_id == 1000){//1000表示尚未包含在模型内的区域
                continue;
            }
            pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//pointer to the neighbor region
            if(g_model.matrix_nn_conn[curnodeid][nd_id] == 0 ){//若从curnodeid到nd_id不可行（无滑行路线或该方向的滑行被禁止），则退出本次循环
                continue;
            }
            nnlength = g_model.matrix_nn_dis[curnodeid][nd_id];//length between current node and the neighbor node
            uchar holdable = g_model.matrix_nn_hold[curnodeid][nd_id];
            act = getAction_nowaiting(currid,curnodeid,neighbor_zone_id);
            isTurn = g_model.matrix_nn_isTurn[curnodeid][nd_id];
            avg_speed = getSpeed_nowaiting(currid,act,isTurn);
            //ioa_first = getIOA(in_bs->m_IOA,nnlength,avg_speed,k);


            uint paircount = 0;
            AW_ENTW_table_nowaiting wtt;
            //这里的扩展基准只可能是Inter或Runway型，所以:
            if(ExitWindow_Intersection_nowaiting(in_bs->m_entw, nnlength, paw, pew, avg_speed,holdable)){
                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){
//                    if(g_veh_id == 223 && neighbor_zone_id == 166){
//                        int xx = 0;
//                    }
                    uint next_node_id;
                    uint next_rgn_id;
                    flag_lane = true;
                    while(flag_lane){
                        if(pnb->m_vector_node.at(0)==nd_id){
                            next_node_id = pnb->m_vector_node.at(1);
                        }
                        else{
                            next_node_id = pnb->m_vector_node.at(0);
                        }
                        next_rgn_id = g_model.m_ndct.m_array_node[next_node_id]->GetAnotherZoneId(neighbor_zone_id);
                        if((next_rgn_id == 1000) || (g_model.matrix_nn_conn[nd_id][next_node_id] == 0)){
                            flag_reachable = false;
                            break;
                        }
                        uint awid_line;
                        dir_nb = pnb->GetDirection(nd_id);
                        if(!EntryWindow_Line_nowaiting(pnb->GetTWVector(dir_nb), pew, pnb->GetCapacity(),avg_speed, entw, awid_line)){
                            flag_reachable = false;
                            break;//
                        }
                        else{//若该路段可以进入，需要进一步确定是否可脱离
                            pnbaw = pnb->GetTWVector(dir_nb).at(awid_line);
                            assert(pnb->m_count_node == 2);
                            //注意，下面nnlength和avg_speed被重新赋值，表示从路段入口到下一个交叉口入口的距离和速度，所以这里先备份
                            double nnlength_pre = nnlength;
                            double avg_speed_pre = avg_speed;
                            nnlength = g_model.matrix_nn_dis[nd_id][next_node_id];//length to the other node of the lane
                            holdable = g_model.matrix_nn_hold[nd_id][next_node_id];
                            act = getAction_nowaiting(neighbor_zone_id,nd_id,next_rgn_id);
                            isTurn = g_model.matrix_nn_isTurn[nd_id][next_node_id];
                            avg_speed = getSpeed_nowaiting(neighbor_zone_id,act,isTurn);
                            if(!ExitWindow_Line_nowaiting(entw, pnbaw, pnb->GetCapacity(), nnlength, pew, avg_speed, holdable)){
                                flag_reachable = false;
                                break;//if the lane is not exit-able, continue
                            }
                            else{
                                //至此，表明在当前路段区域可脱离。与该路段相邻的下一区域为Inter或Runway:
                                TernaryToUnary(neighbor_zone_id, nd_id, awid_line, curbs);//update the curbs
                                b_s_lane = new nowaiting_step(curbs, entw);
                                b_s_lane->m_length = nnlength_pre;
                                b_s_lane->m_speed = avg_speed_pre;
                                b_s_lane->m_prestep = bs_pre;
                                g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存

                                bs_pre = b_s_lane;//
                                nd_id = next_node_id;
                                neighbor_zone_id = next_rgn_id;

                                pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//指向下一顶点
                                if(pnb->m_type != Line){
                                    flag_lane = false;
                                }
                            }
                        }
                    }
                    if(flag_reachable){
                        if(!Overlapped_nowaiting(pew, pnb->GetTWVector(), paircount, wtt)){
                            continue;
                        }
                        else{
                            nowaiting_step* b_s_tmp;
                            Basicstep bs;
                            uint awid;
                            twindow entrytimewindow;
                            AW_ENTW_table_nowaiting::iterator itr;
                            for(itr=wtt.begin();itr!=wtt.end();itr++){
                                awid = itr->first;
                                entrytimewindow = itr->second;
                                //double cost2_time = nnlength/avg_speed;
                                //double cost2_delay = entrytimewindow.tstart - entw.tstart - cost2_time;
                                //double cost2 = calCost(nnlength,avg_speed,entertime-tenter);
                                TernaryToUnary(next_rgn_id, next_node_id, awid, bs);
                                NowaitingStepVector rsvec;
                                bool flag = false;
                                b_s_tmp = new nowaiting_step(bs,entrytimewindow);
                                b_s_tmp->m_length = nnlength;
                                b_s_tmp->m_speed = avg_speed;

                                if(Closed_nowaiting(bs,rsvec)){
                                    uint n = rsvec.size();
                                    for(uint i=0; i<n; i++){
                                        nowaiting_step* prs = rsvec.at(i);
                                        assert(!b_s_tmp->Dominate(prs));
                                        if(prs->Dominate(b_s_tmp)){
                                            flag = true;
                                            break;
                                        }
                                    }
                                    if(flag){
                                        delete b_s_tmp;
                                        b_s_tmp = NULL;
                                        continue;//
                                    }

                                }
                                assert(flag == false);//说明closelist中没有能够dominate b_s_tmp的step
                                std::vector<uint> rsvec_open;//由于不清楚直接调用clear函数是否会释放内存，这里先重新定义一个
                                if(Opened_nowaiting(bs, rsvec_open)){//判断是否open，并确定相应的base_step指针
                                    //b_s_tmp = new robust_step(bs,entertime,ioa_second,cost1_time+cost2_time,cost1_delay+cost2_delay);
                                    assert(!rsvec_open.empty());
                                    uint count = 0;//记录已经从openlist中删除的元素个数
                                    uint n = rsvec_open.size();
                                    for(uint i = 0; i<n; i++){
                                        nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsvec_open.at(i)-count);
                                        assert(prs->m_bs == bs);
                                        if(prs->Dominate(b_s_tmp)){
                                            flag = true;
                                            break;
                                        }
                                        else if(b_s_tmp->Dominate(prs)){
                                            openlistexp.erase(openlistexp.begin()+rsvec_open.at(i)-count);//从openlist中删除prs
                                            delete prs;//析构
                                            count++;
                                        }
                                    }
                                    if(flag){
                                        delete b_s_tmp;
                                        continue;//
                                    }
                                }

                                assert(flag == false);//说明openlist中没有可以dominate b_s_tmp的step
                                b_s_tmp->m_prestep = bs_pre;
                                openlistexp.push_back(b_s_tmp);//add to open
                            }
                        }
                    }
                }
                /*if current neighbor is an inter or a runway*/
                else if((pnb->m_type == Inter) || (pnb->m_type == Runway)){

//                    if(g_veh_id == 1 && neighbor_zone_id == 8
//                            && nd_id == 5){
//                        int xx = 0;
//                    }

                    if(!Overlapped_nowaiting(pew,pnb->GetTWVector(), paircount, wtt)){
                        continue;
                    }
                    else{
                        nowaiting_step* b_s_tmp;
                        Basicstep bs;
                        uint awid;
                        twindow entrytimewindow;
                        AW_ENTW_table_nowaiting::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            awid = itr->first;
                            entrytimewindow = itr->second;
                            //double costtmp_time = nnlength/avg_speed;
                            //double costtmp_delay = entrytimewindow.tstart - in_bs->m_entw.tstart - costtmp_time;
                            //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                            TernaryToUnary(neighbor_zone_id, nd_id, awid, bs);
                            NowaitingStepVector rsv;
                            b_s_tmp = new nowaiting_step(bs,entrytimewindow);
                            b_s_tmp->m_length = nnlength;
                            b_s_tmp->m_speed = avg_speed;

                            bool flag = false;
                            if(Closed_nowaiting(bs,rsv)){
                                assert(!rsv.empty());
                                uint n = rsv.size();
                                for(uint i=0; i<n; i++){
                                    nowaiting_step* prs = (nowaiting_step*)rsv.at(i);
                                    assert(!b_s_tmp->Dominate(prs));
                                    if(prs->Dominate(b_s_tmp)){
                                        flag = true;
                                        break;
                                    }
                                }
                                if(flag){
                                    delete b_s_tmp;
                                    b_s_tmp = NULL;
                                    continue;
                                }
                            }
                            assert(flag == false);
                            std::vector<uint> rsv_open;
                            if(Opened_nowaiting(bs, rsv_open)){
                                uint count = 0;//记录已经从openlist中删除的元素个数
                                assert(!rsv_open.empty());
                                uint n = rsv_open.size();
                                for(uint i=0; i<n; i++){
                                    nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsv_open.at(i)-count);
                                    assert(prs->m_bs == bs);
                                    if(prs->Dominate(b_s_tmp)){
                                        flag = true;
                                        break;
                                    }
                                    else if(b_s_tmp->Dominate(prs)){
                                        openlistexp.erase(openlistexp.begin()+rsv_open.at(i)-count);//从openlist中删除prs
                                        delete prs;//析构
                                        count++;
                                    }
                                }
                                if(flag){
                                    delete b_s_tmp;
                                    b_s_tmp = NULL;
                                    continue;
                                }
                            }
                            assert(flag == false);
                            b_s_tmp->m_prestep = in_bs;
                            //uint xx = openlistexp.size();
                            openlistexp.push_back(b_s_tmp);
                            //xx = openlistexp.size();
                        }
                    }
                }
                /*else if current neighbor is a stand or a buffer*/
                else{//stand和air-buffer内的冲突情况暂不考虑，所以：
//                    if(g_veh_id == 1 && neighbor_zone_id == 418){
//                        int xx = 0;
//                    }
                    nowaiting_step* b_s_tmp;
                    Basicstep bs;
                    //double entertime = ntoa(ioa_first,pew->tstart);
                    twindow entrytw = twindow(pew->tstart,pew->tend);
                    //double costtmp_time = nnlength/avg_speed;
                    //double costtmp_delay = entrytw.tstart - in_bs->m_entw.tstart - costtmp_time;
                    //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                    TernaryToUnary(neighbor_zone_id, nd_id, 0, bs);
                    NowaitingStepVector rsv;
                    bool flag = false;
                    b_s_tmp = new nowaiting_step(bs,entrytw);
                    b_s_tmp->m_length = nnlength;
                    b_s_tmp->m_speed = avg_speed;

                    if(Closed_nowaiting(bs,rsv)){
                        assert(!rsv.empty());
                        uint n = rsv.size();
                        for(uint i=0; i<n; i++){
                            nowaiting_step* prs = (nowaiting_step*)rsv.at(i);
                            assert(!b_s_tmp->Dominate(prs));
                            if(prs->Dominate(b_s_tmp)){
                                flag = true;
                                break;
                            }
                        }
                        if(flag){
                            delete b_s_tmp;
                            b_s_tmp = NULL;
                            continue;
                        }

                    }
                    assert(flag == false);
                    std::vector<uint> rsidv;
                    if(Opened_nowaiting(bs, rsidv)){
                        uint count = 0;//记录已经从openlist中删除的元素个数
                        assert(!rsidv.empty());
                        uint n = rsidv.size();
                        for(uint i=0; i<n; i++){
                            nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsidv.at(i)-count);//因为删除元素导致后面的元素位置前移了
                            assert(prs->m_bs == bs);//
                            if(prs->Dominate(b_s_tmp)){
                                flag = true;
                                break;
                            }
                            else if(b_s_tmp->Dominate(prs)){
                                openlistexp.erase(openlistexp.begin()+rsidv.at(i)-count);//从openlist中删除prs
                                delete prs;//析构
                                count++;//
                            }
                        }
                        if(flag){
                            delete b_s_tmp;
                            b_s_tmp = NULL;
                            continue;
                        }
                    }
                    assert(!flag);
                    b_s_tmp->m_prestep = in_bs;
                    openlistexp.push_back(b_s_tmp);
                }
            }
        }
    }
    delete pew;
}

void Expand_nowaiting_loopless(nowaiting_step *in_bs)// 2017-8-17 added by TZ
{
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
//    if(curnodeid==161&& currid == 86&& g_veh_id == 54){
//        int x = 0;
//    }
    uint curawid = GetAWindowIndex(in_bs->m_bs);//current free time window id
    CRegion* prgn = g_model.m_rct.GetRegion(currid);//pointer to current region
    assert(prgn->m_type != Line);//按照动态路由算法的原理，这里不应当出现Line类型的扩展基准
    if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
        return;
    }//（因此，expand事实上只对Inter和Runway类型的扩展基准进行扩展）
    assert((prgn->m_type == Inter) || (prgn->m_type == Runway));

    double avg_speed;//TODO:这里是否可以考虑增加转弯和直行时平均速度存在的差异？
    twindow* paw;
    paw = prgn->GetTWVector().at(curawid);
// for debug
    if (g_veh_id == 9 && currid == 251 && curnodeid == 559 && curawid ==1){
        qDebug() << QString("region %1, node %2, entw [%3, %4], preregion %5, prenode %6").arg(currid).arg(curnodeid).arg(in_bs->m_entw.tstart).arg(in_bs->m_entw.tend)
                    .arg(GetRegionIndex( in_bs->m_prestep->m_bs)).arg(GetNodeIndex(in_bs->m_prestep->m_bs));

    }

    CRegion* pnb;// pointer to neighbor region
    double nnlength;//length to the neighbor node
    twindow* pew = new twindow;
    uint ndcount = prgn->m_count_node;//number of nodes contained in current region
    uint nd_id;//node id
    uint neighbor_zone_id;
    char dir_nb;
    twindow* pnbaw = 0;
    Basicstep curbs;
    twindow entw;
    cnode* pnode ;
    g_accesscount += ndcount-1; //分枝个数等于进行可达性判定的次数，完成对所有分枝的可达性判定等于完成一次扩展
    //double ioa_first;
    //double ioa_second;
    uint act;//
    int isTurn;
    nowaiting_step* bs_pre;
    nowaiting_step* b_s_lane;
    bool flag_lane = false;
    bool flag_reachable = true;
    //对当前区域包含的每个节点ndid（除当前步骤对应的节点外），判定由当前区域和ndid确定的相邻区域的可达性
    for(uint i=0; i<ndcount; i++){
        nd_id = prgn->m_vector_node.at(i);
        pnode = g_model.m_ndct.m_array_node[nd_id];
        if(nd_id == curnodeid){
            continue;
        }
        // loop checking
        uint loopflag = 0;
        nowaiting_step* pnwstmp  = (nowaiting_step*)in_bs->m_prestep;
        uint rgntmp = GetRegionIndex(pnwstmp->m_bs);
        while(rgntmp != g_start_region){
            if(rgntmp == neighbor_zone_id){
                loopflag = 1;
                break;
            }
            pnwstmp = (nowaiting_step*)pnwstmp->m_prestep;
            rgntmp = GetRegionIndex(pnwstmp->m_bs);
        }
        if(loopflag == 1){
            continue;
        }
        //
        else{
            bs_pre = in_bs;
            //curbs = in_bs->m_bs;//initialize current bs as the input bs for expansion
            neighbor_zone_id = pnode->GetAnotherZoneId(currid);
            if(neighbor_zone_id == 1000){//1000表示尚未包含在模型内的区域
                continue;
            }
            pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//pointer to the neighbor region
            if(g_model.matrix_nn_conn[curnodeid][nd_id] == 0 ){//若从curnodeid到nd_id不可行（无滑行路线或该方向的滑行被禁止），则退出本次循环
                continue;
            }
            nnlength = g_model.matrix_nn_dis[curnodeid][nd_id];//length between current node and the neighbor node
            uchar holdable = g_model.matrix_nn_hold[curnodeid][nd_id];
            act = getAction_nowaiting(currid,curnodeid,neighbor_zone_id);
            isTurn = g_model.matrix_nn_isTurn[curnodeid][nd_id];
            avg_speed = getSpeed_nowaiting(currid,act,isTurn);
            //ioa_first = getIOA(in_bs->m_IOA,nnlength,avg_speed,k);


            uint paircount = 0;
            AW_ENTW_table_nowaiting wtt;
            //这里的扩展基准只可能是Inter或Runway型，所以:
            if(ExitWindow_Intersection_nowaiting(in_bs->m_entw, nnlength, paw, pew, avg_speed,holdable)){
                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){
//                    if(g_veh_id == 362 && neighbor_zone_id == 164){
//                        int xx = 0;
//                    }
                    uint next_node_id;
                    uint next_rgn_id;
                    flag_lane = true;
                    while(flag_lane){
                        if(pnb->m_vector_node.at(0)==nd_id){
                            next_node_id = pnb->m_vector_node.at(1);
                        }
                        else{
                            next_node_id = pnb->m_vector_node.at(0);
                        }
                        next_rgn_id = g_model.m_ndct.m_array_node[next_node_id]->GetAnotherZoneId(neighbor_zone_id);
                        if((next_rgn_id == 1000) || (g_model.matrix_nn_conn[nd_id][next_node_id] == 0)){
                            flag_reachable = false;
                            break;
                        }
                        uint awid_line;
                        dir_nb = pnb->GetDirection(nd_id);
                        if(!EntryWindow_Line_nowaiting(pnb->GetTWVector(dir_nb), pew, pnb->GetCapacity(),avg_speed, entw, awid_line)){
                            flag_reachable = false;
                            break;//
                        }
                        else{//若该路段可以进入，需要进一步确定是否可脱离
                            pnbaw = pnb->GetTWVector(dir_nb).at(awid_line);
                            assert(pnb->m_count_node == 2);
                            //注意，下面nnlength和avg_speed被重新赋值，表示从路段入口到下一个交叉口入口的距离和速度，所以这里先备份
                            double nnlength_pre = nnlength;
                            double avg_speed_pre = avg_speed;
                            nnlength = g_model.matrix_nn_dis[nd_id][next_node_id];//length to the other node of the lane
                            holdable = g_model.matrix_nn_hold[nd_id][next_node_id];
                            act = getAction_nowaiting(neighbor_zone_id,nd_id,next_rgn_id);
                            isTurn = g_model.matrix_nn_isTurn[nd_id][next_node_id];
                            avg_speed = getSpeed_nowaiting(neighbor_zone_id,act,isTurn);
                            if(!ExitWindow_Line_nowaiting(entw, pnbaw, pnb->GetCapacity(), nnlength, pew, avg_speed, holdable)){
                                flag_reachable = false;
                                break;//if the lane is not exit-able, continue
                            }
                            else{
                                //至此，表明在当前路段区域可脱离。与该路段相邻的下一区域为Inter或Runway:
                                TernaryToUnary(neighbor_zone_id, nd_id, awid_line, curbs);//update the curbs
                                b_s_lane = new nowaiting_step(curbs, entw);
                                b_s_lane->m_length = nnlength_pre;
                                b_s_lane->m_speed = avg_speed_pre;
                                b_s_lane->m_prestep = bs_pre;
                                g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存

                                bs_pre = b_s_lane;//
                                nd_id = next_node_id;
                                neighbor_zone_id = next_rgn_id;

                                pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//指向下一顶点
                                if(pnb->m_type != Line){
                                    flag_lane = false;
                                }
                            }
                        }
                    }
                    if(flag_reachable){
                        if(!Overlapped_nowaiting(pew, pnb->GetTWVector(), paircount, wtt)){
                            continue;
                        }
                        else{
                            nowaiting_step* b_s_tmp;
                            Basicstep bs;
                            uint awid;
                            twindow entrytimewindow;
                            AW_ENTW_table_nowaiting::iterator itr;
                            for(itr=wtt.begin();itr!=wtt.end();itr++){
                                awid = itr->first;
                                entrytimewindow = itr->second;
                                //double cost2_time = nnlength/avg_speed;
                                //double cost2_delay = entrytimewindow.tstart - entw.tstart - cost2_time;
                                //double cost2 = calCost(nnlength,avg_speed,entertime-tenter);
                                TernaryToUnary(next_rgn_id, next_node_id, awid, bs);
                                NowaitingStepVector rsvec;
                                bool flag = false;
                                b_s_tmp = new nowaiting_step(bs,entrytimewindow);
                                b_s_tmp->m_length = nnlength;
                                b_s_tmp->m_speed = avg_speed;

                                if(Closed_nowaiting(bs,rsvec)){
                                    uint n = rsvec.size();
                                    for(uint i=0; i<n; i++){
                                        nowaiting_step* prs = rsvec.at(i);
                                        assert(!b_s_tmp->Dominate(prs));
                                        if(prs->Dominate(b_s_tmp)){
                                            flag = true;
                                            break;
                                        }
                                    }
                                    if(flag){
                                        delete b_s_tmp;
                                        b_s_tmp = NULL;
                                        continue;//
                                    }

                                }
                                assert(flag == false);//说明closelist中没有能够dominate b_s_tmp的step
                                std::vector<uint> rsvec_open;//由于不清楚直接调用clear函数是否会释放内存，这里先重新定义一个
                                if(Opened_nowaiting(bs, rsvec_open)){//判断是否open，并确定相应的base_step指针
                                    //b_s_tmp = new robust_step(bs,entertime,ioa_second,cost1_time+cost2_time,cost1_delay+cost2_delay);
                                    assert(!rsvec_open.empty());
                                    uint count = 0;//记录已经从openlist中删除的元素个数
                                    uint n = rsvec_open.size();
                                    for(uint i = 0; i<n; i++){
                                        nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsvec_open.at(i)-count);
                                        assert(prs->m_bs == bs);
                                        if(prs->Dominate(b_s_tmp)){
                                            flag = true;
                                            break;
                                        }
                                        else if(b_s_tmp->Dominate(prs)){
                                            openlistexp.erase(openlistexp.begin()+rsvec_open.at(i)-count);//从openlist中删除prs
                                            delete prs;//析构
                                            count++;
                                        }
                                    }
                                    if(flag){
                                        delete b_s_tmp;
                                        continue;//
                                    }
                                }

                                assert(flag == false);//说明openlist中没有可以dominate b_s_tmp的step
                                b_s_tmp->m_prestep = bs_pre;
                                openlistexp.push_back(b_s_tmp);//add to open
                            }
                        }
                    }
                }
                /*if current neighbor is an inter or a runway*/
                else if((pnb->m_type == Inter) || (pnb->m_type == Runway)){

//                    if(g_veh_id == 54 && neighbor_zone_id == 134
//                            && nd_id == 167){
//                        int xx = 0;
//                    }

                    if(!Overlapped_nowaiting(pew,pnb->GetTWVector(), paircount, wtt)){
                        continue;
                    }
                    else{
                        nowaiting_step* b_s_tmp;
                        Basicstep bs;
                        uint awid;
                        twindow entrytimewindow;
                        AW_ENTW_table_nowaiting::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            awid = itr->first;
                            entrytimewindow = itr->second;
                            //double costtmp_time = nnlength/avg_speed;
                            //double costtmp_delay = entrytimewindow.tstart - in_bs->m_entw.tstart - costtmp_time;
                            //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                            TernaryToUnary(neighbor_zone_id, nd_id, awid, bs);
                            NowaitingStepVector rsv;
                            b_s_tmp = new nowaiting_step(bs,entrytimewindow);
                            b_s_tmp->m_length = nnlength;
                            b_s_tmp->m_speed = avg_speed;

                            bool flag = false;
                            if(Closed_nowaiting(bs,rsv)){
                                assert(!rsv.empty());
                                uint n = rsv.size();
                                for(uint i=0; i<n; i++){
                                    nowaiting_step* prs = (nowaiting_step*)rsv.at(i);
                                    assert(!b_s_tmp->Dominate(prs));
                                    if(prs->Dominate(b_s_tmp)){
                                        flag = true;
                                        break;
                                    }
                                }
                                if(flag){
                                    delete b_s_tmp;
                                    b_s_tmp = NULL;
                                    continue;
                                }
                            }
                            assert(flag == false);
                            std::vector<uint> rsv_open;
                            if(Opened_nowaiting(bs, rsv_open)){
                                uint count = 0;//记录已经从openlist中删除的元素个数
                                assert(!rsv_open.empty());
                                uint n = rsv_open.size();
                                for(uint i=0; i<n; i++){
                                    nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsv_open.at(i)-count);
                                    assert(prs->m_bs == bs);
                                    if(prs->Dominate(b_s_tmp)){
                                        flag = true;
                                        break;
                                    }
                                    else if(b_s_tmp->Dominate(prs)){
                                        openlistexp.erase(openlistexp.begin()+rsv_open.at(i)-count);//从openlist中删除prs
                                        delete prs;//析构
                                        count++;
                                    }
                                }
                                if(flag){
                                    delete b_s_tmp;
                                    b_s_tmp = NULL;
                                    continue;
                                }
                            }
                            assert(flag == false);
                            b_s_tmp->m_prestep = in_bs;
                            //uint xx = openlistexp.size();
                            openlistexp.push_back(b_s_tmp);
                            //xx = openlistexp.size();
                        }
                    }
                }
                /*else if current neighbor is a stand or a buffer*/
                else{//stand和air-buffer内的冲突情况暂不考虑，所以：
//                    if(g_veh_id == 1 && neighbor_zone_id == 418){
//                        int xx = 0;
//                    }
                    nowaiting_step* b_s_tmp;
                    Basicstep bs;
                    //double entertime = ntoa(ioa_first,pew->tstart);
                    twindow entrytw = twindow(pew->tstart,pew->tend);
                    //double costtmp_time = nnlength/avg_speed;
                    //double costtmp_delay = entrytw.tstart - in_bs->m_entw.tstart - costtmp_time;
                    //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                    TernaryToUnary(neighbor_zone_id, nd_id, 0, bs);
                    NowaitingStepVector rsv;
                    bool flag = false;
                    b_s_tmp = new nowaiting_step(bs,entrytw);
                    b_s_tmp->m_length = nnlength;
                    b_s_tmp->m_speed = avg_speed;

                    if(Closed_nowaiting(bs,rsv)){
                        assert(!rsv.empty());
                        uint n = rsv.size();
                        for(uint i=0; i<n; i++){
                            nowaiting_step* prs = (nowaiting_step*)rsv.at(i);
                            assert(!b_s_tmp->Dominate(prs));
                            if(prs->Dominate(b_s_tmp)){
                                flag = true;
                                break;
                            }
                        }
                        if(flag){
                            delete b_s_tmp;
                            b_s_tmp = NULL;
                            continue;
                        }

                    }
                    assert(flag == false);
                    std::vector<uint> rsidv;
                    if(Opened_nowaiting(bs, rsidv)){
                        uint count = 0;//记录已经从openlist中删除的元素个数
                        assert(!rsidv.empty());
                        uint n = rsidv.size();
                        for(uint i=0; i<n; i++){
                            nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsidv.at(i)-count);//因为删除元素导致后面的元素位置前移了
                            assert(prs->m_bs == bs);//
                            if(prs->Dominate(b_s_tmp)){
                                flag = true;
                                break;
                            }
                            else if(b_s_tmp->Dominate(prs)){
                                openlistexp.erase(openlistexp.begin()+rsidv.at(i)-count);//从openlist中删除prs
                                delete prs;//析构
                                count++;//
                            }
                        }
                        if(flag){
                            delete b_s_tmp;
                            b_s_tmp = NULL;
                            continue;
                        }
                    }
                    assert(!flag);
                    b_s_tmp->m_prestep = in_bs;
                    openlistexp.push_back(b_s_tmp);
                }
            }
        }
    }
    delete pew;
}

bool Overlapped_nowaiting(twindow* in_ptw, TWVector& in_twv, uint& out_count, AW_ENTW_table_nowaiting& out_wtt)
{
    //这里不仅要判断是否存在overlap，还要根据IOA进一步排除无意义的overlap，最后只保留足够大的overlap信息
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    out_count = 0;
    twindow entw;//entry time window
    if(twnum == 0){
        return false;
    }
    else{
        for(uint i=0; i<twnum; i++){
            ptwtmp = in_twv[i];
            if(EntryWindow_Intersection_nowaiting(in_ptw, ptwtmp, entw)){
                out_count++;
                out_wtt.insert(AW_ENTW_table_nowaiting::value_type(i, entw));
            }
        }
        return (out_count > 0);
    }
}

bool EntryWindow_Intersection_nowaiting(twindow* in_ptw1, twindow* in_ptw2, twindow& out_entw)
{
    //double len = in_ptw2->tend - in_ptw2->tstart;

    double start = fmax(in_ptw1->tstart,in_ptw2->tstart);
    double end = fmin(in_ptw1->tend,in_ptw2->tend);
    if(end > start){
        out_entw.tstart = start;
        out_entw.tend = end;
        return true;
    }
    return false;

}

void GetMinHeuristicCostOfOpenlist_nowaiting(nowaiting_step *&out_bs)
{
    out_bs = (nowaiting_step*)openlistexp.at(0);
    uint index = 0;
    nowaiting_step* bs;
    for(uint i=1; i < openlistexp.size();i++){
        bs = (nowaiting_step*)openlistexp.at(i);
        //breadth-first
        if(out_bs->m_entw.tstart+g_array_h[GetNodeIndex(out_bs->m_bs)]/g_runway_speed - (bs->m_entw.tstart+g_array_h[GetNodeIndex(bs->m_bs)]/g_runway_speed) > 1e-3){//注意，这里就简单的基于entry time window的起始时刻来选择step进行扩展，是否可以有更好的策略？
            out_bs = bs;
            index = i;
        }

    }
    //将它从openlist中取出,并加入closelist
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}

void GetMinHeuristicCostOfOpenlist_nowaiting_deepfirst(nowaiting_step *&out_bs)
{
    out_bs = (nowaiting_step*)openlistexp.at(0);
    uint index = 0;
    nowaiting_step* bs;
    for(uint i=1; i < openlistexp.size();i++){
        bs = (nowaiting_step*)openlistexp.at(i);
        //breadth-first
        if(g_array_h[GetNodeIndex(out_bs->m_bs)] - g_array_h[GetNodeIndex(bs->m_bs)] > 1e-3){//注意，这里就简单的基于entry time window的起始时刻来选择step进行扩展，是否可以有更好的策略？
            out_bs = bs;
            index = i;
        }

    }
    //将它从openlist中取出,并加入closelist
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}

bool ExitWindow_Intersection_nowaiting(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable)
{
    double delta = 5;//航空器机身完全脱离当前区域需要的时间
    double ratio = 0.1;//当前区段通行时间弹性系数为0.1
    double traverse = in_nnlength/in_speed;//区段通行时间

    double tee = in_entw.tstart + traverse;//earliest time of exit
    double tle;//latest time of exit
    if(in_holdable){
        tle = in_paw->tend - delta;
    }
    else{
        tle = fmin(in_entw.tend+traverse*(1+ratio),in_paw->tend - delta);
    }
    if(tle < tee){
        return false;
    }
    else{
        if(in_entw.tstart < in_paw->tstart){
            qDebug("error, the enter time is not among the aw");
            return false;
        }
        out_pew->tstart = tee;
        out_pew->tend = fmin(tle,out_pew->tstart+c_waitmax);
        return true;

    }
}

bool EntryWindow_Line_nowaiting(TWVector& in_twv, twindow* in_ew, uchar in_cap, double in_speed, twindow &out_entw, uint& out_awid)
{
    double delta = 5;
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    twindow entw;
    //uint index;
    //int index_out1 = -1;
    //int index_out2 = -1;
    rOccVariable* poccv;
    rOccVariable* poccpre;
    int status = 0;//denote special status of the in-out vector: when there is an in and an out appear at same time
    for(uint i=0; i<twnum; i++){
        ptwtmp = in_twv.at(i);
        int index_in1 = -1;
        int index_in2 = ptwtmp->m_occ_vector_robust.size();
        //分析可知，区域控制规则决定了唯一可能的enter-able情形如下
            if((in_ew->tstart > ptwtmp->tstart) && (in_ew->tstart < ptwtmp->tend)){
                out_awid = i;
                //若ORS为空
                if(ptwtmp->m_occ_vector_robust.empty()){
                    out_entw.tstart = in_ew->tstart;
                    out_entw.tend = fmin(in_ew->tend,ptwtmp->tend);
                    return true;
                }
                //else,确定当前航空器占用时刻在ORS中的位置
                assert(ptwtmp->m_occ_vector_robust.size()>=2);//若不为空，ORS中元素个数必定大于等于2
                int tmp = -1;
                for(uint index = 0; index < ptwtmp->m_occ_vector_robust.size();index++){
                    poccv = ptwtmp->m_occ_vector_robust.at(index);
                    if(poccv->m_action == 1){
                        if(poccv->m_time > in_ew->tstart ){
                            index_in2 = index;
                            index_in1 =tmp;
                            break;
                        }
                        tmp = index;
                    }
                }
                //若是最后一个
                if(index_in2 == ptwtmp->m_occ_vector_robust.size()){
                    rOccVariable* prov = ptwtmp->m_occ_vector_robust.at(tmp);//之前最后一个进入该区域的航空器的占用变量
                    if(prov->m_count == in_cap){
                        assert(tmp+1 < ptwtmp->m_occ_vector_robust.size());
                        rOccVariable* ptmp = ptwtmp->m_occ_vector_robust.at(tmp+1);
                        assert(ptmp->m_count <in_cap);
                        assert(ptmp->m_action == -1);
                        double ts = ptmp->m_time+delta;////
                        //double tau = ntoa(in_ioa,ts);
                        out_entw.tstart = fmax(in_ew->tstart,ts);
                        out_entw.tend = fmin(in_ew->tend,ptwtmp->tend);
                        if(out_entw.tstart < out_entw.tend)
                            return true;
                        else
                            return false;

                    }
                    else{
                        double ts = prov->m_time+delta;
                        out_entw.tstart = fmax(in_ew->tstart,ts);
                        out_entw.tend = fmin(in_ew->tend,ptwtmp->tend);
                        if(out_entw.tstart < out_entw.tend)
                            return true;
                        else
                            return false;

                    }
                }
                //若是第一个
                else if(index_in1 == -1){
                    assert(index_in2 == 0);
                    rOccVariable* prov = ptwtmp->m_occ_vector_robust.at(0);
                    double tee = in_ew->tstart;
                    double tle = fmin(in_ew->tend,prov->m_time-delta);
                    if(tle > tee){
                        out_entw.tstart = tee;
                        out_entw.tend = tle;
                        return true;
                    }
                    else{
                        return false;
                    }
                }
                //一般情况
                else{

                    poccpre = ptwtmp->m_occ_vector_robust.at(index_in1);
                    poccv = ptwtmp->m_occ_vector_robust.at(index_in2);
                    double tl = poccpre->m_time+delta;
                    double tr = poccv->m_time-delta;
                    double left = fmax(tl,in_ew->tstart);
                    double right = fmin(tr,in_ew->tend);
                    if(right < left){
                        return false;
                    }
                    if(poccpre->m_count < in_cap){
                        out_entw.tstart = left;
                        out_entw.tend = right;
                        return true;
                    }
                    else{
                        /*if(g_priority == 88){
                            qDebug("...");
                        }*/

                        assert(poccpre->m_count == in_cap);//满容量了
                        for(uint index= index_in1+1; index<index_in2;index++){
                            poccv = ptwtmp->m_occ_vector_robust.at(index);
                            assert(poccv->m_count<=in_cap);
                            if(poccv->m_count < in_cap){
                                assert(poccv->m_action == -1);
                                left = fmax(left,poccv->m_time+delta);////
                                if(right > left){
                                    out_entw.tstart = left;
                                    out_entw.tend = right;
                                    return true;
                                }
                                else{
                                    return false;
                                }
                            }
                       }
                  }
              }
              break;//因为最多只可能有一个可达的时间窗，所以不必再继续检查其余的可用时间窗
         }
    }
    return false;
}

bool ExitWindow_Line_nowaiting(twindow in_entw,twindow* in_aw,
                            uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed, uchar in_hold)
{
    assert(in_entw.tstart >= in_aw->tstart);
    double ratio = 0.1;//当前区段通行时间弹性系数为0.1
    double traverse = in_nnlength/in_speed;//区段通行时间
    double delta = 5;//for 机身

    /*if the occvector is empty, it's straightforward*/
    if(in_aw->m_occ_vector_robust.size() == 0){
        double tee = in_entw.tstart + traverse;
        double tle;
        if(in_hold){
            tle = in_aw->tend - delta;
        }
        else{
            tle = fmin(in_entw.tend + traverse*(1+ratio), in_aw->tend - delta);
        }
        if(tle > tee){
            out_pew->tstart = tee;
            out_pew->tend = fmin(tle,out_pew->tstart+c_waitmax);
            return true;
        }
        else{
            return false;
        }
    }


    /*else, */
    assert(in_aw->m_occ_vector_robust.size() >= 2);
    rOccVariable* pocc;
    uchar precount = 0;
    uint out = 0;
    uint i;
    char status = 1;
    for(i=0;i<in_aw->m_occ_vector_robust.size();i++){
        pocc = in_aw->m_occ_vector_robust.at(i);
        if(pocc->m_time > in_entw.tstart){
            if(i == 0){
//                if(g_veh_id == 6447 && fabs(pocc->m_time - 971269.5)<1){
//                    int x =0;
//                }
                precount = 0;
            }
            else{
                pocc = in_aw->m_occ_vector_robust.at(i-1);
                precount = pocc->m_count;//当前区域中的航空器个数
            }
            break;
        }
    }
    double tl;//临时,存放输出ew的左右边界值
    double tr;
    if(i == 0){//若当前规划航空器先于之前进入该路段的所有航空器进入该路段
        tl = traverse + in_entw.tstart;
        rOccVariable* ptmp;
        double time_tmp;
        for(;i<in_aw->m_occ_vector_robust.size();i++){
            ptmp = in_aw->m_occ_vector_robust.at(i);
            if(ptmp->m_action == 1){
                if(ptmp->m_count == in_cap){//遇到了满容量时间窗
                    time_tmp = ptmp->m_time - delta;
                    if(time_tmp < tl-1e-3){//容量冲突
                        return false;
                    }
                    else{
                        //tr还受no-hold的约束:
                        if(in_hold){
                            tr = time_tmp;
                        }
                        else{
                            tr = fmin(time_tmp,in_entw.tend + traverse*(1+ratio));
                        }
                        break;
                    }
                }

            }
            else if(ptmp->m_action == -1){//第一个进入该路段，所以也要第一个脱离
                if(in_hold){
                    tr = ptmp->m_time - delta;
                }
                else{
                    tr = fmin(ptmp->m_time - delta,in_entw.tend + traverse*(1+ratio));//此时，当前航空器要先于其他航空器脱离该路段
                }
                break;//找到之前第一个脱离动作后即跳出循环
            }
            /*if(in_aw->m_occ_vector_robust.at(i)->m_count == in_cap){//若在第一个out动作之前遇到满容量时间窗
                tr = ioaStart(in_aw->m_occ_vector_robust.at(i)->m_ioa,in_aw->m_occ_vector_robust.at(i)->m_time);
                break;
            }*/
        }
        if(tr>tl){
            out_pew->tstart = tl;
            out_pew->tend = fmin(tr,out_pew->tstart+c_waitmax);
            return true;
        }
        else{
            return false;
        }
    }
    else if(i == in_aw->m_occ_vector_robust.size()){//if the entry time lie out of all the occpancy records, it's also straightforward
        pocc = in_aw->m_occ_vector_robust.at(in_aw->m_occ_vector_robust.size()-1);//最后一个out动作
        assert(pocc->m_action == -1);
        out_pew->tstart = fmax(pocc->m_time+delta,in_entw.tstart+traverse);
        if(in_hold){
           out_pew->tend = in_aw->tend - delta;
        }
        else{
            out_pew->tend = fmin(in_entw.tend+traverse*(1+ratio),in_aw->tend - delta);
        }
        if(out_pew->tend > out_pew->tstart){
            out_pew->tend = fmin(out_pew->tend,out_pew->tstart+c_waitmax);
            return true;
        }
        else{
            return false;
        }
    }
    else{//一般的情况
        tl = traverse + in_entw.tstart;//初始化tl和tr
        if(in_hold){
           tr = in_aw->tend - delta;
        }
        else{
            tr = fmin(in_entw.tend+traverse*(1+ratio),in_aw->tend - delta);
        }
        if(precount == 0){//表明之前进入的航空器已经全部离开
            pocc = in_aw->m_occ_vector_robust.at(i-1);
            assert(pocc->m_action == -1);
            double tmp = pocc->m_time+delta;
            if(tl < tmp){
                tl = tmp;
            }
            for(;i<in_aw->m_occ_vector_robust.size();i++){
                pocc = in_aw->m_occ_vector_robust.at(i);
                if(pocc->m_action == 1){
                    if(pocc->m_count == in_cap){
                        tr = fmin(pocc->m_time-delta,tr);
                        break;
                    }
                }
                else if(pocc->m_action == -1){
                    tr = fmin(pocc->m_time-delta,tr);
                    break;
                }
            }
            if(tr>tl){
                out_pew->tstart = tl;
                out_pew->tend = fmin(tr,out_pew->tstart+c_waitmax);
                return true;
            }
            else{
                return false;
            }
        }
        else{//未离开的航空器个数大于零
            for(;i<in_aw->m_occ_vector_robust.size();i++){
                pocc = in_aw->m_occ_vector_robust.at(i);
                if(pocc->m_action == -1){
                    out++;
                }
                switch(status){//初始化=1
                case 1://还有先前进入该路段的航空器未离开该路段
                    if(pocc->m_count == in_cap){//在先前进入的航空器全部脱离当前区域前遇到满容量时间窗则表明不可脱离，直接返回
                        return false;
                    }
                    if(out == precount){
                        status = 2;
                        double tmp = pocc->m_time+delta;//最早可以脱离的时刻
                        if(tl < tmp){
                            tl = tmp;
                        }
                    }
                    break;
                case 2://确定了先前进入该路段的航空器中最晚离开该路段的脱离时刻后执行以下case
                    if(out == (precount+1)){
                        status = 3;
                        tr = fmin(pocc->m_time-delta, tr);
                        if(tr>tl){
                            out_pew->tstart = tl;
                            out_pew->tend = fmin(tr,out_pew->tstart+c_waitmax);
                            return true;
                        }
                        else{
                            return false;
                        }
                    }
                    else if(pocc->m_count == in_cap){
                        tr = fmin(tr,pocc->m_time-delta);
                        if(tr>tl){
                            out_pew->tstart = tl;
                            out_pew->tend = fmin(tr,out_pew->tstart+c_waitmax);
                            return true;
                        }
                        else{
                            return false;
                        }
                    }
                    break;

                }

            }
        }

    }
    assert(status != 1);
    //if(status == 1){
    //    int x = 0;
    //}
    if(status == 2){
        if(tr>tl){
            out_pew->tstart = tl;
            out_pew->tend = fmin(tr,out_pew->tstart+c_waitmax);
            return true;
        }
        else{
            return false;
        }
    }
}


bool Closed_nowaiting(Basicstep in_bs, NowaitingStepVector& rsv)
{
    nowaiting_step* pb_s;
    for(uint i=0; i<closelist.size();i++){
        pb_s = (nowaiting_step*)closelist.at(i);
        if(pb_s->m_bs == in_bs){
            rsv.push_back(pb_s);
        }
    }
    if(rsv.empty()){
        return false;
    }
    else{
        return true;
    }
}

bool Opened_nowaiting(Basicstep in_bs, std::vector<uint> &rsv)
{
    rsv.clear();
    nowaiting_step* pb_s;
    for(uint i=0; i<openlistexp.size();i++){
        pb_s = (nowaiting_step*)openlistexp.at(i);
        if(pb_s->m_bs == in_bs){
            rsv.push_back(i);//注意这里存的是元素的index(并且是按index从小到大存储的)
        }
    }
    if(rsv.empty()){
        return false;
    }
    else{
        return true;
    }

}




void UpdateTimeWindowVector_nowaiting(Path& in_path, CRegionCollect &rct)
{
    if(in_path.empty()){
        return;
    }
    double delta=5;
    uint act;
    double avg_speed;

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
    nowaiting_step* pb_s1;
    nowaiting_step* pb_s2;
    uint n = in_path.size();//
    int isTurn;
    for(uint i = 0; i<n-1; i++){
        pb_s1 = (nowaiting_step*)in_path.at(i);
        pb_s2 = (nowaiting_step*)in_path.at(i+1);
        rngid = GetRegionIndex(pb_s1->m_bs);
        ndid = GetNodeIndex(pb_s1->m_bs);
        act =  getAction_nowaiting(rngid,ndid,GetRegionIndex(pb_s2->m_bs));
        isTurn = g_model.matrix_nn_isTurn[ndid][GetNodeIndex(pb_s2->m_bs)];
        avg_speed = getSpeed_nowaiting(rngid,act,isTurn);

        tw.tstart = pb_s1->m_entrytime;
        tw.tend = pb_s2->m_entrytime+delta;//考虑了机身的脱离时间

        prgn = rct.GetRegion(rngid);
        if(prgn->m_type == Line){
            dir =prgn->GetDirection(ndid);
            RemoveTimewindow_Line_nowaiting(prgn->GetTWVector(1-dir),&tw);//从反向可用时间窗集合中去除tw
            UpdateOccupation_nowaiting(prgn->GetTWVector(dir), pb_s1, pb_s2, prgn->m_num_capacity);//根据tw更新当前所在可用时间窗的占用记录
        }
        else{//对Runway类似Inter进行处理（独占）
            RemoveTimewindow_Inter_nowaiting(prgn->GetTWVector(), &tw);
        }
        //stand和buffer暂不处理
    }
}


void RemoveTimewindow_Line_nowaiting(TWVector& twv, twindow* in_ptw)
{
    twindow* ptw;
    twindow* pnewtw;
    double tstart = in_ptw->tstart;
    double tend = in_ptw->tend;
    double tmp;
    uint index_aw;
    uint i;

    //确定in_ptw所在时间窗
    for(i=0; i<twv.size(); i++){
        ptw = twv.at(i);
        if(tstart < ptw->tend){
            index_aw = i;
            break;
        }
    }
    assert(i != twv.size());
    ptw = twv.at(index_aw);
    //从该时间窗中去除in_ptw
    if(tend < ptw->tstart+1e-3){//若in_ptw恰好落在时间窗之间的空白处，直接返回
        return;
    }
    else if(tend > ptw->tend+1e-3){//若in_ptw长度超过了当前time window(注意这里因为是从反向可用时间窗中去除占用时间窗，所以有可能超过)
        tmp = tstart;
        tstart = ptw->tend;
        ptw->tend = tmp;
        if((ptw->tend - ptw->tstart) < c_twlengthmin){
            twv.erase(twv.begin()+i);
            delete ptw;
        }
    }
    else if(tstart < ptw->tstart+1e-3){
        ptw->tstart = tend;
        if((ptw->tend - ptw->tstart) < c_twlengthmin){
            twv.erase(twv.begin()+i);
            delete ptw;
        }
    }
    else{
        double gaps = (tstart - ptw->tstart);
        double gape = (ptw->tend - tend);
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
                if(ptw->m_occ_vector_robust.size() > 0){
                    rOccVariable* pocc;
                    uint index;//
                    uint loop;
                    for(loop=0; loop < ptw->m_occ_vector_robust.size();loop++){
                        pocc = ptw->m_occ_vector_robust.at(loop);
                        if(pocc->m_time > ptw->tend){
                            index = loop;
                            break;
                        }
                    }

                    if(loop!=ptw->m_occ_vector_robust.size()){
                        //if(loop%2 != 0){
                        //    int error = 1;
                        //}
                        for(uint i = index ;i<ptw->m_occ_vector_robust.size();i++){
                            pocc = ptw->m_occ_vector_robust.at(i);
                            pnewtw->m_occ_vector_robust.push_back(pocc);

                        }

                        ptw->m_occ_vector_robust.erase(ptw->m_occ_vector_robust.begin()+index, ptw->m_occ_vector_robust.end());
                    }

                }

            }
            else{
                ptw->tend = tstart;
            }
        }
    }
}

void UpdateOccupation_nowaiting(TWVector& twv, nowaiting_step* pb_s, nowaiting_step* pb_e, uint in_cap)
{
    twindow* ptw;
    uint i;

    for(i=0;i<twv.size();i++){
        ptw = twv.at(i);
        if(pb_s->m_entrytime < ptw->tend){//确定了占用发生时刻所在的时间窗
            assert(pb_s->m_entrytime > ptw->tstart);
            break;
        }
    }
    assert(i!= twv.size());
    rOccVector& occv = ptw->m_occ_vector_robust;
    //if the occvector is empty, it's straightforward
    if((occv.size() == 0)){
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, 0, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,0,-1);
        newocc1->m_pair = newocc2;
        newocc2->m_pair = newocc1;
        newocc1->m_id = g_veh_id;
        newocc2->m_id = g_veh_id;
        occv.push_back(newocc1);
        occv.push_back(newocc2);
        return;
    }
    //否则，occv.size()应该为偶数
    assert( occv.size()%2 == 0);//otherwise, it's a little complicated
    assert(occv.at(0)->m_action == 1);//第一个为进入动作
    assert(occv.at(occv.size()-1)->m_action == -1);//最后一个为离开动作离开

    uint indexstart=0;
    uint indexend=occv.size();
    uint flag_start = 0;
    uint flag_end = 0;
    uint curcap;
    rOccVariable* poc;
    if(occv.at(0)->m_time >= pb_e->m_entrytime){//若当前航空器先于其他航空器进入并离开该路段
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, 0, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,0,-1);
        newocc1->m_pair = newocc2;
        newocc2->m_pair = newocc1;
        newocc1->m_id = g_veh_id;
        newocc2->m_id = g_veh_id;
        occv.insert(occv.begin(),newocc1);
        occv.insert(occv.begin()+1,newocc2);
        //inoutv.insert(inoutv.begin(),1);
        //inoutv.insert(inoutv.begin()+1,-1);
        return;
    }
    else if(occv.at(occv.size()-1)->m_time <= pb_s->m_entrytime){//若当前航空器在先前所有航空器离开该路段后进入并离开
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, 0, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,0,-1);
        newocc1->m_pair = newocc2;
        newocc2->m_pair = newocc1;
        newocc1->m_id = g_veh_id;
        newocc2->m_id = g_veh_id;
        occv.push_back(newocc1);
        occv.push_back(newocc2);
        //inoutv.push_back(1);
        //inoutv.push_back(-1);
        return;
    }
    else{//一般情况
        for(i=0;i<occv.size();i++){
            poc = occv.at(i);
            if(flag_start == 0){
                if(poc->m_time > pb_s->m_entrytime){
                    indexstart = i;
                    flag_start = 1;
                }
            }
            if(flag_end == 0){
                if(poc->m_time > pb_e->m_entrytime){
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
                poc->m_count = poc->m_count+1;

//                if(poc->m_count > in_cap){
//                    int x = 0;
//                }
                assert(poc->m_count<=in_cap);
            }
            if(indexstart == 0){
                curcap = 0;
            }
            else{
                poc = occv.at(indexstart-1);
                curcap = poc->m_count;
            }
//            if(curcap+1 > in_cap){
//                int x=0;
//            }
            assert(curcap+1 <= in_cap);
            rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,0,1);

            occv.insert(occv.begin()+indexstart, newocc1);
            //inoutv.insert(inoutv.begin()+indexstart, 1);
            rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, 0 , 0,-1,newocc1,g_veh_id);
            occv.push_back(newocc2);
            //inoutv.push_back(-1);
            newocc1->m_pair = newocc2;
            newocc1->m_id = g_veh_id;
        }
        else{
            if(indexstart == indexend){
                assert(indexstart>0);
                poc = occv.at(indexstart-1);
                curcap = poc->m_count;
                assert(curcap+1 <= in_cap);
                rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,0,1);
                rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, curcap,0,-1,newocc1,g_veh_id);
                occv.insert(occv.begin()+indexstart,newocc1);
                occv.insert(occv.begin()+indexstart+1,newocc2);
                //inoutv.insert(inoutv.begin()+indexstart,1);
                //inoutv.insert(inoutv.begin()+indexstart+1,-1);
                newocc1->m_pair = newocc2;
                newocc1->m_id = g_veh_id;
            }
            else{
                assert(indexend > indexstart);
                for(uint j = indexstart; j<=indexend-1; j++){
                    poc = occv.at(j);
                    poc->m_count = poc->m_count+1;

//                    if(poc->m_count > in_cap){
//                        int x = 0;
//                    }
                    assert(poc->m_count<=in_cap);
                }
                if(indexstart == 0){
                    curcap = 0;
                }
                else{
                    poc = occv.at(indexstart-1);
                    curcap = poc->m_count;
                }
//                if(curcap+1 > in_cap){
//                    int x=0;
//                }
                assert(curcap+1 <= in_cap);
                rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,0,1);
                poc = occv.at(indexend-1);
                curcap = poc->m_count;
                rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, curcap-1,0,-1,newocc1,g_veh_id);
                occv.insert(occv.begin()+indexstart, newocc1);
                //inoutv.insert(inoutv.begin()+indexstart, 1);
                occv.insert(occv.begin()+indexend+1, newocc2);//因为插入了一个新元素，indexend指向的位置被后移了，需要加1.
                //inoutv.insert(inoutv.begin()+indexend+1, -1);
                newocc1->m_pair = newocc2;
                newocc1->m_id = g_veh_id;
            }
        }
    }
    assert(occv.size()%2 == 0);

}

void RemoveTimewindow_Inter_nowaiting(TWVector& twv, twindow* in_ptw)
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
    if((gapstart < -0.1)||(gapend < -0.1)){
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


void PathFeasibilityCheck_nowaiting()
{
    Vehicle* pveh;
    nowaiting_step* pbs;
    //CRegion* prgn;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        if(pveh->m_path.size() == 0){
            continue;
        }
        pbs = (nowaiting_step*)pveh->m_path.at(0);
        ndin = GetNodeIndex(pbs->m_bs);
        for(uint j=1; j<pveh->m_path.size();j++){
            pbs = (nowaiting_step*)pveh->m_path.at(j);
            ndout = GetNodeIndex(pbs->m_bs);
//            if(g_model.matrix_nn_conn[ndin][ndout]==0){
//                int xx = 0;
//            }
            assert(g_model.matrix_nn_conn[ndin][ndout]==1);
            ndin = ndout;
        }
    }
}

void CheckOccupationVector_nowaiting(CRegionCollect &rct)
{
    double delta = 5;

    CRegion* prgn;
    twindow* ptw;
    uint cap;
    rOccVariable* proc;
    for(uint i=0; i<rct.m_count_region; i++){
        prgn = rct.m_array_region[i];
        cap = prgn->m_num_capacity;
        if(prgn->m_type == Line){//暂时只需考虑Line类型的区域
            for(uint j=0;j<prgn->m_forward_ftwv.size();j++){
                ptw = prgn->m_forward_ftwv.at(j);
                rOccVector& occv = ptw->m_occ_vector_robust;
                //InOutVector& inoutv = ptw->GetInOutVector();
                //assert(occv.size() == inoutv.size());
                for(uint k=1; k<occv.size(); k++){
                    //首先判断容量约束是否被违反
                    proc = occv.at(k);
                    if(proc->m_count > cap){
                        qDebug() << QString("capacity vialotion: zone %1, direction %2\n\t max: %3, real: %4")
                                    .arg(prgn->m_id).arg("forward").arg(cap).arg(proc->m_count);

                        if(k+1 < occv.size()){
                            qDebug() << QString("\tdetailed info: (%1,%2) (%3,%4)")
                                        .arg(proc->m_time).arg(proc->m_count)
                                        .arg(occv.at(k+1)->m_time).arg(occv.at(k+1)->m_count);

                        }

                    }
                    //然后判断当前航空器数量的增减情况是否与in/out动作一致
                    if(proc->m_action == 1){
                        if(proc->m_count != occv.at(k-1)->m_count+1){
                            qDebug() << QString("inout error: zone %1, direction %2").arg(prgn->m_id).arg("forward");
                        }
                    }
                    else if(proc->m_action == -1){
                        if(proc->m_count != occv.at(k-1)->m_count-1){
                            qDebug() << QString("inout error: zone %1, direction %2").arg(prgn->m_id).arg("forward");
                        }
                    }
                }
                //FIFO检查
                std::vector<uint> vec_in;
                std::vector<uint> vec_out;
                for(uint i=0; i<occv.size(); i++){
                    proc = occv.at(i);
                    if(proc->m_action == 1){//in
                        vec_in.push_back(i);
                    }
                    else{//out
                        vec_out.push_back(i);
                    }

                }
                uint n = vec_in.size();//in或out的个数
                assert(n==vec_out.size());
                rOccVariable* proc_out;
                for(uint j=0;j<n;j++){
                    proc = occv.at(vec_in.at(j));
                    proc_out = proc->m_pair;
                    if(proc_out != occv.at(vec_out.at(j))){
                        qDebug() << QString("FIFO violation: zone %1, direction %2, index_in %3")
                                    .arg(prgn->m_id).arg("forward").arg(j);
                    }
                }
                //安全间隔检查
                if(occv.size()>0){
                    assert(occv.size()>1);
                    rOccVariable* proc_next;
                    //focus on "in"
                    proc = occv.at(0);//the first "in"
                    assert(proc->m_action == 1);
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_in.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+delta){
                            qDebug() << QString("Separation violation during entrance: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("forward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                    //focus on "out"
                    proc = occv.at(vec_out.at(0));
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_out.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+delta){
                            qDebug() << QString("Separation violation during exit: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("forward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                }


            }
            for(uint j=1;j<prgn->m_reverse_ftwv.size();j++){
                ptw = prgn->m_reverse_ftwv.at(j);
                rOccVector& occv = ptw->m_occ_vector_robust;
                //InOutVector& inoutv = ptw->GetInOutVector();
                //assert(occv.size() == inoutv.size());
                for(uint k=1; k<occv.size(); k++){
                    //首先判断容量约束是否被违反
                    proc = occv.at(k);
                    if(proc->m_count > cap){
                        qDebug() << QString("capacity vialotion: zone %1, direction %2\n\t max: %3, real: %4").arg(prgn->m_id).arg("reverse").arg(cap).arg(proc->m_count);
                        if(k+1 < occv.size()){
                            qDebug() << QString("\tdetailed info: (%1,%2) (%3,%4)")
                                        .arg(proc->m_time).arg(proc->m_count)
                                        .arg(occv.at(k+1)->m_time).arg(occv.at(k+1)->m_count);
                        }
                    }
                    //然后判断当前航空器数量的增减情况是否与inout动作一致
                    if(proc->m_action == 1){
                        if(proc->m_count != occv.at(k-1)->m_count+1){
                            qDebug() << QString("inout error: zone %1, direction %2")
                                        .arg(prgn->m_id).arg("reverse");
                        }
                    }
                    else if(proc->m_action == -1){
                        if(proc->m_count != occv.at(k-1)->m_count-1){
                            qDebug() << QString("inout error: zone %1, direction %2").arg(prgn->m_id).arg("reverse");
                        }
                    }
                }
                //FIFO检查
                std::vector<uint> vec_in;
                std::vector<uint> vec_out;
                for(uint i=0; i<occv.size(); i++){
                    proc = occv.at(i);
                    if(proc->m_action == 1){//in
                        vec_in.push_back(i);
                    }
                    else{//out
                        vec_out.push_back(i);
                    }

                }
                uint n = vec_in.size();//in或out的个数
                assert(n==vec_out.size());
                rOccVariable* proc_out;
                for(uint j=0;j<n;j++){
                    proc = occv.at(vec_in.at(j));
                    proc_out = proc->m_pair;
                    if(proc_out != occv.at(vec_out.at(j))){
                        qDebug() << QString("FIFO violation: zone %1, direction %2, index_in %3")
                                    .arg(prgn->m_id).arg("backward").arg(j);
                    }
                }
                //安全间隔检查
                if(occv.size()>0){
                    assert(occv.size()>1);
                    rOccVariable* proc_next;
                    //focus on "in"
                    proc = occv.at(0);//the first "in"
                    assert(proc->m_action == 1);
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_in.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+delta){
                            qDebug() << QString("Separation violation during entrance: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("backward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                    //focus on "out"
                    proc = occv.at(vec_out.at(0));
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_out.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+delta){
                            qDebug() << QString("Separation violation during exit: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("backward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                }
            }
        }
    }
    qDebug() << "rOccupationVector check finished.";
}

void ConflictDetection_nowaiting()
{
    ExtractOccupancyInfo_nowaiting();
    CheckOccupancyInfo_nowaiting(g_model.m_rct);
    SimutaneousExchangeDetection_nowaiting();//检测是否存在同时资源交换现象
    ClearOccupancyInfo_nowaiting(g_model.m_rct);//检测完后清空占用信息，以允许再次检测
    qDebug() << "conflict detection finished.";
}

void ExtractOccupancyInfo_nowaiting()
{
    double t1;
    double t2;
    uint zone1;
    uint zone2;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    Vehicle* pveh;
    nowaiting_step* pbs;
    CRegion* prgn;
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        if(pveh->m_path.size() == 0){
            continue;
        }
        //------------------------
        pbs = (nowaiting_step*)pveh->m_path.at(0);
        t1 = pbs->m_entrytime;//
        zone1 = GetRegionIndex(pbs->m_bs);
        ndin = GetNodeIndex(pbs->m_bs);
        for(uint j=1; j<pveh->m_path.size();j++){

            pbs = (nowaiting_step*)pveh->m_path.at(j);
            //t2 = ioaEnd(pbs->m_IOA, pbs->m_entrytime)+g_len_plane/g_speed;//最晚（机身）完全脱离的时间(由于存在进入和脱离动作发生时间比较接近的情况，可能导致NTOA和最坏脱离/进入时刻的顺序不一致，程序可能就会判定出现容量冲突。实际上这种情况可以暂时忽略，因为容量设定时本身就留有一定的裕度)
            t2 = pbs->m_entrytime;//
            zone2 = GetRegionIndex(pbs->m_bs);
            ndout = GetNodeIndex(pbs->m_bs);
            occ_info occinfo;
            occinfo.m_id = pveh->m_id;
            occinfo.m_time_start = t1;
            occinfo.m_time_end = t2;
            occinfo.m_nd_in = ndin;
            occinfo.m_nd_out = ndout;
            prgn = g_model.m_rct.GetRegion(zone1);
            if(prgn->m_type == Line){
                occinfo.m_direction = (ndin < ndout) ? 0 : 1;//占用方向
            }
            prgn->m_occinfo_vector.push_back(occinfo);
            zone1 = zone2;
            t1 = pbs->m_entrytime;//最早可能到达的时间
            ndin = ndout;
//            if(fabs(t1-76396.5)<1e-1){
//                int x=0;
//            }
        }
    }
}

void CheckOccupancyInfo_nowaiting(CRegionCollect& rct)
{
    CheckOccupancyInfo(rct);
}

void SimutaneousExchangeDetection_nowaiting()
{
    SimutaneousExchangeDetection();
}

void ClearOccupancyInfo_nowaiting(CRegionCollect& rct)
{
    ClearOccupancyInfo(rct);
}

void HoldDetection_nowaiting()
{
    double ratio = 0.1;

    Vehicle* pveh;
    nowaiting_step* pbs;
    //CRegion* prgn;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    double tin;
    double tout;
    double length;
    double speed;
    uint rgnid;
    int isTurn;
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        if(pveh->m_path.size() == 0){
            continue;
        }
        pbs = (nowaiting_step*)pveh->m_path.at(0);
        tin = pbs->m_entrytime;
        ndin = GetNodeIndex(pbs->m_bs);
        rgnid = GetRegionIndex(pbs->m_bs);

        for(uint j=1; j<pveh->m_path.size();j++){
            pbs = (nowaiting_step*)pveh->m_path.at(j);
            tout = pbs->m_entrytime;
            ndout = GetNodeIndex(pbs->m_bs);
            length = g_model.matrix_nn_dis[ndin][ndout];//length to the other node of the lane
            uint nextinterid = g_model.m_ndct.m_array_node[ndout]->GetAnotherZoneId(rgnid);//another neighbor intersection's id of the lane( assumes that a lane connects with an intersection at either end)
            uint act = getAction_nowaiting(rgnid,ndin,nextinterid);
            isTurn = g_model.matrix_nn_isTurn[ndin][ndout];
            speed = getSpeed_nowaiting(rgnid,act,isTurn);
//            if(g_model.matrix_nn_conn[ndin][ndout]==0){
//                int xx = 0;
//            }
            if(g_model.matrix_nn_hold[ndin][ndout]==0){
                //assert(tout-tin < length/speed*(1+ratio)+0.1);
                if(tout-tin >= length/speed*(1+ratio)+0.1){
                    int xx = 0;
                }
            }
            ndin = ndout;
            tin = tout;
            rgnid = GetRegionIndex(pbs->m_bs);
        }
    }
}

double getSpeed_nowaiting(uint rgnid, uint action, const int in_isTurn)//这里没有考虑弯道速度呀
{
    CRegion* prgn = g_model.m_rct.GetRegion(rgnid);
    if(prgn->m_type == Runway ){
        if(action == TAXI){
            if(in_isTurn!=0)
                return g_speed_turn;
            else
                return g_speed;
        }
        else{//对于LAND和TAKEOFF暂时同样处理
            assert(action == LAND || action==TAKEOFF);
            return g_runway_speed;
        }
    }
    else if(prgn->m_type == Inter || prgn->m_type == Line){
        assert(action == TAXI);
        if(in_isTurn!=0)
            return g_speed_turn;
        else
            return g_speed;
    }
    else{
        qDebug() << "Undefined speed scenario.";
        return g_speed;
    }
}

uint getAction_nowaiting(uint srgnid, uint sndid, uint ergnid)
{
    CRegion* prgn = g_model.m_rct.GetRegion(srgnid);
    if(prgn->m_type == Inter || prgn->m_type == Line){
        return TAXI;
    }
    else if(prgn->m_type == Runway){
        CRegion* prgn_next = g_model.m_rct.GetRegion(ergnid);
        if(prgn_next->m_type == Buffer){
            return TAKEOFF;
        }
        cnode* pnd = g_model.m_ndct.m_array_node[sndid];
        uint rgn_pre = pnd->GetAnotherZoneId(srgnid);
        CRegion* prgn_pre = g_model.m_rct.GetRegion(rgn_pre);
        if(prgn_pre->m_type == Buffer){
            return LAND;
        }
        return TAXI;
    }
    else{
        return TAXI;
    }
}

void printPathDetail_nowaiting(Path& rpath, QString fdir)
{
    QFile file(QString("%1\\PathDetailofVehicle%2.txt").arg(fdir).arg(g_veh_id));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the path detail file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        if(rpath.empty()){
            qDebug() << QString("Empty path for vehicle %1!").arg(g_veh_id);
            file.close();
            return;
        }
        nowaiting_step* rs = (nowaiting_step*)rpath.at(0);
        //time | node | region | delay | holdable? |[entw.tstart,entw.tend]
        QString strline = QString("%1 %2 %3 %4 %5 %6 %7").arg(rs->m_entrytime,0,'f',3)
                .arg(GetNodeIndex(rs->m_bs)).arg(GetRegionIndex(rs->m_bs)).arg(rs->m_entrytime - g_pveh->m_start_time)
                .arg(1).arg(rs->m_entw.tstart,0,'f',3).arg(rs->m_entw.tend,0,'f',3);//第一行输出的等待时间是delay_start
        ts << strline << '\n';
        nowaiting_step* rs_next;
        for(uint i=1;i<rpath.size();i++){
            rs_next = (nowaiting_step*)rpath.at(i);
            double delay = rs_next->m_entrytime - rs->m_entrytime - rs_next->m_length/rs_next->m_speed;
            uint holdable = g_model.matrix_nn_hold_copy[GetNodeIndex(rs->m_bs)][GetNodeIndex(rs_next->m_bs)];
            //time | node | region| holdable? | [entw.tstart,entw.tend]
            strline = QString("%1 %2 %3 %4 %5 %6 %7").arg(rs_next->m_entrytime,0,'f',3)
                    .arg(GetNodeIndex(rs_next->m_bs)).arg(GetRegionIndex(rs_next->m_bs)).arg(delay,0,'f',3)
                    .arg(holdable).arg(rs_next->m_entw.tstart,0,'f',3).arg(rs_next->m_entw.tend,0,'f',3);
            ts << strline << '\n';

            rs = rs_next;
        }
        file.close();
    }
}

void genTBST_nowaiting(Vehicle* pveh)
{
    Path& path = pveh->m_path;
    if(path.empty()){
        return;
    }
    assert(path.size()>1);
    nowaiting_step* pbs = (nowaiting_step*)path.at(path.size()-1);
    pbs->m_entrytime = pbs->m_entw.tstart;
    uint j;
    nowaiting_step* pbs_pre;
    for(uint i=1; i<path.size(); i++){
        j = path.size()-1-i;
        pbs_pre = (nowaiting_step*)path[j];
        double time = pbs->m_entrytime - pbs->m_length/pbs->m_speed;
        if(time>pbs_pre->m_entw.tend+1e-3){
            pbs_pre->m_entrytime = pbs_pre->m_entw.tend;
        }
        else{
            assert(time>=pbs_pre->m_entw.tstart-1e-3);
            pbs_pre->m_entrytime = time;
        }
        pbs = pbs_pre;
    }
}
