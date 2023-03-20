#include "TP_robust.h"
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
#include <math.h>
#include "TP_robust_globalvariables.h"


//double g_k = 0;
//double g_ioa = 0;//IOA的默认长度为10s
//double g_len_plane = 0;//飞机机身长度（m）
//double g_sep = 0/g_speed;//最小安全间隔（单位：s）（机头——机头距离），取为两倍平均机身长度/速度


void Experiment_Robust()
{
    printParam(QString("..\\data_out\\singlepath\\参数设置.txt"));//将当前参数设置写入文件
    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件
    SequencePlan_robust();
    printResult(QString("..\\data_out\\singlepath\\singlepath_den%1.txt")
                .arg(g_frequency));
    releaseMemory();//释放内存占用，避免内存泄漏
}

void releaseMemory()
{
    //TO be implemented
}

int SequencePlan_robust()
{
    int returnvalue;
    QString fdir = QString("..\\data_out\\singlepath");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        //rPath robustPath;//TODO:要在头文件中定义这个数据类型
        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
//        if(g_veh_id == 6){
//            int x = 0;
//        }
        if(g_priority%50 == 0){//每若干次循环清理一次时间窗
            TimeWindowSlimming(veh->m_start_time);
        }
        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_robust(veh->m_rPath, veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue == 0){
            adjustArrivalRunwayOccupancy(veh);
            ctvector.push_back(t2-t1);
            UpdateTimeWindowVector_robust(veh->m_rPath,g_model.m_rct);
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);
            //if(g_veh_id == 54){
                printPathDetail(veh->m_rPath,fdir);
                //SavePathToFile_robust(veh->m_id, veh->m_rPath,QString("..\\data_out\\RobustDRT_path_of_vehicle%1.txt").arg(veh->m_id));
            //}
//            if(g_veh_id == 9){
//                QString file = QString("..\\data_out\\Robust_twindowaftertarget_%1.txt").arg(veh->m_id);
//                PrintTimeWindows(file, g_model.m_rct);//for debug
//            }
        }
        else{
            ctvector.push_back(-1);
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }
        itr_cnt_vector.push_back(g_iterationcount);
        ResetGlobalVariables();//当前航空器规划完毕后复位必要的全局变量：
        g_priority++;//denote agent's planning priority
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){//线程休眠一段时间以降低CPU占用率
            QCoreApplication::processEvents();
        }
    }
    //全部规划完毕后：
    PathFeasibilityCheck();//检测路径的物理可行性
    CheckOccupationVector_robust(g_model.m_rct);//检查路段的占用向量中是否存在错误（路段容量、FIFO冲突）
    ConflictDetection_robust();//检查是否存在（交叉口和路段对头）冲突
    ResetTimeWindows(g_model.m_rct);//复位时间窗约束
    delete[] g_array_h;//释放启发值数组
    qDebug() << QString("------------Planning completed-----------");
    return 0;
}

int Plan_robust(rPath &out_path, Vehicle *veh)//TODO:如何考虑stand holding问题？
{
    double exp_time = 0;
    double t1, t2;
//    if(veh->m_id == 10){
//        int xx = 0;
//    }
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
    double ioa = g_ioa;//length of the initial IOA
    //double k = 0.1;//按ioa/travel time=0.1
    uint rgnid;
    uint ndid;
    uint awid;
    ndid = g_start_node;
    rgnid = first_taxiway_region;
    twindow ew(ioaStart(ioa,g_start_time), c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the next zone to be visited, i.e., the first taxiway zone
    /*tackle the first taxiway zone*/
    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
        WindowTimeTable wtt;
        uint num;
        if(!Overlapped_robust(&ew,ioa, prgn->GetTWVector(),num,wtt)){//
            QString info = QString("error, the entertime of target %1 is not properly set").arg(veh->m_id);
            qDebug(info.toAscii());
            return 1;
        }
        else{
            WindowTimeTable::iterator itr;
            for(itr=wtt.begin();itr!=wtt.end();itr++){
                awid = itr->first;
                tenter = itr->second;//i.e., NTOA
                bs = TernaryToUnary(rgnid, ndid, awid);
                double cost;
                if(veh->m_type == DEPAR){
                    if(g_flag_startdelay){
                        cost = calCost(0,g_speed,tenter - veh->m_start_time);
                    }
                    else{
                        cost = 0;//直接取0合适吗？
                    }

                }
                else if(veh->m_type == ARRIV){
                    if(g_flag_startdelay){
                        cost = calCost(0,g_speed,tenter - veh->m_start_time);//0表示这里假定从起始位置到第一个控制的距离为0，更实际的方式可以考虑将其作为规划输入参数
                    }
                    else{
                        cost = 0;
                    }
                }
                else{
                    cost = 0;
                    qDebug() << QString("Unkown vehicle type. Please check.");
                }

                robust_step* b_s = new robust_step(bs, tenter, ioa,cost);//TODO(Ready):增加robust_step类三参数的构造函数
                openlistexp.push_back(b_s);
                robust_step* b_s_pre = new robust_step(TernaryToUnary(g_start_region,0,0),0,0,0);
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
    robust_step* pb_s;
    //-----------------------//
    const uint ordernum = 2;//
    uint ordercount = 0;
    //-----------------------//
    while(!openlistexp.empty()){
        g_iterationcount++;//迭代次数增加1

        uint n = openlistexp.size();


        GetMinHeuristicCostOfOpenlist_robust(pb_s);//get least cost bs from open

        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
//        if(g_veh_id == 54 && curregion == 77 && curnode == 192){
//            int xx = 0;
//        }
//        if(g_veh_id == 54 && curregion == 79 && curnode == 181){
//            int xx = 0;
//        }

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found
            ordercount++;

            rPath path;
            path.push_back(pb_s);
            do{
                pb_s = (robust_step*)pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                out_path.push_back(path.at(n - 1 - i));//调正顺序
            }
            status =  0;//denote path found

            //g_expandtime_vector.push_back(exp_time);//将扩展过程所用的时间保存
            break;//退出while

        }
        else{
            //t1 = GetHighPrecisionCurrentTime();
            Expand_robust(pb_s);//preregion is used to prevent doubling back
            //t2 = GetHighPrecisionCurrentTime();
            //exp_time += t2 - t1;
        }

    }

    return status;
}

void Expand_robust(robust_step* in_bs)
{
    double k = g_k;//计算IOA时会用到
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
//    if(currid == 446&&curnodeid==311 && g_veh_id == 6){
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
//    if(prgn->m_type == Inter){
//        avg_speed = g_speed;
//    }
//    else{
//        //avg_speed = g_speed * 5;//假定跑道上滑行的速度是普通滑行速度的5倍(TODO:滑行的航空器穿越跑道时，速度也要5倍吗？)
//        avg_speed = g_speed;
//    }
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
    double ioa_first;
    double ioa_second;
    uint act;//
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
            if(g_model.matrix_nn_conn[curnodeid][nd_id] == 0 ){//若从curnodeid到nd_id不可行（无滑行路线或该方向的滑行被禁止），则退出本次循环
                continue;
            }
            nnlength = g_model.matrix_nn_dis[curnodeid][nd_id];//length between current node and the neighbor node
            act = getAction(currid,curnodeid,neighbor_zone_id);
            avg_speed = getSpeed(currid,act);
            //ioa_first = k*nnlength/avg_speed;
            ioa_first = getIOA(in_bs->m_IOA,nnlength,avg_speed,k);


            uint paircount = 0;
            WindowTimeTable wtt;
            //这里的扩展基准只可能是Inter或Runway型，所以:
            if(ExitWindow_Intersection_robust(in_bs->m_entrytime,ioa_first,nnlength, paw, pew, avg_speed)){
                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){
//                    if(g_veh_id == 6 && neighbor_zone_id == 78){
//                        int xx = 0;
//                    }
                    uint awid_line;
                    dir_nb = pnb->GetDirection(nd_id);
                    if(!EnterTime_Line_robust(pnb->GetTWVector(dir_nb), pew,ioa_first, pnb->GetCapacity(),avg_speed, tenter, awid_line)){
                        continue;//if the neighbor lane is not accessible, continue
                    }
                    else{//若该路段可以进入，需要进一步确定是否可脱离

                        double cost1 = calCost(nnlength,avg_speed,tenter - in_bs->m_entrytime);//从交叉口入口运行到相邻路段入口所花费的代价

                        pnbaw = pnb->GetTWVector(dir_nb).at(awid_line);
                        assert(pnb->m_count_node == 2);
                        uint next_node_id;
                        if(pnb->m_vector_node.at(0) == nd_id){
                            next_node_id = pnb->m_vector_node.at(1);
                        }
                        else{
                            next_node_id = pnb->m_vector_node.at(0);
                        }
                        //注意，下面nnlength被重新赋值，表示从路段入口到下一个交叉口入口的距离
                        nnlength = g_model.matrix_nn_dis[nd_id][next_node_id];//length to the other node of the lane
                        uint nextinterid = g_model.m_ndct.m_array_node[next_node_id]->GetAnotherZoneId(neighbor_zone_id);
                        act = getAction(neighbor_zone_id,nd_id,nextinterid);
                        avg_speed = getSpeed(neighbor_zone_id,act);
                        //ioa_second = k*nnlength/avg_speed;
                        ioa_second = getIOA(ioa_first,nnlength,avg_speed,k);


                        if(!ExitWindow_Line_robust(tenter,ioa_second, pnbaw, pnb->GetCapacity(), nnlength, pew, avg_speed)){
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
                                if(!Overlapped_robust(pew,ioa_second, pnb_next->GetTWVector(), paircount, wtt)){
                                    continue;
                                }
                                else{
                                    robust_step* b_s_tmp;
                                    robust_step* b_s_lane;
                                    Basicstep bs;
                                    uint awid;
                                    volatile double entertime;
                                    WindowTimeTable::iterator itr;
                                    for(itr=wtt.begin();itr!=wtt.end();itr++){
                                        awid = itr->first;
                                        entertime = itr->second;
                                        double cost2 = calCost(nnlength,avg_speed,entertime-tenter);
                                        TernaryToUnary(nextinterid, next_node_id, awid, bs);
                                        if(Closed(bs)){
                                            continue;
                                        }
                                        else if(Opened_robust(bs, b_s_tmp)){//判断是否open，并确定相应的base_step指针
                                            if(b_s_tmp->m_cost > (in_bs->m_cost+cost1+cost2)){
                                                //for the lane
                                                b_s_lane = new robust_step(curbs, tenter, ioa_first, in_bs->m_cost+cost1);
                                                g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存
                                                b_s_lane->m_prestep = in_bs;
                                                //for the intersection
                                                b_s_tmp->m_entrytime = entertime;
                                                b_s_tmp->m_prestep = b_s_lane;
                                                b_s_tmp->m_IOA = ioa_second;
                                                b_s_tmp->m_cost = b_s_lane->m_cost+cost2;

                                            }

                                        }
                                        else{
                                            b_s_tmp = new robust_step(bs, entertime,ioa_second,in_bs->m_cost+cost1+cost2);
                                            openlistexp.push_back(b_s_tmp);//add bs to open
                                            b_s_lane = new robust_step(curbs, tenter,ioa_first,in_bs->m_cost+cost1);
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
                    if(!Overlapped_robust(pew,ioa_first, pnb->GetTWVector(), paircount, wtt)){
                        continue;
                    }
                    else{
                        robust_step* b_s_tmp;
                        Basicstep bs;
                        uint awid;
                        double entertime;
                        WindowTimeTable::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            awid = itr->first;
                            entertime = itr->second;
                            double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                            TernaryToUnary(neighbor_zone_id, nd_id, awid, bs);
                            if(Closed(bs)){
                                continue;
                            }
                            else if(Opened_robust(bs, b_s_tmp)){
                                if(fabs(b_s_tmp->m_entrytime - 3867.469)<1e-3){
                                    int xxx = 0;
                                }
                                if(b_s_tmp->m_cost > in_bs->m_cost+costtmp){
                                    b_s_tmp->m_entrytime = entertime;
                                    b_s_tmp->m_prestep = in_bs;
                                    b_s_tmp->m_IOA = ioa_first;
                                    b_s_tmp->m_cost = in_bs->m_cost+costtmp;
                                }

                            }
                            else{
                                b_s_tmp = new robust_step(bs, entertime,ioa_first,in_bs->m_cost+costtmp);
                                b_s_tmp->m_prestep = in_bs;
                                openlistexp.push_back(b_s_tmp);
                            }
                        }
                    }
                }
                /*else if current neighbor is a stand or a buffer*/
                else{//stand和air-buffer内的冲突情况暂不考虑，所以：
                    robust_step* b_s_tmp;
                    Basicstep bs;
                    double entertime = ntoa(ioa_first,pew->tstart);
                    double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                    TernaryToUnary(neighbor_zone_id, nd_id, 0, bs);
                    if(Closed(bs)){
                        continue;
                    }
                    else if(Opened_robust(bs, b_s_tmp)){
                        if(b_s_tmp->m_cost > in_bs->m_cost+costtmp){
                            b_s_tmp->m_entrytime = entertime;
                            b_s_tmp->m_prestep = in_bs;
                            b_s_tmp->m_IOA = ioa_first;
                            b_s_tmp->m_cost = in_bs->m_cost+costtmp;
                        }

                    }
                    else{
                        b_s_tmp = new robust_step(bs, entertime,ioa_first,in_bs->m_cost+costtmp);
                        b_s_tmp->m_prestep = in_bs;
                        openlistexp.push_back(b_s_tmp);
                    }
                }
            }
        }
    }
    delete pew;
}

bool Opened_robust(Basicstep in_bs, robust_step*& out_b_s)
{
    robust_step* pb_s;
    for(uint i=0; i<openlistexp.size();i++){
        pb_s = (robust_step*)openlistexp.at(i);
        if(pb_s->m_bs == in_bs){
            out_b_s = pb_s;
            return true;
        }
    }
    return false;
}

void adjustArrivalRunwayOccupancy(Vehicle* pveh)
{
    if(pveh->m_type == DEPAR){
        return;
    }
    if(pveh->m_rPath.empty()){
        qDebug() << QString("S: No path found for vehicle %1").arg(pveh->m_id);
        return;
    }
    robust_step* prs;
    robust_step* prs_next;
    assert(pveh->m_rPath.size()>1);
    prs = pveh->m_rPath.at(0);//第一个step
    prs_next = pveh->m_rPath.at(1);//第二个step
    double unimp = g_model.matrix_nn_dis[GetNodeIndex(prs->m_bs)][GetNodeIndex(prs_next->m_bs)]/g_runway_speed;
    double delay = prs_next->m_entrytime - prs->m_entrytime - unimp;
    if(delay > 1e-3){//说明在进入快速脱离道之前进行了等待
        prs->m_entrytime += delay;
        prs->m_cost += delay;
    }

}

void GetMinHeuristicCostOfOpenlist_robust(robust_step *&out_bs)
{
    out_bs = (robust_step*)openlistexp.at(0);
    double cost = out_bs->m_cost + g_w*g_array_h[GetNodeIndex(out_bs->m_bs)]/g_runway_speed;//TODO:确定一下这样计算的启发值是不是admissible的
    double tmptime;
    double tmpcost;
    uint index = 0;
    robust_step* bs;
    uint ndid;
    for(uint i=0; i < openlistexp.size();i++){
        bs = (robust_step*)openlistexp.at(i);
        ndid = GetNodeIndex(bs->m_bs);
        tmptime = bs->m_cost;
        tmpcost = tmptime + g_w*g_array_h[ndid]/g_runway_speed;
        if(cost > tmpcost){
            out_bs = (robust_step*)openlistexp.at(i);
            cost = out_bs->m_cost + g_w*g_array_h[GetNodeIndex(out_bs->m_bs)]/g_runway_speed;
            index = i;
        }
    }
    //将它从openlist中取出,并加入closelist
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}

double calCost(double len, double speed, double duration)
{
    return g_w*len/speed + g_w_delay*(duration - len/speed);
}


void printParam(QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the record file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        QString strline;

        strline = QString("g_frequency = %1;").arg(g_frequency);
        ts << strline << '\n';

        if(g_config.flag_heuristic == sd){
            strline = QString("g_config.flag_heuristic = sd;");
        }
        else if(g_config.flag_heuristic == euclidean){
            strline = QString("g_config.flag_heuristic = euclidean;");
        }
        else if(g_config.flag_heuristic == zero){
            strline = QString("g_config.flag_heuristic = zero;");
        }
        else{
            qDebug()<<QString("Unkown heuristic flag.");
            strline = QString("g_config.flag_heuristic = unknown;");
        }
        ts << strline << '\n';

        if(g_flag_startdelay){
            strline = QString("g_flag_startdelay = true;");
        }
        else{
            strline = QString("g_flag_startdelay = false;");
        }
        ts << strline << '\n';
        strline = QString("g_ratio_range = %1;").arg(g_ratio_range);
        ts << strline << '\n';
        strline = QString("g_pathnumber = %1;").arg(g_pathnumber);
        ts << strline << '\n';
        strline = QString("g_k = %1;").arg(g_k);
        ts << strline;//最后一行不必换行
        file.close();
    }
}

void printPathDetail(rPath& rpath, QString fdir)
{
    Vehicle* pveh = g_vehs.at(g_veh_id-1);//注意g_veh_id是从1开始计数的，所以这里要-1
    QFile file(QString("%1\\PathDetailofVehicle%2.txt").arg(fdir).arg(g_veh_id));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the path detail file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        if(rpath.empty()){
            qDebug() << QString("Empty path for vehicle %1!").arg(g_veh_id);
            return;
        }
        robust_step* rs = rpath.at(0);
        //NTOA | node | region | IOA | delay
        QString strline = QString("%1 %2 %3 %4 %5").arg(rs->m_entrytime,0,'f',3)
                .arg(GetNodeIndex(rs->m_bs)).arg(GetRegionIndex(rs->m_bs))
                .arg(rs->m_IOA,0,'f',1).arg(rs->m_entrytime - pveh->m_start_time,0,'f',1);//第一行输出的等待时间是delay_start
        ts << strline << '\n';
        robust_step* rs_next;
        double delay;
        double len;
        uint nd = GetNodeIndex(rs->m_bs);
        uint nd_next;
        uint act;
        for(uint i=1;i<rpath.size();i++){
            if(i==1 && pveh->m_type == ARRIV){
                act = LAND;
            }
            else if(i==rpath.size()-1 && pveh->m_type == DEPAR){
                act = TAKEOFF;
            }
            else{
                act = TAXI;
            }
            rs_next = rpath.at(i);
            nd_next = GetNodeIndex(rs_next->m_bs);
            len = g_model.matrix_nn_dis[nd][nd_next];
            delay = rs_next->m_entrytime - rs->m_entrytime -
                    len/getSpeed(GetRegionIndex(rs->m_bs),act);
            //NTOA | node | region | IOA | delay
            strline = QString("%1 %2 %3 %4 %5").arg(rs_next->m_entrytime,0,'f',3)
                    .arg(GetNodeIndex(rs_next->m_bs)).arg(GetRegionIndex(rs_next->m_bs))
                    .arg(rs_next->m_IOA,0,'f',1).arg(delay,0,'f',1);
            ts << strline << '\n';

            rs = rs_next;
            nd = nd_next;
        }
        file.close();
    }
}

double getSpeed(uint rgnid, uint action)
{
    CRegion* prgn = g_model.m_rct.GetRegion(rgnid);
    if(prgn->m_type == Runway ){
        if(action == TAXI){
            return g_speed;
        }
        else{//对于LAND和TAKEOFF暂时同样处理
            assert(action == LAND || action==TAKEOFF);
            return g_runway_speed;
        }
    }
    else if(prgn->m_type == Inter || prgn->m_type == Line){
        assert(action == TAXI);
        return g_speed;
    }
    else{
        qDebug() << "Undefined speed scenario.";
        return g_speed;
    }
}

void printResult(QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the record file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        Vehicle* pveh;
        for(uint i=0;i<g_vehs.size();i++){
            pveh = g_vehs.at(i);
            uint id = pveh->m_id;
            if(pveh->m_rPath.empty()){
                continue;
            }
            //taxi time
            robust_step* rss = pveh->m_rPath.at(0);
            robust_step* rse = pveh->m_rPath.at(pveh->m_rPath.size()-1);
            double taxitime = rse->m_entrytime - rss->m_entrytime;
            //taxi distance
            double taxidis = 0;
            robust_step* prs_cur;
            robust_step* prs_next;
            uint nd_cur;
            uint nd_next;
            prs_cur = rss;
            for(uint j=1;j<pveh->m_rPath.size();j++){
                prs_next = pveh->m_rPath.at(j);
                nd_cur = GetNodeIndex(prs_cur->m_bs);
                nd_next = GetNodeIndex(prs_next->m_bs);
                taxidis = taxidis+g_model.matrix_nn_dis[nd_cur][nd_next];
                prs_cur = prs_next;
            }
            QString str;
            if(pveh->m_type == DEPAR){
                str = QString("%1").arg(-1);//打印-1表示航空器离港
            }
            else if(pveh->m_type== ARRIV){
                str = QString("%1").arg(1);//打印1表示航空器进港
            }
            else{
                str = QString("%1").arg(0);//打印0表示出现未知类型航空器（实验中本不应当出现这种情况）
            }
            /*单路径时不用分析delay*/
            //delay
            //double delay = taxitime - taxidis/g_speed;
            //经过仔细考虑，这里暂时在结果分析时不计入在起始位置的等待
//            if(g_flag_startdelay){
//                delay = delay + rss->m_entrytime - pveh->m_start_time;//
//            }
//            if(pveh->m_type == ARRIV){
//                delay = delay + rss->m_entrytime - pveh->m_start_time;
//            }
//            if(fabs(delay)<1e-3){
//                delay = 0;
//            }
            //save result to file

//            ts << QString("%1\t%2\t%3\t%4\t%5\n").arg(id).arg(taxitime,0,'f',1)
//                  .arg(taxidis,0,'f',1).arg(delay,0,'f',1).arg(str);

            // 标识 | 进离港类型 | taxi time | taxi distance | computation time
            ts << QString("%1\t%2\t%3\t%4\t%5\t%6\n").arg(id).arg(str).arg(taxitime,0,'f',1)
                  .arg(taxidis,0,'f',1).arg(ctvector.at(i),0,'f',2).arg(itr_cnt_vector.at(i));

            //ts << QString("%1\t%2\n").arg(id).arg(taxitime,0,'f',1);
        }
        file.close();
    }
}




void CheckOccupationVector_robust(CRegionCollect &rct)
{
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
                        if(proc_next->m_time < proc->m_time+g_sep){
                            qDebug() << QString("Separation violation during entrance: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("forward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                    //focus on "out"
                    proc = occv.at(vec_out.at(0));
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_out.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+g_sep){
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
                        if(proc_next->m_time < proc->m_time+g_sep){
                            qDebug() << QString("Separation violation during entrance: zone %1, direction %2, index_in %3")
                                        .arg(prgn->m_id).arg("backward").arg(k);
                        }
                        proc = proc_next;//progress
                    }
                    //focus on "out"
                    proc = occv.at(vec_out.at(0));
                    for(uint k=1; k<n; k++){
                        proc_next = occv.at(vec_out.at(k));//the next "in"
                        if(proc_next->m_time < proc->m_time+g_sep){
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


void PathFeasibilityCheck()
{
    Vehicle* pveh;
    robust_step* pbs;
    //CRegion* prgn;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        if(pveh->m_rPath.size() == 0){
            continue;
        }
        pbs = pveh->m_rPath.at(0);
        ndin = GetNodeIndex(pbs->m_bs);
        for(uint j=1; j<pveh->m_rPath.size();j++){
            pbs = pveh->m_rPath.at(j);
            ndout = GetNodeIndex(pbs->m_bs);
//            if(g_model.matrix_nn_conn[ndin][ndout]==0){
//                int xx = 0;
//            }
            assert(g_model.matrix_nn_conn[ndin][ndout]==1);
            ndin = ndout;
        }
    }
}

void ConflictDetection_robust()
{
    ExtractOccupancyInfo_robust();
    CheckOccupancyInfo_robust(g_model.m_rct);
    SimutaneousExchangeDetection_robust();//检测是否存在同时资源交换现象
    ClearOccupancyInfo_robust(g_model.m_rct);//检测完后清空占用信息，以允许再次检测
    qDebug() << "conflict detection finished.";
}

void SimutaneousExchangeDetection_robust()
{
    SimutaneousExchangeDetection();
}

void ExtractOccupancyInfo_robust()
{
    double t1;
    double t2;
    uint zone1;
    uint zone2;
    uint ndin;//进入区域的节点
    uint ndout;//离开区域的节点
    Vehicle* pveh;
    robust_step* pbs;
    CRegion* prgn;
    for(uint i=0; i<g_vehs.size(); i++){
        pveh = g_vehs.at(i);
        if(pveh->m_rPath.size() == 0){
            continue;
        }
        //------------------------
        pbs = pveh->m_rPath.at(0);
        t1 = ioaStart(pbs->m_IOA, pbs->m_entrytime);//最早可能到达的时间
        zone1 = GetRegionIndex(pbs->m_bs);
        ndin = GetNodeIndex(pbs->m_bs);
        for(uint j=1; j<pveh->m_rPath.size();j++){

            pbs = pveh->m_rPath.at(j);
            //t2 = ioaEnd(pbs->m_IOA, pbs->m_entrytime)+g_len_plane/g_speed;//最晚（机身）完全脱离的时间(由于存在进入和脱离动作发生时间比较接近的情况，可能导致NTOA和最坏脱离/进入时刻的顺序不一致，程序可能就会判定出现容量冲突。实际上这种情况可以暂时忽略，因为容量设定时本身就留有一定的裕度)
            t2 = ioaEnd(pbs->m_IOA, pbs->m_entrytime);//
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
            t1 = ioaStart(pbs->m_IOA, pbs->m_entrytime);//最早可能到达的时间
            ndin = ndout;
//            if(fabs(t1-76396.5)<1e-1){
//                int x=0;
//            }
        }
    }
}


void CheckOccupancyInfo_robust(CRegionCollect& rct)
{
    CheckOccupancyInfo(rct);
}

void ClearOccupancyInfo_robust(CRegionCollect& rct)
{
    ClearOccupancyInfo(rct);
}






bool Overlapped_robust(twindow* in_ptw, double in_ioa, TWVector& in_twv, uint& out_count, WindowTimeTable& out_wtt)
{
    //这里不仅要判断是否存在overlap，还要根据IOA进一步排除无意义的overlap，最后只保留足够大的overlap信息
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
            if(EnterTime_Intersection_robust(in_ptw, in_ioa, ptwtmp, tmin)){
                out_count++;
                out_wtt.insert(WindowTimeTable::value_type(i, tmin));
            }
        }
        return (out_count > 0);
    }
}

bool EnterTime_Intersection_robust(twindow* in_ptw1, double in_ioa, twindow* in_ptw2, double& out_tmin)
{
    double len = in_ptw2->tend - in_ptw2->tstart;
    double ioa;
    //为了防止同时资源交换
    if(in_ioa < 1e-3){
        ioa = 1e-3;
    }
    else{
        ioa = in_ioa;
    }
    if(len < in_ioa){
        return false;
    }
    else{
        if((in_ptw1->tstart >= in_ptw2->tstart) && (in_ptw2->tend - in_ptw1->tstart >= ioa)){
            out_tmin = ntoa(in_ioa,in_ptw1->tstart);
            return true;
        }
        else if((in_ptw2->tstart > in_ptw1->tstart)&&(in_ptw1->tend - in_ptw2->tstart >= ioa )){
            out_tmin = ntoa(in_ioa,in_ptw2->tstart);
            return true;
        }
        else return false;
    }
}


double ioaStart(double ioa, double ntoa)
{
    return ntoa-ioa/2;
}

double ntoa(double ioa, double ioa_start)
{
    return ioa/2 + ioa_start;
}

bool ExitWindow_Intersection_robust(double in_time, double in_ioa, double in_nnlength,
                                    twindow* in_paw, twindow* out_pew, double in_speed)
{
    double tend = in_time + in_nnlength/in_speed;//i.e., \tau_{e}
    double tsTe = ioaStart(in_ioa,tend);
    double delta = g_len_plane/in_speed;//航空器机身完全脱离当前区域需要的时间
    if(in_paw->tend < (in_ioa+delta+tsTe)){//没有足够长的时间窗包容运行时间的不确定性
        return false;
    }
    else{
        if(in_time < in_paw->tstart){
            qDebug("error, the enter time is not among the aw");
            return false;
        }
        out_pew->tstart = tsTe;
        out_pew->tend = in_paw->tend - delta;
        return true;

    }
}

bool ExitWindow_Line_robust(const double in_tenter,double in_ioa, twindow* in_aw,
                            uchar in_cap, double in_nnlength, twindow* out_pew, double in_speed)
{
    assert(in_tenter >= in_aw->tstart);
    double delta = g_len_plane/in_speed;//for 机身
    double sep = g_sep;//安全间隔

    /*if the occvector is empty, it's straightforward*/
    if(in_aw->m_occ_vector_robust.size() == 0){
        out_pew->tstart = ioaStart(in_ioa,in_nnlength/in_speed + in_tenter);
        out_pew->tend = in_aw->tend - delta;
        if(out_pew->tend - out_pew->tstart >= in_ioa){
            return true;
        }
        else{
            return false;
        }
    }
    assert(in_aw->m_occ_vector_robust.size() >= 2);
    /*else, */
    rOccVariable* pocc;
    uchar precount = 0;
    uint out = 0;
    uint i;
    char status = 1;
    for(i=0;i<in_aw->m_occ_vector_robust.size();i++){
        pocc = in_aw->m_occ_vector_robust.at(i);
        if(pocc->m_time > in_tenter){
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
        tl = ioaStart(in_ioa, in_nnlength/in_speed + in_tenter);
        rOccVariable* ptmp;
        double time_tmp;
        for(;i<in_aw->m_occ_vector_robust.size();i++){
            ptmp = in_aw->m_occ_vector_robust.at(i);
            if(ptmp->m_action == 1){
                if(ptmp->m_count == in_cap){//遇到了满容量时间窗
                    time_tmp = ioaStart(ptmp->m_ioa,ptmp->m_time) - delta;
                    if(time_tmp < (tl+in_ioa)){//容量冲突
                        return false;
                    }
                    else{
                        tr = time_tmp;
                        break;
                    }
                }

            }
            else if(ptmp->m_action == -1){//第一个进入该路段，所以也要第一个脱离
                tr = ioaEnd(in_ioa,ptmp->m_time - sep);//此时，当前航空器要先于其他航空器脱离该路段
                break;//找到之前第一个脱离动作后即跳出循环
            }
            /*if(in_aw->m_occ_vector_robust.at(i)->m_count == in_cap){//若在第一个out动作之前遇到满容量时间窗
                tr = ioaStart(in_aw->m_occ_vector_robust.at(i)->m_ioa,in_aw->m_occ_vector_robust.at(i)->m_time);
                break;
            }*/
        }
        if(tr-tl >= in_ioa){
            out_pew->tstart = tl;
            out_pew->tend = tr;
            return true;
        }
        else{
            return false;
        }
    }
    else if(i == in_aw->m_occ_vector_robust.size()){//if the entry time lie out of all the occpancy records, it's also straightforward
        pocc = in_aw->m_occ_vector_robust.at(in_aw->m_occ_vector_robust.size()-1);//最后一个out动作
        assert(pocc->m_action == -1);
        out_pew->tstart = fmax(ioaStart(in_ioa, pocc->m_time+sep),ioaStart(in_ioa, in_nnlength/in_speed + in_tenter));
        out_pew->tend = in_aw->tend - delta;
        if(out_pew->tend - out_pew->tstart >= in_ioa){
            return true;
        }
        else{
            return false;
        }
    }
    else{//一般的情况
        tl = ioaStart(in_ioa, in_nnlength/in_speed + in_tenter);//初始化tl和tr
        tr = in_aw->tend - delta;
        if(precount == 0){//表明之前进入的航空器已经全部离开
            pocc = in_aw->m_occ_vector_robust.at(i-1);
            assert(pocc->m_action == -1);
            double tmp = ioaStart(in_ioa, pocc->m_time+sep);
            if(tl < tmp){
                tl = tmp;
            }
            for(;i<in_aw->m_occ_vector_robust.size();i++){
                pocc = in_aw->m_occ_vector_robust.at(i);
                if(pocc->m_action == 1){
                    if(pocc->m_count == in_cap){
                        tr = ioaStart(pocc->m_ioa,pocc->m_time)-delta;
                        break;
                    }
                }
                else if(pocc->m_action == -1){
                    tr = ioaEnd(in_ioa,pocc->m_time-sep);
                    break;
                }
            }
            if(tr-tl >= in_ioa){
                out_pew->tstart = tl;
                out_pew->tend = tr;
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
                    if(pocc->m_count == in_cap){
                        return false;
                    }
                    if(out == precount){
                        status = 2;
                        double tmp = ioaEnd(in_ioa, pocc->m_time+sep);//最早可以脱离的时刻
                        if(tl < tmp){
                            tl = tmp;
                        }
                    }
                    break;
                case 2://确定了先前进入该路段的航空器中最晚离开该路段的脱离时刻后执行以下case
                    if(out == (precount+1)){
                        status = 3;
                        tr = fmin(ioaEnd(in_ioa, pocc->m_time-sep), tr);
                        if(tr-tl >= in_ioa){
                            out_pew->tstart = tl;
                            out_pew->tend = tr;
                            return true;
                        }
                        else{
                            return false;
                        }
                    }
                    else if(pocc->m_count == in_cap){
                        tr = ioaStart(pocc->m_ioa,pocc->m_time)-delta;
                        if(tr-tl >= in_ioa){
                            out_pew->tstart = tl;
                            out_pew->tend = tr;
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
        if(tr-tl > in_ioa){
            out_pew->tstart = tl;
            out_pew->tend = tr;
            return true;
        }
        else{
            return false;
        }
    }
}

bool EnterTime_Line_robust(TWVector& in_twv, twindow* in_ew, double in_ioa, uchar in_cap, double in_speed, double &out_tenter, uint& out_awid)
{

    uint twnum = in_twv.size();
    twindow* ptwtmp;
    double tenter;
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
                    double endtime = in_ioa+in_ew->tstart;//Te的结束时刻
                    assert(endtime<=in_ew->tend);
                    if(endtime > ptwtmp->tend){//
                        return false;
                    }
                    tenter = ntoa(in_ioa,in_ew->tstart);//最早可能脱离前一区域并进入当前区域的NTOA
                    out_tenter = tenter;
                    return true;
                }
                //else,确定当前航空器占用时刻在ORS中的位置
                tenter = ntoa(in_ioa,in_ew->tstart);
                assert(ptwtmp->m_occ_vector_robust.size()>=2);//若不为空，ORS中元素个数必定大于等于2
                int tmp = -1;
                for(uint index = 0; index < ptwtmp->m_occ_vector_robust.size();index++){
                    poccv = ptwtmp->m_occ_vector_robust.at(index);
                    if(poccv->m_action == 1){
                        if(poccv->m_time > tenter ){
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
                        double ts = ioaEnd(ptmp->m_ioa, ptmp->m_time)+g_len_plane/in_speed;////
                        //double tau = ntoa(in_ioa,ts);
                        if(ts <= in_ew->tstart){
                            double endtime = in_ioa+in_ew->tstart;//Te的结束时刻
                            assert(endtime<=in_ew->tend);
                            if(endtime > ptwtmp->tend){//
                                return false;
                            }
                            tenter = ntoa(in_ioa,in_ew->tstart);//最早可能脱离前一区域并进入当前区域的NTOA
                            out_tenter = tenter;
                            return true;
                        }
                        else{
                            double endtime = in_ioa+ts;
                            if(endtime <= in_ew->tend && endtime <= ptwtmp->tend){
                                tenter = ntoa(in_ioa,ts);
                                out_tenter = tenter;
                                return true;
                            }
                            else{
                                return false;
                            }
                        }
                    }
                    else{
                        double ts = ioaStart(in_ioa, prov->m_time+g_sep);
                        if(ts <= in_ew->tstart){
                            double endtime = in_ioa+in_ew->tstart;//Te的结束时刻
                            assert(endtime<=in_ew->tend);
                            if(endtime > ptwtmp->tend){//
                                return false;
                            }
                            tenter = ntoa(in_ioa,in_ew->tstart);//最早可能脱离前一区域并进入当前区域的NTOA
                            out_tenter = tenter;
                            return true;

                        }
                        else{
                            double endtime = in_ioa+ts;
                            if(endtime <= in_ew->tend && endtime <= ptwtmp->tend){
                                tenter = ntoa(in_ioa,ts);
                                out_tenter = tenter;
                                return true;
                            }
                            else{
                                return false;
                            }
                        }

                    }
                }
                //若是第一个
                else if(index_in1 == -1){
                    assert(index_in2 == 0);
                    rOccVariable* prov = ptwtmp->m_occ_vector_robust.at(0);
                    tenter = ntoa(in_ioa,in_ew->tstart);
                    if(prov->m_time-g_sep >= tenter){
                        out_tenter = tenter;
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
                    double tl = poccpre->m_time+g_sep;
                    double tr = poccv->m_time-g_sep;
                    double left = fmax(tl,in_ew->tstart);
                    double right = fmin(tr,in_ew->tend);
                    if(right-left < in_ioa){
                        return false;
                    }
                    if(poccpre->m_count < in_cap){
                        out_tenter = ntoa(in_ioa,left);
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
                                left = fmax(left,ioaEnd(poccv->m_ioa,poccv->m_time)+g_len_plane/in_speed);////
                                if(right-left >= in_ioa){
                                    out_tenter = ntoa(in_ioa,left);
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

double ioaEnd(double ioa, double ntoa)
{
    return (ntoa + ioa/2);
}
//double ioa(double len, double speed, double k)
//{
//    return k*(len/speed);
//}

void SavePathToFile_robust(uint id,rPath& rpath, QString in_filename)
{
    QFile file(in_filename);
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
        robust_step* pb_s;
        for(uint i=0; i<rpath.size();i++){
            pb_s = rpath.at(i);
            strline = QString("%1 %2 %3 %4").arg(pb_s->m_entrytime,0,'f',3)
                    .arg(GetNodeIndex(pb_s->m_bs)).arg(GetRegionIndex(pb_s->m_bs))
                    .arg(pb_s->m_IOA,0,'f',1);
            ts << strline << '\n';
        }
        file.close();
    }
}

void UpdateTimeWindowVector_robust(rPath& in_path, CRegionCollect &rct)
{
    if(in_path.empty()){
        return;
    }
    double delta;
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
    robust_step* pb_s1;
    robust_step* pb_s2;
    uint n = in_path.size();//
    for(uint i = 0; i<n-1; i++){
        pb_s1 = in_path.at(i);
        pb_s2 = in_path.at(i+1);
        rngid = GetRegionIndex(pb_s1->m_bs);
        ndid = GetNodeIndex(pb_s1->m_bs);
        act =  getAction(rngid,ndid,GetRegionIndex(pb_s2->m_bs));
        avg_speed = getSpeed(rngid,act);
        delta = g_len_plane/avg_speed;

        tw.tstart = ioaStart(pb_s1->m_IOA, pb_s1->m_entrytime);
        tw.tend = ioaEnd(pb_s2->m_IOA, pb_s2->m_entrytime)+delta;//考虑了机身的脱离时间

        prgn = rct.GetRegion(rngid);
        if(prgn->m_type == Line){
            dir =prgn->GetDirection(ndid);
            RemoveTimewindow_Line_robust(prgn->GetTWVector(1-dir),&tw);
            UpdateOccupation_robust(prgn->GetTWVector(dir), pb_s1, pb_s2, prgn->m_num_capacity);
        }
        else{//对Runway类似Inter进行处理（独占）
            RemoveTimewindow_Inter_robust(prgn->GetTWVector(), &tw);
        }
        //stand和buffer暂不处理
    }
}

void RemoveTimewindow_Inter_robust(TWVector& twv, twindow* in_ptw)
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

void UpdateOccupation_robust(TWVector& twv, robust_step* pb_s, robust_step* pb_e, uint in_cap)
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
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, pb_s->m_IOA, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,pb_e->m_IOA,-1);
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
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, pb_s->m_IOA, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,pb_e->m_IOA,-1);
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
        rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, 1, pb_s->m_IOA, 1);
        rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime,0,pb_e->m_IOA,-1);
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
            rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,pb_s->m_IOA,1);

            occv.insert(occv.begin()+indexstart, newocc1);
            //inoutv.insert(inoutv.begin()+indexstart, 1);
            rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, 0 , pb_e->m_IOA,-1,newocc1,g_veh_id);
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
                rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,pb_s->m_IOA,1);
                rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, curcap,pb_e->m_IOA,-1,newocc1,g_veh_id);
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
                rOccVariable* newocc1 = new rOccVariable(pb_s->m_entrytime, curcap+1,pb_s->m_IOA,1);
                poc = occv.at(indexend-1);
                curcap = poc->m_count;
                rOccVariable* newocc2 = new rOccVariable(pb_e->m_entrytime, curcap-1,pb_e->m_IOA,-1,newocc1,g_veh_id);
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


void RemoveTimewindow_Line_robust(TWVector& twv, twindow* in_ptw)
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
        //assert(tend < ptw->tstart);
//        if(tend > ptw->tstart){
//            int x = 0;
//        }
    }
    ptw = twv.at(index_aw);
    //从该时间窗中去除in_ptw
    if(tend <= ptw->tstart){//若in_ptw恰好落在时间窗之间的空白处，直接返回
        return;
    }
    else if(tend >= ptw->tend){//若in_ptw长度超过了当前time window(注意这里因为是从反向可用时间窗中去除占用时间窗，所以有可能超过)
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
                //判断in-out向量信息是否正确
                //int y;
                //if(ptw->GetOccVector().size()%2 != 0){
                //    y = ptw->GetOccVector().size();
                //}
                if(ptw->m_occ_vector_robust.size() > 0){
                    rOccVariable* pocc;
                    char io_tmp;
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
                            //io_tmp = ptw->GetInOutVector().at(i);
                            //pnewtw->m_occ_vector_robust.push_back(new rOccVariable(pocc->m_time,pocc->m_count,));
                            pnewtw->m_occ_vector_robust.push_back(pocc);
                            //pnewtw->GetInOutVector().push_back(io_tmp);

                        }
                        //for(uint i=index; i<ptw->GetOccVector().size();i++){
                        //    delete ptw->GetOccVector().at(i);
                        //}
                        ptw->m_occ_vector_robust.erase(ptw->m_occ_vector_robust.begin()+index, ptw->m_occ_vector_robust.end());
                        //ptw->GetInOutVector().erase(ptw->GetInOutVector().begin()+index, ptw->GetInOutVector().end());
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



uint getAction(uint srgnid, uint sndid, uint ergnid)
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

double getIOA(double in_ioa, double in_len, double in_speed, double k)
{
    return fmax(in_ioa,k*in_len/in_speed);//返回上一个step的IOA和当前运行区间可能造成的时间不确定性中的大者
    //return k*in_len/in_speed;//仅返回当前运行区间可能造成的时间不确定性
}

/////////////////////////////////for stand holding////////////////////////////////////////////

//double CalcRunwayArrivalTime(Vehicle* pveh)//for departing aircraft only
//{
//    uint ns = pveh->m_start_node;
//    uint ne = pveh->m_end_node;
//    double len = g_model.matrix_nn_sd[ns][ne];
//    return len/g_speed + pveh->m_start_time + g_len_plane/g_speed;//返回无阻碍运行的情况下到达目标位置的时刻
//}

//由于Stand holding问题的研究意义和作用还没搞太清楚，这里先暂停reverse搜索算法的编写：
//int Plan_robust_reverse(rPath& out_path, Vehicle *veh)//for departing aircraft, considering stand holding
//{
//    double exp_time = 0;
//    double t1, t2;

//    g_start_region = veh->m_end_region;
//    g_start_node = veh->m_end_node;
//    g_end_region = veh->m_start_region;
//    g_end_node = veh->m_start_node;
//    g_start_time = CalcRunwayArrivalTime(veh);
//    Heuristics(g_end_node);//启发值更新

//    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
//            ->GetAnotherZoneId(g_start_region);//第一个滑行道区域编号

//    Basicstep bs;
//    double tenter;
//    double ioa = g_ioa;//length of the initial IOA
//    //double k = 0.1;//按ioa/travel time=0.1
//    uint rgnid;
//    uint ndid;
//    uint awid;
//    ndid = g_start_node;
//    rgnid = first_taxiway_region;
//    twindow ew(ioaStart(ioa,g_start_time), c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
//    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the next zone to be visited, i.e., the first taxiway zone
//    /*tackle the first taxiway zone*/
//    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
//        WindowTimeTable wtt;
//        uint num;
//        if(!Overlapped_robust(&ew,ioa, prgn->GetTWVector(),num,wtt)){//
//            QString info = QString("error, the entertime of target %1 is not properly set").arg(veh->m_id);
//            qDebug(info.toAscii());
//            return 1;
//        }
//        else{
//            WindowTimeTable::iterator itr;
//            for(itr=wtt.begin();itr!=wtt.end();itr++){
//                awid = itr->first;
//                tenter = itr->second;//i.e., NTOA
//                bs = TernaryToUnary(rgnid, ndid, awid);
//                double cost;
//                if(veh->m_type == DEPAR){
//                    cost = 0;//直接取0合适吗？
//                }
//                else if(veh->m_type == ARRIV){
//                    cost = calCost(0,g_speed,tenter-g_start_time);//0表示这里假定从起始位置到第一个控制的距离为0，更实际的方式可以考虑将其作为规划输入参数
//                }
//                else{
//                    cost = 0;
//                    qDebug() << QString("Unkown vehicle type. Please check.");
//                }
//                robust_step* b_s = new robust_step(bs, tenter, ioa,cost);//TODO(Ready):增加robust_step类三参数的构造函数
//                openlistexp.push_back(b_s);
//                robust_step* b_s_pre = new robust_step(TernaryToUnary(g_start_region,0,0));
//                closelist.push_back(b_s_pre);
//                b_s->m_prestep = b_s_pre;
//            }
//        }
//    }
//    else if(prgn->m_type == Line){//if the first taxiway zone is a lane (this case should only appear for arrival aircrafts)
//        qDebug() << QString("the first taxiway shouldn't be a lane");
//    }
//    /*do iterative search until: 1.the openlist is empty  2.the target bs is chosen for expansion*/
//    uint curregion;//current region (for expansion)
//    uint curnode;//current node (for expansion)
//    int status = 1;
//    robust_step* pb_s;
//    while(!openlistexp.empty()){
//        g_iterationcount++;//迭代次数增加1

//        uint n = openlistexp.size();


//        GetMinHeuristicCostOfOpenlist_robust(pb_s);//get least cost bs from open

//        bs = pb_s->m_bs;
//        curregion = GetRegionIndex(bs);
//        curnode = GetNodeIndex(bs);

//        if((curregion == g_end_region) && (curnode == g_end_node)){//path found
//            rPath path;
//            path.push_back(pb_s);
//            do{
//                pb_s = (robust_step*)pb_s->m_prestep;
//                path.push_back(pb_s);
//                bs = pb_s->m_bs;
//            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
//            uint n = path.size();
//            for(uint i=0; i<n;i++){
//                out_path.push_back(path.at(n - 1 - i));//调正顺序
//            }
//            status =  0;//denote path found

//            //g_expandtime_vector.push_back(exp_time);//将扩展过程所用的时间保存
//            break;//退出while
//        }
//        else{
//            //t1 = GetHighPrecisionCurrentTime();
//            Expand_robust(pb_s);//preregion is used to prevent doubling back
//            //t2 = GetHighPrecisionCurrentTime();
//            //exp_time += t2 - t1;
//        }

//    }

//    return status;
//}
