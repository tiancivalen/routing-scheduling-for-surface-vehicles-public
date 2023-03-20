#include "TP_robust_multiple.h"
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


void Experiment_Robust_multiple()
{
    printParam(QString("..\\data_out\\multipath\\参数设置.txt"));
    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件
    SequencePlan_robust_multiple();
    printResult_multiple(QString("..\\data_out\\multipath\\multipath_den%1_num%2.txt")
         .arg(g_frequency).arg(g_pathnumber));
}

void Experiment_Robust_multiple4comp()
{
    printParam(QString("..\\data_out\\multipath\\参数设置.txt"));
    ParsePlanFile4comp(g_model.file_dir+"sequenceplan.txt");//解析计划文件
    SequencePlan_robust_multiple4comp();
    printResult4comp(QString("..\\data_out\\multipath\\singlepath4comp_den%1_num%2.txt")
                         .arg(g_frequency).arg(g_pathnumber));
    printResult_multiple(QString("..\\data_out\\multipath\\multipath_den%1_num%2.txt")
         .arg(g_frequency).arg(g_pathnumber));
}

void printResult4comp(QString filename)
{
    QFile file(filename);
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the record file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        Vehicle* pveh;
        for(uint i=0;i<g_vehs4comp.size();i++){
            pveh = g_vehs4comp.at(i);
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


void ParsePlanFile4comp(QString planfilename)
{
    g_vehs.clear(); //复位全局变量
    g_vehs4comp.clear();
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
        //g_vehs.clear();
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

                Vehicle* pveh = new Vehicle;
                pveh->m_id = veh->m_id;
                pveh->m_start_time = veh->m_start_time;
                pveh->m_start_region = veh->m_start_region;
                pveh->m_type = veh->m_type;
                pveh->m_start_node = veh->m_start_node;
                pveh->m_end_region = veh->m_end_region;
                pveh->m_end_node = veh->m_end_node;
                g_vehs4comp.push_back(pveh);

                g_objcount++;
            }
        }
        file.close();
    }
}

int SequencePlan_robust_multiple4comp()
{
    int returnvalue;
    QString fdir = QString("..\\data_out\\multipath");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    Vehicle* veh4comp;
    uint num = g_vehs.size();
    num = (uint)fmin(g_plan_num,num);////----------两者取其小------------//////
//    num = 60;
    for(uint i=0; i<num; i++){
        veh = g_vehs.at(i);
        veh4comp = g_vehs4comp.at(i);
        g_veh_id = veh->m_id;//id ready
        if(g_priority%50 == 0){//每若干次循环清理一次时间窗
            TimeWindowSlimming(veh->m_start_time);
        }
        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_robust_multiple(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue == 0){
            adjustArrivalRunwayOccupancy_multiple(veh);//对进港航空器，将跑道上的等待转移到起始位置
            ctvector.push_back(t2-t1);
            pathSelect(veh);//从候选path集合中选择一条,然后才能更新时间窗信息
            itr_cnt_vector.push_back(g_iterationcount);

            ResetGlobalVariables();
            int r = Plan_robust(veh4comp->m_rPath,veh4comp);
            adjustArrivalRunwayOccupancy(veh4comp);

            if(g_veh_id == 127){
                printAllPaths(veh,fdir);
                printPathDetail(veh4comp->m_rPath,fdir);
            }
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);

            if(g_veh_id == 53){
                QString file = QString("..\\data_out\\Robust_twindowaftertarget_%1.txt").arg(veh->m_id);
                PrintTimeWindows(file, g_model.m_rct);//for debug
            }
        }
        else{
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }
        UpdateTimeWindowVector_robust(veh->m_rPath,g_model.m_rct);
        ResetGlobalVariables();//当前航空器规划完毕后复位必要的全局变量：
        g_priority++;//denote agent's planning priority

        //-----线程休眠一段时间以降低CPU占用率
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){
            QCoreApplication::processEvents();
        }
        //---------------------------------
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


void adjustArrivalRunwayOccupancy_multiple(Vehicle* pveh)
{
    if(pveh->m_type == DEPAR){
        return;
    }
    if(pveh->m_SinglePathLenVector.empty()){
        qDebug() << QString("M: No path found for vehicle %1").arg(pveh->m_id);
        return;
    }
    robust_step* prs;
    robust_step* prs_next;
    uint sid = 0;
    for(uint i=0; i<pveh->m_SinglePathLenVector.size();i++){
        uint n = pveh->m_SinglePathLenVector.at(i);
        assert(n>1);
        prs = pveh->m_rPath_all.at(sid);//第一个step
        prs_next = pveh->m_rPath_all.at(sid+1);//第二个step
        //assert(prs->m_cost_delay < 1e-3);//这个版本假定不计入在起始位置的等待
        double delay = prs_next->m_cost_delay - prs->m_cost_delay;
        if(prs_next->m_cost_delay > 1e-3){//说明在进入快速脱离道之前进行了等待
            prs->m_entrytime += delay;
            prs->m_cost_delay += delay;
            //prs_next->m_cost_delay = prs->m_cost_delay;//在跑道上不等待
        }
        sid = sid+n;
    }
}

void printAbstractPathsInfo(Vehicle* veh,QString fdir)
{
    if(veh->m_rPath_all.empty()){
        qDebug() << QString("No path found for vehicle %1").arg(veh->m_id);
        return;
    }
    QFile file(QString("%1\\AbstractPathsofVehicle%2.txt").arg(fdir).arg(veh->m_id));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the path detail file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        uint sid = 0;//start index for each path
        for(uint i=0; i<veh->m_SinglePathLenVector.size();i++){
            uint n = veh->m_SinglePathLenVector.at(i);
            assert(n>1);
            robust_step* prs = veh->m_rPath_all.at(sid);
            robust_step* prs_end = veh->m_rPath_all.at(sid+n-1);

            double finish_time = prs_end->m_entrytime;//
            double delay_start = prs->m_entrytime - veh->m_start_time;
            double runway_time;
            robust_step* prsrunway;
            if(veh->m_type == ARRIV){
                prsrunway = veh->m_rPath_all.at(sid+1);
                assert(fabs(prsrunway->m_cost_delay - prs->m_cost_delay) < 1e-3);//在跑道不能等待
                runway_time = prsrunway->m_entrytime - prs->m_entrytime;
            }
            else{
                assert(veh->m_type == DEPAR);
                prsrunway = veh->m_rPath_all.at(sid+n-2);
                assert(fabs(prs_end->m_cost_delay - prsrunway->m_cost_delay)<1e-3);//在跑道不能等待
                runway_time = prs_end->m_entrytime - prsrunway->m_entrytime;
            }
            double taxi_time = prs_end->m_entrytime - prs->m_entrytime - runway_time;
            double delay_taxi = prs_end->m_cost_delay - prs->m_cost_delay;
            /*跑道上的delay为0，所以不用计算*/

            robust_step* prs_next;
            double len = 0;
            for(uint j=1; j<n; j++){
                prs_next = veh->m_rPath_all.at(j+sid);
                len = len + g_model.matrix_nn_dis[GetNodeIndex(prs->m_bs)][GetNodeIndex(prs_next->m_bs)];
                prs = prs_next;
            }

            //finish time  | runway time| taxi time | distance | delay during taxiing | delay at the start position
            QString strline = QString("%1\t%2\t%3\t%4\t%5\t%6")
                    .arg(finish_time,0,'f',2).arg(runway_time,0,'f',2)
                    .arg(taxi_time,0,'f',2).arg(len,0,'f',2)
                    .arg(delay_taxi,0,'f',2).arg(delay_start,0,'f',2);
            ts << strline << '\n';
            sid = sid+n;//
        }
        file.close();
    }
}

int SequencePlan_robust_multiple()
{
    int returnvalue;
    QString fdir = QString("..\\data_out\\multipath");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    num = (uint)fmin(g_plan_num,num);////----------两者取其小------------//////
    for(uint i=0; i<num; i++){
        //rPath robustPath;//TODO:要在头文件中定义这个数据类型
        veh = g_vehs.at(i);
        g_veh_id = veh->m_id;//id ready
//        if(g_veh_id == 6){
//            int x = 0;
//        }
        if(g_priority%50 == 0){//每若干次循环清理一次时间窗
            TimeWindowSlimming(veh->m_start_time);
        }
        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_robust_multiple(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue == 0){
            adjustArrivalRunwayOccupancy_multiple(veh);//对进港航空器，将跑道上的等待转移到起始位置
            ctvector.push_back(t2-t1);//求解时间
            pathSelect(veh);//从候选path集合中选择一条,然后才能更新时间窗信息
            UpdateTimeWindowVector_robust(veh->m_rPath,g_model.m_rct);
            qDebug()<<QString("path generated for agent %1").arg(veh->m_id);
            //if(g_veh_id <= 12){
            //printPathDetail(veh->m_rPath,fdir);

                //printAllPaths(veh,fdir);
                printAbstractPathsInfo(veh,fdir);
            //}
//            if(g_veh_id == 53){
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

        //-----线程休眠一段时间以降低CPU占用率
        QElapsedTimer t;
        t.start();
        while(t.elapsed()<1){
            QCoreApplication::processEvents();
        }
        //---------------------------------
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

int Plan_robust_multiple(Vehicle *veh)//TODO:如何考虑stand holding问题？
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
                double cost_time;
                double cost_delay;
                if(veh->m_type == DEPAR){
                    if(g_flag_startdelay){
                        cost_time = 0;
                        cost_delay = tenter - veh->m_start_time;
                    }
                    else{
                        cost_time = 0;
                        cost_delay = 0;
                    }

                }
                else if(veh->m_type == ARRIV){
                    if(g_flag_startdelay){
                        cost_time = 0;
                        cost_delay = tenter - veh->m_start_time;
                    }
                    else{
                        cost_time = 0;
                        cost_delay = 0;
                    }
                }
                else{
                    cost_time = 0;
                    cost_delay = 0;
                    qDebug() << QString("Unkown vehicle type. Please check.");
                }

                robust_step* b_s = new robust_step(bs, tenter, ioa,cost_time,cost_delay);//TODO(Ready):增加robust_step类三参数的构造函数
                openlistexp.push_back(b_s);//这里不考虑dominate的问题，直接加入openlist
                robust_step* b_s_pre = new robust_step(TernaryToUnary(g_start_region,0,0),0,0,0,0);
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
    robust_step* pb_s;
    //-----------------------//
    const uint ordernum = 2;//
    uint ordercount = 0;
    //-----------------------//
    while(!openlistexp.empty()){
        g_iterationcount++;//迭代次数增加1

        uint n = openlistexp.size();


        GetMinHeuristicCostOfOpenlist_robust_multiple(pb_s);//get least cost bs from open

        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
//        if(g_veh_id == 12 && curregion == 163
//                && curnode == 203){
//            int xx = 0;
//        }

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found
            ordercount++;//path个数加1
//            if(veh->m_id == 6 && ordernum != ordercount){
//                continue;
//            }
            //else{
            rPath path;
            path.push_back(pb_s);
            do{
                pb_s = (robust_step*)pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                veh->m_rPath_all.push_back(path.at(n - 1 - i));//调正顺序
            }
            veh->m_SinglePathLenVector.push_back(n);//记录step个数

            status =  0;//denote path found

            //g_expandtime_vector.push_back(exp_time);//将扩展过程所用的时间保存
            if(ordercount == g_pathnumber){//如果已经生成的path个数达到了人为指定的上限，则退出搜索
                qDebug() << QString("Required number of paths found for vehicle %1").arg(g_veh_id);
                break;//退出while
            }

            //}

        }
        else{
            //t1 = GetHighPrecisionCurrentTime();
            Expand_robust_multiple(pb_s);//preregion is used to prevent doubling back
            //t2 = GetHighPrecisionCurrentTime();
            //exp_time += t2 - t1;
        }

    }

    return status;
}


void Expand_robust_multiple(robust_step* in_bs)
{
    double k = g_k;//计算IOA时会用到
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
//    if(currid == 446&&curnodeid==313 && g_veh_id == 6){
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
            ioa_first = getIOA(in_bs->m_IOA,nnlength,avg_speed,k);


            uint paircount = 0;
            WindowTimeTable wtt;
            //这里的扩展基准只可能是Inter或Runway型，所以:
            if(ExitWindow_Intersection_robust(in_bs->m_entrytime,ioa_first,nnlength, paw, pew, avg_speed)){
                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){
//                    if(g_veh_id == 1 && neighbor_zone_id == 418){
//                        int xx = 0;
//                    }
                    uint awid_line;
                    dir_nb = pnb->GetDirection(nd_id);
                    if(!EnterTime_Line_robust(pnb->GetTWVector(dir_nb), pew,ioa_first, pnb->GetCapacity(),avg_speed, tenter, awid_line)){
                        continue;//if the neighbor lane is not accessible, continue
                    }
                    else{//若该路段可以进入，需要进一步确定是否可脱离

                        double cost1_time = nnlength/avg_speed;
                        double cost1_delay = tenter - in_bs->m_entrytime - cost1_time;
                        //double cost1 = calCost(nnlength,avg_speed,tenter - in_bs->m_entrytime);//从交叉口入口运行到相邻路段入口所花费的代价

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
                        uint nextinterid = g_model.m_ndct.m_array_node[next_node_id]->GetAnotherZoneId(neighbor_zone_id);//another neighbor intersection's id of the lane( assumes that a lane connects with an intersection at either end)
                        act = getAction(neighbor_zone_id,nd_id,nextinterid);
                        avg_speed = getSpeed(neighbor_zone_id,act);
                        ioa_second = getIOA(ioa_first,nnlength,avg_speed,k);

                        if(!ExitWindow_Line_robust(tenter,ioa_second, pnbaw, pnb->GetCapacity(), nnlength, pew, avg_speed)){
                            continue;//if the lane is not exit-able, continue
                        }
                        else{
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
                                        double cost2_time = nnlength/avg_speed;
                                        double cost2_delay = entertime - tenter - cost2_time;
                                        //double cost2 = calCost(nnlength,avg_speed,entertime-tenter);
                                        TernaryToUnary(nextinterid, next_node_id, awid, bs);
                                        RobustStepVector rsvec;
                                        bool flag = false;
                                        b_s_tmp = new robust_step(bs,entertime,ioa_second,
                                                                  in_bs->m_cost_time+cost1_time+cost2_time,
                                                                  in_bs->m_cost_delay+cost1_delay+cost2_delay);
                                        if(Closed_robust_multiple(bs,rsvec)){
                                            uint n = rsvec.size();
                                            for(uint i=0; i<n; i++){
                                                robust_step* prs = rsvec.at(i);
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
                                        if(Opened_robust_multiple(bs, rsvec_open)){//判断是否open，并确定相应的base_step指针
                                            //b_s_tmp = new robust_step(bs,entertime,ioa_second,cost1_time+cost2_time,cost1_delay+cost2_delay);
                                            assert(!rsvec_open.empty());
                                            uint count = 0;//记录已经从openlist中删除的元素个数
                                            uint n = rsvec_open.size();
                                            for(uint i = 0; i<n; i++){
                                                robust_step* prs = (robust_step*)openlistexp.at(rsvec_open.at(i)-count);
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
                                        //for the lane
                                        b_s_lane = new robust_step(curbs, tenter, ioa_first,
                                                                   in_bs->m_cost_time+cost1_time,
                                                                   in_bs->m_cost_delay+cost1_delay);
                                        g_tmp_basestepvector.push_back(b_s_lane);//添加到临时容器，最后统一释放内存
                                        b_s_lane->m_prestep = in_bs;
                                        //for the intersection
                                        b_s_tmp->m_prestep = b_s_lane;
                                        openlistexp.push_back(b_s_tmp);//add to open


                                    }
                                }
                            }
                        }
                    }
                }
                /*if current neighbor is an inter or a runway*/
                else if((pnb->m_type == Inter) || (pnb->m_type == Runway)){

//                    if(g_veh_id == 12 && neighbor_zone_id == 172
//                            && nd_id == 216){
//                        int xx = 0;
//                    }

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
                            double costtmp_time = nnlength/avg_speed;
                            double costtmp_delay = entertime - in_bs->m_entrytime - costtmp_time;
                            //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                            TernaryToUnary(neighbor_zone_id, nd_id, awid, bs);
                            RobustStepVector rsv;
                            b_s_tmp = new robust_step(bs,entertime,ioa_first,
                                                      in_bs->m_cost_time+costtmp_time,
                                                      in_bs->m_cost_delay+costtmp_delay);
                            bool flag = false;
                            if(Closed_robust_multiple(bs,rsv)){
                                assert(!rsv.empty());
                                uint n = rsv.size();
                                for(uint i=0; i<n; i++){
                                    robust_step* prs = rsv.at(i);
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
                            if(Opened_robust_multiple(bs, rsv_open)){
                                uint count = 0;//记录已经从openlist中删除的元素个数
                                assert(!rsv_open.empty());
                                uint n = rsv_open.size();
                                for(uint i=0; i<n; i++){
                                    robust_step* prs = (robust_step*)openlistexp.at(rsv_open.at(i)-count);
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
                    robust_step* b_s_tmp;
                    Basicstep bs;
                    double entertime = ntoa(ioa_first,pew->tstart);
                    double costtmp_time = nnlength/avg_speed;
                    double costtmp_delay = entertime - in_bs->m_entrytime - costtmp_time;
                    //double costtmp = calCost(nnlength,avg_speed,entertime-in_bs->m_entrytime);
                    TernaryToUnary(neighbor_zone_id, nd_id, 0, bs);
                    RobustStepVector rsv;
                    bool flag = false;
                    b_s_tmp = new robust_step(bs,entertime,ioa_first,
                                              in_bs->m_cost_time+costtmp_time,
                                              in_bs->m_cost_delay+costtmp_delay);
                    if(Closed_robust_multiple(bs,rsv)){
                        assert(!rsv.empty());
                        uint n = rsv.size();
                        for(uint i=0; i<n; i++){
                            robust_step* prs = rsv.at(i);
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
                    if(Opened_robust_multiple(bs, rsidv)){
                        uint count = 0;//记录已经从openlist中删除的元素个数
                        assert(!rsidv.empty());
                        uint n = rsidv.size();
                        for(uint i=0; i<n; i++){
                            robust_step* prs = (robust_step*)openlistexp.at(rsidv.at(i)-count);//因为删除元素导致后面的元素位置前移了
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


void pathSelect(Vehicle* veh)
{
    uint select = 0;//默认选择第一个path
    uint sid = 0;
    for(uint i=0; i<select; i++){
        sid = sid+veh->m_SinglePathLenVector.at(i);
    }
    uint n = veh->m_SinglePathLenVector.at(select);
    assert(veh->m_rPath.empty());
    for(uint i=0; i<n; i++){
        veh->m_rPath.push_back(veh->m_rPath_all.at(sid+i));
    }
}

bool Opened_robust_multiple(Basicstep in_bs, std::vector<uint> &rsv)
{
    rsv.clear();
    robust_step* pb_s;
    for(uint i=0; i<openlistexp.size();i++){
        pb_s = (robust_step*)openlistexp.at(i);
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

bool Closed_robust_multiple(Basicstep in_bs, RobustStepVector& rsv)
{
    robust_step* pb_s;
    for(uint i=0; i<closelist.size();i++){
        pb_s = (robust_step*)closelist.at(i);
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

void GetMinHeuristicCostOfOpenlist_robust_multiple(robust_step *&out_bs)
{
    out_bs = (robust_step*)openlistexp.at(0);
    double cost_time = out_bs->m_cost_time + g_array_h[GetNodeIndex(out_bs->m_bs)]/g_runway_speed;//TODO:确定一下这样计算的启发值是不是admissible的
    double cost_delay = out_bs->m_cost_delay + 0;
    double cost = calCost_multiple(cost_time,cost_delay);
    double tmptime;
    double tmpdelay;
    double tmpcost;
    uint index = 0;
    robust_step* bs;
    uint ndid;
    for(uint i=0; i < openlistexp.size();i++){
        bs = (robust_step*)openlistexp.at(i);
        ndid = GetNodeIndex(bs->m_bs);
        tmptime = bs->m_cost_time + g_array_h[ndid]/g_runway_speed;
        tmpdelay = bs->m_cost_delay + 0;
        tmpcost = calCost_multiple(tmptime,tmpdelay);
        if(cost > tmpcost){
            out_bs = bs;
            cost = tmpcost;
            index = i;
        }
    }
    //将它从openlist中取出,并加入closelist
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}

double calCost_multiple(double time, double delay)
{
    return g_w*time + g_w_delay*delay;
}

void printAllPaths(Vehicle* veh,QString fdir)
{
    if(veh->m_rPath_all.empty()){
        qDebug() << QString("No path info for vehicle %1").arg(veh->m_id);
        return;
    }
    QFile file(QString("%1\\AllPathsofVehicle%2.txt").arg(fdir).arg(veh->m_id));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug("Error: faied to create the path detail file, please check");
        return;
    }
    else{
        QTextStream ts(&file);
        uint sid = 0;//start index for each path
        for(uint i=0;i<veh->m_SinglePathLenVector.size();i++){
            double len = 0;
            uint n = veh->m_SinglePathLenVector.at(i);
            assert(n>1);
            robust_step* prs = veh->m_rPath_all.at(sid);
            QString strline = QString("%1 %2 %3 %4 %5").arg(prs->m_entrytime,0,'f',3)
                    .arg(GetNodeIndex(prs->m_bs)).arg(GetRegionIndex(prs->m_bs))
                    .arg(prs->m_IOA,0,'f',1).arg(prs->m_entrytime - veh->m_start_time,0,'f',1);
            ts << strline << '\n';
            robust_step* prs_next;
            for(uint j=1; j<n; j++){
                prs_next = veh->m_rPath_all.at(j+sid);
                QString strline = QString("%1 %2 %3 %4 %5").arg(prs_next->m_entrytime,0,'f',3)
                        .arg(GetNodeIndex(prs_next->m_bs)).arg(GetRegionIndex(prs_next->m_bs))
                        .arg(prs_next->m_IOA,0,'f',1).arg(prs_next->m_cost_delay - prs->m_cost_delay,0,'f',3);
                ts << strline << '\n';
                len = len + g_model.matrix_nn_dis[GetNodeIndex(prs->m_bs)][GetNodeIndex(prs_next->m_bs)];
                prs = prs_next;
            }
            ts <<'\n' << "Distance traveled: " << len << '\n';
            ts << '\n'<<'\n' << '\n'<<'\n';

            sid = sid+n;//
        }
        file.close();
    }
}

void printResult_multiple(QString filename)
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
            //delay
            double delay_taxi = rse->m_cost_delay - rss->m_cost_delay;//经过仔细考虑，由于输入数据是随机生成的，和真实数据相比很可能存在一定的不合理性，所以这里暂时在结果分析时暂时区别对待在起始位置的等待和在滑行过程中的等待
            double delay_start = rss->m_entrytime - pveh->m_start_time;
//            if(g_flag_startdelay){
//                delay = delay + rss->m_entrytime - pveh->m_start_time;//
//            }
//            if(pveh->m_type == ARRIV){
//                delay = delay + rss->m_entrytime - pveh->m_start_time;
//            }
            if(fabs(delay_taxi)<1e-3){
                delay_taxi = 0;
            }
            //save result to file
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
            //输出格式为：标识  进离港类型  运行时间  移动距离  滑行中的等待时间  起始位置的等待时间  计算时间  找到的路径数量 迭代次数
            ts << QString("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\t%9\n")
                  .arg(id).arg(str)
                  .arg(taxitime,0,'f',1).arg(taxidis,0,'f',1)
                  .arg(delay_taxi,0,'f',1).arg(delay_start,0,'f',1)
                  .arg(ctvector.at(i),0,'f',2).arg(pveh->m_SinglePathLenVector.size())
                  .arg(itr_cnt_vector.at(i));
            //ts << QString("%1\t%2\n").arg(id).arg(taxitime,0,'f',1);
        }
        file.close();
    }
}

