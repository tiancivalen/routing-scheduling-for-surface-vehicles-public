#include "TP_nowaiting_compare.h"
#include "TP_nowaiting.h"
#include "TP_nowaiting_orig.h"
#include "TP_nowaiting_include.h"
#include <QElapsedTimer>
#include <QCoreApplication>
#include <QDebug>
#include "globalvariable.h"
#include "TaxiPlanningAlgorithm.h"
#include <math.h>
#include <assert.h>
#include "operation.h"

ComputationTimeVector ctvector_orig;
IterationCountVector  itr_cnt_vector_orig;
extern double g_frequency;

void Experiment_nowaiting_compare()
{
    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件

    //sequential planning for both methods with the same time window constraints
    int returnvalue;
    QString fdir = QString("..\\EXP\\ForCompare_nowaiting\\WithHoldConstraint");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
        //if(veh->m_id == 164){
        //    int x = 0;
        //}
        //----------每若干次循环清理一次时间窗-----------
        if(g_priority%50 == 0){
            TimeWindowSlimming(veh->m_start_time);
        }
        //------------------------------------------

        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_nowaiting(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue != 0){
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }
        else{
            qDebug()<<QString("path found for agent %1").arg(veh->m_id);
        }
        genTBST_nowaiting(veh);
        printPathDetail_nowaiting(veh->m_path,fdir+"\\nowaiting");
        itr_cnt_vector.push_back(g_iterationcount);//记录迭代次数
        ctvector.push_back(t2-t1);//calclation time recoding
        ResetGlobalVariables();

        Path path_tmp;
        t1 = GetHighPrecisionCurrentTime();
        Plan_nowaiting_orig(veh,path_tmp);
        t2 = GetHighPrecisionCurrentTime();
        printPathDetail_nowaiting_orig(path_tmp,fdir+"\\original");
        ctvector_orig.push_back(t2-t1);//calclation time
        itr_cnt_vector_orig.push_back(g_iterationcount);//记录迭代次数
        //if(g_veh_id == 164){
        //    QString file = QString("..\\data_out\\twindowaftertarget_%1.txt").arg(veh->m_id);
        //    PrintTimeWindows(file, g_model.m_rct);//for debug
        //}
        //ctvector.push_back(t2-t1);//calclation time recoding
        UpdateTimeWindowVector_nowaiting(veh->m_path,g_model.m_rct);

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


    PrintComputeTime(fdir+"\\computetime_nowaiting.txt", ctvector);
    PrintIterationCount(fdir+"\\iterationcount_nowaiting.txt",itr_cnt_vector);
    PrintComputeTime(fdir+"\\computetime_orig.txt", ctvector_orig);
    PrintIterationCount(fdir+"\\iterationcount_orig.txt",itr_cnt_vector_orig);

}

void Experiment_nowaiting_compare_WithoutHC()
{
    //去掉hold约束
    uint nd_count = g_model.m_ndct.m_count_node;
    for(uint i=0; i<nd_count; i++){
        for(uint j=0; j<nd_count; j++){
            g_model.matrix_nn_hold[i][j] = 1;//全部置为1
        }
    }


    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件

    //sequential planning for both methods with the same time window constraints
    int returnvalue;
    QString fdir = QString("..\\EXP\\ForCompare_nowaiting\\WithoutHC");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
        //if(veh->m_id == 164){
        //    int x = 0;
        //}
        //----------每若干次循环清理一次时间窗-----------
        if(g_priority%50 == 0){
            TimeWindowSlimming(veh->m_start_time);
        }
        //------------------------------------------

        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_nowaiting(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue != 0){
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }
        else{
            qDebug()<<QString("path found for agent %1").arg(veh->m_id);
        }
        genTBST_nowaiting(veh);
        printPathDetail_nowaiting(veh->m_path,fdir+"\\nowaiting");
        itr_cnt_vector.push_back(g_iterationcount);//记录迭代次数
        ctvector.push_back(t2-t1);//calclation time recoding
        ResetGlobalVariables();
        //
        Path path_tmp;
        t1 = GetHighPrecisionCurrentTime();
        Plan_nowaiting_orig(veh,path_tmp);
        t2 = GetHighPrecisionCurrentTime();
        printPathDetail_nowaiting_orig(path_tmp,fdir+"\\original");
        ctvector_orig.push_back(t2-t1);//calclation time
        itr_cnt_vector_orig.push_back(g_iterationcount);//记录迭代次数
        //if(g_veh_id == 164){
        //    QString file = QString("..\\data_out\\twindowaftertarget_%1.txt").arg(veh->m_id);
        //    PrintTimeWindows(file, g_model.m_rct);//for debug
        //}
        //
        UpdateTimeWindowVector_nowaiting(veh->m_path,g_model.m_rct);



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

    PrintComputeTime(fdir+"\\computetime_nowaiting.txt", ctvector);
    PrintIterationCount(fdir+"\\iterationcount_nowaiting.txt",itr_cnt_vector);
    PrintComputeTime(fdir+"\\computetime_orig.txt", ctvector_orig);
    PrintIterationCount(fdir+"\\iterationcount_orig.txt",itr_cnt_vector_orig);
}

void Experiment_nowaiting_compare_FullHC()
{
    //
    uint nd_count = g_model.m_ndct.m_count_node;
    for(uint i=0; i<nd_count; i++){
        for(uint j=0; j<nd_count; j++){
            g_model.matrix_nn_hold[i][j] = 0;//全部置为0
        }
    }


    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件

    //sequential planning for both methods with the same time window constraints
    int returnvalue;
    QString fdir = QString("..\\EXP\\ForCompare_nowaiting\\FullHC");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
        //if(veh->m_id == 164){
        //    int x = 0;
        //}
        //----------每若干次循环清理一次时间窗-----------
        if(g_priority%50 == 0){
            TimeWindowSlimming(veh->m_start_time);
        }
        //------------------------------------------

        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_nowaiting(veh);//TODO:注意这里Path是否需要重新定义，因为引入了IOA
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue != 0){
            if(returnvalue == 1)
                qDebug()<<QString("failed to find feasible path for agent %1: openlist is empty!").arg(veh->m_id);
            else if(returnvalue== 2)
                qDebug()<<QString("failed to find feasible path for agent %1: time out!").arg(veh->m_id);
            else{
                qDebug()<<QString("failed to find feasible path for agent %1").arg(veh->m_id);
            }
        }
        else{
            qDebug()<<QString("path found for agent %1").arg(veh->m_id);
        }
        genTBST_nowaiting(veh);
        printPathDetail_nowaiting(veh->m_path,fdir+"\\nowaiting");
        itr_cnt_vector.push_back(g_iterationcount);//记录迭代次数
        ctvector.push_back(t2-t1);//calclation time recoding
        ResetGlobalVariables();
        //
//        Path path_tmp;
//        t1 = GetHighPrecisionCurrentTime();
//        Plan_nowaiting_loopless_orig(veh,path_tmp);
//        t2 = GetHighPrecisionCurrentTime();
//        printPathDetail_nowaiting_orig(path_tmp,fdir+"\\original");
//        ctvector_orig.push_back(t2-t1);//calclation time
//        itr_cnt_vector_orig.push_back(g_iterationcount);//记录迭代次数
        //if(g_veh_id == 164){
        //    QString file = QString("..\\data_out\\twindowaftertarget_%1.txt").arg(veh->m_id);
        //    PrintTimeWindows(file, g_model.m_rct);//for debug
        //}
        //
        UpdateTimeWindowVector_nowaiting(veh->m_path,g_model.m_rct);



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

    PrintComputeTime(fdir+"\\computetime_nowaiting.txt", ctvector);
    PrintIterationCount(fdir+"\\iterationcount_nowaiting.txt",itr_cnt_vector);
//    PrintComputeTime(fdir+"\\computetime_orig.txt", ctvector_orig);
//    PrintIterationCount(fdir+"\\iterationcount_orig.txt",itr_cnt_vector_orig);
}

void Experiment_mtt_lane()
{
    //
    uint nd_count = g_model.m_ndct.m_count_node;
    for(uint i=0; i<nd_count; i++){
        for(uint j=0; j<nd_count; j++){
            g_model.matrix_nn_hold[i][j] = 1 - g_model.matrix_nn_hold[i][j];//lane 0, intersection 1
        }
    }


    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");//解析计划文件

    //sequential planning for both methods with the same time window constraints
    int returnvalue;
    QString fdir = QString("..\\EXP\\ForCompare_nowaiting\\LaneHC");
    g_array_h = new double[g_model.m_ndct.m_count_node];//初始化启发值数组
    g_priority = 1;//初始化当前移动目标的规划优先级为1
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);////----------两者取其小--------------//////
    for(uint i=0; i<num; i++){
        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
        //if(veh->m_id == 164){
        //    int x = 0;
        //}
        //----------每若干次循环清理一次时间窗-----------
        if(g_priority%50 == 0){
            TimeWindowSlimming(veh->m_start_time);
        }
        //------------------------------------------

        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_nowaiting(veh);//
        double t2 = GetHighPrecisionCurrentTime();
        if(returnvalue != 0){
            if(returnvalue == 1)
                qDebug()<<QString("failed to find feasible path for agent %1: openlist is empty!").arg(veh->m_id);
            else if(returnvalue== 2)
                qDebug()<<QString("failed to find feasible path for agent %1: time out!").arg(veh->m_id);
            else{
                qDebug()<<QString("failed to find feasible path for agent %1").arg(veh->m_id);
            }
        }
        else{
            qDebug()<<QString("path found for agent %1").arg(veh->m_id);
        }
        genTBST_nowaiting(veh);
        printPathDetail_nowaiting(veh->m_path,fdir+"\\nowaiting");
        itr_cnt_vector.push_back(g_iterationcount);//记录迭代次数
        ctvector.push_back(t2-t1);//calclation time recoding
        ResetGlobalVariables();
        //
//        Path path_tmp;
//        t1 = GetHighPrecisionCurrentTime();
//        Plan_nowaiting_loopless_orig(veh,path_tmp);
//        t2 = GetHighPrecisionCurrentTime();
//        printPathDetail_nowaiting_orig(path_tmp,fdir+"\\original");
//        ctvector_orig.push_back(t2-t1);//calclation time
//        itr_cnt_vector_orig.push_back(g_iterationcount);//记录迭代次数
        //if(g_veh_id == 164){
        //    QString file = QString("..\\data_out\\twindowaftertarget_%1.txt").arg(veh->m_id);
        //    PrintTimeWindows(file, g_model.m_rct);//for debug
        //}
        //
        UpdateTimeWindowVector_nowaiting(veh->m_path,g_model.m_rct);



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

    PrintComputeTime(fdir+"\\computetime_nowaiting.txt", ctvector);
    PrintIterationCount(fdir+"\\iterationcount_nowaiting.txt",itr_cnt_vector);
//    PrintComputeTime(fdir+"\\computetime_orig.txt", ctvector_orig);
//    PrintIterationCount(fdir+"\\iterationcount_orig.txt",itr_cnt_vector_orig);
}
