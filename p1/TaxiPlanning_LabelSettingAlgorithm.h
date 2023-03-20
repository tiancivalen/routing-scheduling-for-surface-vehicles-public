#ifndef TAXIPLANNING_LABELSETTINGALGORITHM_H
#define TAXIPLANNING_LABELSETTINGALGORITHM_H
#include <vector>

#include "common.h"
#include "cregioncollect.h"

class Vehicle;

//Ravizza's label-setting algorithm applied to our taxiway model

/****一些新类和数据结构的集中定义****/
//class CLabel
class CLabel
{
public:
    CLabel(uint in_id, double in_ts, double in_te, CLabel* in_pre);
    uint m_nodeid;                  //Label对应的node的id
    uint m_zoneid;                  //对象到达Label所在节点时所在的区域编号
    double ts;                      //start time of the interval
    double te;                      //end time of the interval
    CLabel* pPreLabel;              //前一个Lable
    double tin_from_pre;            //从前一个Label所在节点进入下一个区域的时刻
    bool Dominate(CLabel* in_pLab); //判断当前Label对象是否dominate in_pLab
};

//class CLableCollect
class CLabelCollect
{
public:
    std::vector<CLabel*> m_H;       //类似于Ravizza文中的H
    std::vector<CLabel*> m_C;       //类似于CloseList

    void Initialize();              //初始化
    CLabel* GetMin();               //从m_H中取出key最小的Label对象的指针
    void Reset();                   //复位到初始化完毕时的状态
};

//class CLSNode
class CLSNode
{
public:
    int m_id;
    std::vector<CLabel*> m_LabelVector;
};

//class CLSNodeCollect
class CLSNodeCollect
{
public:
    uint m_count;
    std::vector<CLSNode*> m_LSNodeVector;
    void Initialize();              //为每个node创建CLSNode对象
    void Reset();                   //复位到初始化完毕时的状态
    CLSNode* GetLSNodebyID(uint in_nodeid);
};

class CPathStep
{
public:
    uint m_nodeid;
    double m_time;
};

typedef  std::vector<CLabel*>        Chain_LS;
typedef  std::vector<CPathStep>      Path_LS;
typedef  std::vector<Path_LS>        v_PathLS;
typedef  std::vector<CLabel*>        v_Label;

/****全局变量****/
extern CLabelCollect g_LabColl;
extern CLSNodeCollect g_LSNodeColl;
extern v_PathLS     g_vector_path_ls;
/****主要函数的集中定义****/
int Plan_LS(Chain_LS& out_path, Vehicle *veh);               //利用Label setting方法为Vehicle规划路径
int  SequencePlan_LS();                                     //对应于Plan_LS的顺序规划
uint GetZoneIDBetweenTwoNodes(uint in_nd1, uint in_nd2);    //根据两个节点的编号确定连接二者的滑行道所在的区域
//GetZoneIDbyTwoNodes这个函数有问题，不再使用
uint GetZoneIDbyTwoNodes(uint in_nd1, uint in_nd2);         //根据两个节点的编号确定连接二者的滑行道所在的区域，进而确定与in_nd2相连的另一区域并返回
void ResetGlobalVariables_LS();
void UpdateTimeWindowVector_LS(Chain_LS &in_path, CRegionCollect &rct);
void PrintTaxiTimeAndDistance_LS(QString in_timefile, QString in_disfile);
void Convert(Chain_LS& in_chain, Path_LS& out_path);        //将chain转换为path
void SavePathToFile_LS(uint &in_id, Path_LS& in_path, QString filename);
void ExtractOccupancyInfo_LS();
void ConflictDetection_LS();
void PrintFinishTime_LS(QString filename);                  //将对象到达目标位置的时刻写入文件
#endif // TAXIPLANNING_LABELSETTINGALGORITHM_H
