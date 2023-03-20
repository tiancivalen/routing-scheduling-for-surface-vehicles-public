#ifndef CTAXIWAYMODEL_H
//by Tianci Zhang

#define CTAXIWAYMODEL_H
#include <qglobal.h>
#include "cregioncollect.h"
#include "cnodecollect.h"
#include <QString>
class CTaxiwayModel
{
public:
    CTaxiwayModel();
    ~CTaxiwayModel();
public:
    void Initialize(uint flag=0);//ORS is used in default (if not 0 ORS is not used, and zones are used exclusively)
private:
    void ConsistentCheck();//check the consistency of input data, helping resolve issues in data parsing
    void PrintInfo();//print statistic info of the taxiway model to file

public:
    QString file_dir;//input data path
    uint m_flag_ors;//flag for ORS usage (0 is used, none 0 is not used)
public:
    CRegionCollect m_rct;
    CNodeCollect m_ndct;
    uchar** matrix_nn_conn;//node-node connectivity
    uchar** matrix_nn_hold;//holdable or not
    uchar** matrix_nn_hold_copy; // copy of matrix_nn_hold
    int** matrix_nn_isTurn;//turn (1: turn, 0: non turn, -1: not connected)
    double** matrix_nn_dis;//node-node trajectory length read from file
    double** matrix_nn_sd;//node-node static shortest path length get by the Floyd algorithm
public:
    static const    double c_infinity = (2^30)-1;
private:
    bool m_bInitialized;
};

#endif // CTAXIWAYMODEL_H
