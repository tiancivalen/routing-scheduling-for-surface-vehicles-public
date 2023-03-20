//by Tianci Zhang

#include "operation.h"
#include <QDebug>
#include <QFile>
#include <QDir>
#include <QFileInfo>
#include <assert.h>
#include "globalvariable.h"
#include <windows.h>
#include <QTime>
#include <math.h>

//bool g_print =false;

double GetHighPrecisionCurrentTime()
{
        LARGE_INTEGER litmp;
        LONGLONG Qpart1;
        double curtime;
        double dfFreq;
        //
        QueryPerformanceFrequency(&litmp);
        dfFreq = (double)litmp.QuadPart;
        QueryPerformanceCounter(&litmp);
        Qpart1 = litmp.QuadPart;
        curtime = ((double)Qpart1) / dfFreq;
        curtime = curtime*1000;
        return curtime;
}


void ResetTimeWindows(CRegionCollect& rct)
{
    CRegion* prgn;
    twindow* ptw;
    for(uint i=0; i<rct.m_count_region; i++){
        prgn = rct.GetRegion(i);
        if(prgn->m_type == Inter){
            prgn->GetTWVector().clear();
            ptw = new twindow(0, c_timemax);
            prgn->GetTWVector().push_back(ptw);
        }
        else if(prgn->m_type == Line){
            prgn->GetTWVector(0).clear();
            ptw = new twindow(0, c_timemax);
            prgn->GetTWVector(0).push_back(ptw);
            prgn->GetTWVector(1).clear();
            ptw = new twindow(0, c_timemax);
            prgn->GetTWVector(1).push_back(ptw);
        }
    }
}


