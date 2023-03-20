#include "robust_step.h"
#include <QDebug>
#include <QString>
#include <math.h>
#include "globalvariable.h"
#include "TP_robust_globalvariables.h"

bool robust_step::Dominate(robust_step* prs)//该版本与上一版相比放宽了搜索初期的抑制强度，因此某些航空器会找到更多条路径，但求解时间会略有增加。
{
    if(prs->m_bs != m_bs){
        qDebug() << QString("Attention: The base steps are not comparable.");
        return false;
    }
    else{
        double estimator = g_array_h[GetNodeIndex(m_bs)]/g_runway_speed;//对剩余的滑行时间进行的预测

        if((prs->m_cost_time+estimator) > g_ratio_range*(m_cost_time+estimator)
                || fabs(prs->m_cost_time+estimator - g_ratio_range*(m_cost_time+estimator))<1e-3){
            if((prs->m_cost_time+prs->m_cost_delay) > (m_cost_time+m_cost_delay) ||
                    fabs(prs->m_cost_time+prs->m_cost_delay - m_cost_time - m_cost_delay)<1e-3){
                return true;
            }
            else{//表明m_cost_delay特别大
                return false;
            }
        }
        else if(prs->m_cost_time > m_cost_time || fabs(prs->m_cost_time - m_cost_time)<1e-3){//TODO:这里对等号的处理是否完善有待推敲
            if( prs->m_cost_delay > m_cost_delay || fabs(prs->m_cost_delay - m_cost_delay) < 1e-3){//注意：这里表明如果delay相等也会被抑制
            //if(prs->m_cost_delay - m_cost_delay > 1e-3){
                return true;
            }
            else{
                return false;
            }
        }
        else if(prs->m_cost_time < m_cost_time){
            if(g_ratio_range*(m_cost_time+estimator)+m_cost_delay < prs->m_cost_time+estimator+prs->m_cost_delay
                    || fabs(g_ratio_range*(m_cost_time+estimator)+m_cost_delay - prs->m_cost_time - estimator - prs->m_cost_delay)<1e-3){
                return true;
            }
            else{
                return false;
            }
        }
        return false;
    }
}


//bool robust_step::Dominate(robust_step* prs)
//{
//    if(prs->m_bs != m_bs){
//        qDebug() << QString("Attention: The base steps are not comparable.");
//        return false;
//    }
//    else{
//        if(prs->m_cost_time > g_ratio_range*m_cost_time || fabs(prs->m_cost_time - g_ratio_range*m_cost_time)<1e-3){
//            if((prs->m_cost_time+prs->m_cost_delay) > (m_cost_time+m_cost_delay) ||
//                    fabs(prs->m_cost_time+prs->m_cost_delay - m_cost_time - m_cost_delay)<1e-3){
//                return true;
//            }
//            else{//表明m_cost_delay特别大
//                return false;
//            }
//        }
//        else if(prs->m_cost_time > m_cost_time || fabs(prs->m_cost_time - m_cost_time)<1e-3){//TODO:这里对等号的处理是否完善有待推敲
//            if( prs->m_cost_delay > m_cost_delay || fabs(prs->m_cost_delay - m_cost_delay) < 1e-3){
//                return true;
//            }
//            else{
//                return false;
//            }
//        }
//        else if(prs->m_cost_time < m_cost_time){
//            if(g_ratio_range*m_cost_time+m_cost_delay < prs->m_cost_time+prs->m_cost_delay
//                    || fabs(g_ratio_range*m_cost_time+m_cost_delay - prs->m_cost_time - prs->m_cost_delay)<1e-3){
//                return true;
//            }
//            else{
//                return false;
//            }
//        }
//        return false;
//    }
//}

