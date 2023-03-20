//by Tianci Zhang

#include "TP_nowaitingExt.h"
#include "globalvariable.h"
#include "assert.h"
#include <QElapsedTimer>
#include "operation.h"
#include <math.h>
#include <QDebug>
#include <QCoreApplication>
#include <QFile>
#include "TP_robust_globalvariables.h"
#include <algorithm>


extern double g_frequency;


TP_nowaitingExt::TP_nowaitingExt(double weight)
{
    m_weight = weight;
//    m_index = index;
}



bool TP_nowaitingExt::Overlapped(twindow* in_ptw, TWVector& in_twv, uint& out_count, AW_ENTW_table_nowaiting& out_wtt)
{
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    out_count = 0;
    out_wtt.clear();
    twindow entw;//entry time window
    if(twnum == 0){
        return false;
    }
    else{
        for(uint i=0; i<twnum; i++){
            ptwtmp = in_twv[i];
            if(EntryWindow_Intersection(in_ptw, ptwtmp, entw)){
                out_count++;
                out_wtt.insert(AW_ENTW_table_nowaiting::value_type(i, entw));
            }
        }
        return (out_count > 0);
    }
}

bool TP_nowaitingExt::EntryWindow_Intersection(twindow* in_ptw1, twindow* in_ptw2, twindow& out_entw)
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

void TP_nowaitingExt::GetMinHeuristicCostOfOpenlist(nowaiting_step *&out_bs)
{
    out_bs = (nowaiting_step*)openlistexp.at(0);
    uint index = 0;
    nowaiting_step* bs;
    for(uint i=1; i < openlistexp.size();i++){
        bs = (nowaiting_step*)openlistexp.at(i);
        if(out_bs->m_cost+g_array_h[GetNodeIndex(out_bs->m_bs)]/500 - (bs->m_cost+g_array_h[GetNodeIndex(bs->m_bs)]/500) > 1e-3){//200大于实际滑行和起降速度
            out_bs = bs;
            index = i;
        }

    }
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}

void TP_nowaitingExt::GetMinHeuristicCostOfOpenlist_deepfirst(nowaiting_step *&out_bs)
{
    out_bs = (nowaiting_step*)openlistexp.at(0);
    uint index = 0;
    nowaiting_step* bs;
    for(uint i=1; i < openlistexp.size();i++){
        bs = (nowaiting_step*)openlistexp.at(i);
        //breadth-first
        if(g_array_h[GetNodeIndex(out_bs->m_bs)] - g_array_h[GetNodeIndex(bs->m_bs)] > 1e-3){//
            out_bs = bs;
            index = i;
        }

    }
    openlistexp.erase(openlistexp.begin()+index);
    closelist.push_back(out_bs);
}



bool TP_nowaitingExt::ExitWindow_Intersection(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable)
{
    double delta = 5;
    double ratio = 0.1;
    double traverse = in_nnlength/in_speed;

    double tee = in_entw.tstart + traverse;//earliest time of exit
    double tle;//latest time of exit
    if(in_holdable){
        tle = in_paw->tend - delta;
    }
    else{
//        tle = fmin(in_entw.tend+traverse*(1+ratio),in_paw->tend - delta);
        tle = fmin(in_entw.tend+in_nnlength/g_speed_lb,in_paw->tend - delta);
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




bool TP_nowaitingExt::Closed(Basicstep in_bs, NowaitingStepVector& rsv)
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

bool TP_nowaitingExt::Opened(Basicstep in_bs, std::vector<uint> &rsv)
{
    rsv.clear();
    nowaiting_step* pb_s;
    for(uint i=0; i<openlistexp.size();i++){
        pb_s = (nowaiting_step*)openlistexp.at(i);
        if(pb_s->m_bs == in_bs){
            rsv.push_back(i);
        }
    }
    if(rsv.empty()){
        return false;
    }
    else{
        return true;
    }

}






void TP_nowaitingExt::RemoveTimewindow_Line(TWVector& twv, twindow* in_ptw)
{
    twindow* ptw;
    twindow* pnewtw;
    double tstart = in_ptw->tstart;
    double tend = in_ptw->tend;
    double tmp;
    uint index_aw;
    uint i;


    for(i=0; i<twv.size(); i++){
        ptw = twv.at(i);
        if(tstart < ptw->tend){
            index_aw = i;
            break;
        }
    }
    assert(i != twv.size());
    ptw = twv.at(index_aw);

    if(tend < ptw->tstart+1e-3){
        return;
    }
    else if(tend > ptw->tend+1e-3){
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

void TP_nowaitingExt::UpdateOccupation(TWVector& twv, nowaiting_step* pb_s, nowaiting_step* pb_e, uint in_cap)
{
    twindow* ptw;
    uint i;

    for(i=0;i<twv.size();i++){
        ptw = twv.at(i);
        if(pb_s->m_entrytime < ptw->tend){
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
    //
    assert( occv.size()%2 == 0);//otherwise, it's a little complicated
    assert(occv.at(0)->m_action == 1);//
    assert(occv.at(occv.size()-1)->m_action == -1);//

    uint indexstart=0;
    uint indexend=occv.size();
    uint flag_start = 0;
    uint flag_end = 0;
    uint curcap;
    rOccVariable* poc;
    if(occv.at(0)->m_time >= pb_e->m_entrytime){//
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
    else if(occv.at(occv.size()-1)->m_time <= pb_s->m_entrytime){//
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
    else{//
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
                    break;//
                }
            }
        }
        assert(flag_start == 1);
        if(flag_end == 0){//
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
                occv.insert(occv.begin()+indexend+1, newocc2);//
                //inoutv.insert(inoutv.begin()+indexend+1, -1);
                newocc1->m_pair = newocc2;
                newocc1->m_id = g_veh_id;
            }
        }
    }
    assert(occv.size()%2 == 0);

}

void TP_nowaitingExt::RemoveTimewindow_Inter(TWVector& twv, twindow* in_ptw)
{
    twindow* ptw;
    double gapstart;
    double gapend;
    uint i;//used to denote the index of free time window in which current agent occupies the intersection
   // ptw = twv.at(in_awid);//
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


void TP_nowaitingExt::PathFeasibilityCheck()
{
    Vehicle* pveh;
    nowaiting_step* pbs;
    //CRegion* prgn;
    uint ndin;//
    uint ndout;//
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
            if(g_model.matrix_nn_conn[ndin][ndout]==0){
                int xx = 0;
            }
            assert(g_model.matrix_nn_conn[ndin][ndout]==1);
            ndin = ndout;
        }
    }
}



void TP_nowaitingExt::SimutaneousExchangeDetection()
{
    CRegion* prgn;
    occ_info oci_1;
    occ_info oci_2;
    for(uint i=0; i<g_model.m_rct.m_count_region; i++){
        prgn = g_model.m_rct.GetRegion(i);
        if((prgn->m_type == Stand) || (prgn->m_type == Buffer)){
            continue;
        }
        else if(prgn->m_occinfo_vector.size() < 2){
            continue;
        }
        else{
            for(uint j=0; j<prgn->m_occinfo_vector.size()-1; j++){
                oci_1 = prgn->m_occinfo_vector.at(j);
                for(uint k=j+1; k<prgn->m_occinfo_vector.size(); k++){
                    oci_2 = prgn->m_occinfo_vector.at(k);
                    if((oci_1.m_nd_in == oci_2.m_nd_out) &&
                            (fabs(oci_1.m_time_start - oci_2.m_time_end) < 1e-5))
                    {
                        qDebug() << QString("Simultaneous resource exchange detected:(%1,%2) @%3 %4")
                                    .arg(oci_1.m_id).arg(oci_2.m_id)
                                    .arg(i).arg(oci_1.m_time_start);
                    }
                }
            }
        }

    }
    qDebug() << "simutaneous resource exchange detection finished.";
}

void TP_nowaitingExt::ClearOccupancyInfo(CRegionCollect& rct)
{
    CRegion* prgn;
    for(uint i=0; i<rct.m_count_region; i++){
        prgn = rct.GetRegion(i);
        prgn->m_occinfo_vector.clear();
    }
}

void TP_nowaitingExt::HoldDetection()
{
    double ratio = 0.1;

    Vehicle* pveh;
    nowaiting_step* pbs;
    //CRegion* prgn;
    uint ndin;//
    uint ndout;//
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
            uint act = getAction(rgnid,ndin,nextinterid);
            isTurn = g_model.matrix_nn_isTurn[ndin][ndout];
            speed = getSpeed(rgnid,act,isTurn);
//            if(g_model.matrix_nn_conn[ndin][ndout]==0){
//                int xx = 0;
//            }
            if(g_model.matrix_nn_hold[ndin][ndout]==0){
                //assert(tout-tin < length/speed*(1+ratio)+0.1);
//                if(tout-tin >= length/speed*(1+ratio)+0.1){
                if(tout-tin >= length/g_speed_lb+0.1){
                    int xx = 0;
                    qDebug()<< QString("HoldDetection Warning: MTT violated!");
                }
            }
            ndin = ndout;
            tin = tout;
            rgnid = GetRegionIndex(pbs->m_bs);
        }
    }
}

double TP_nowaitingExt::getSpeed(uint rgnid, uint action, const int in_isTurn)//
{
    CRegion* prgn = g_model.m_rct.GetRegion(rgnid);
    if(prgn->m_type == Runway ){
        if(action == TAXI){
            if(in_isTurn!=0)
                return g_speed_turn;
            else
                return g_speed;
        }
        else{//
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

uint TP_nowaitingExt::getAction(uint srgnid, uint sndid, uint ergnid)
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

void TP_nowaitingExt::printPathDetail(Path& rpath, QString fdir)
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
                .arg(1).arg(rs->m_entw.tstart,0,'f',3).arg(rs->m_entw.tend,0,'f',3);//
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



void TP_nowaitingExt::genTBST(Vehicle* pveh)
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

void TP_nowaitingExt::TimeWindowSlimming(double in_cur_time)
{
    CRegion* prgn;
    twindow* ptw;
    uint index = 0;
    for(uint i=0; i<g_model.m_rct.m_count_region; i++){
        prgn = g_model.m_rct.GetRegion(i);
        if(prgn->m_type == Line){//
            TWVector& twv = prgn->GetTWVector(0);//
            //
            for(index=0; index<twv.size(); index++){
                ptw = twv.at(index);
                if(ptw->tend > in_cur_time){
                    if(index > 0){
                        twv.erase(twv.begin(),twv.begin()+index-1);
                    }
                    break;
                }
            }
            TWVector& twv_r = prgn->GetTWVector(1);//
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
        else{//
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



void TP_nowaitingExt::ParsePlanFile(QString planfilename)
{
    g_vehs.clear(); //
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

void TP_nowaitingExt::ResetGlobalVariables()
{
    for(uint i=0; i<closelist.size();i++){
        delete closelist.at(i);
    }
//    closelist.clear();
    Closelist().swap(closelist);
    assert(closelist.empty());
    for(uint i=0; i<openlistexp.size(); i++){
        delete openlistexp.at(i);
    }
//    openlistexp.clear();
    OpenlistForExpand().swap(openlistexp);
    assert(openlistexp.empty());
    for(uint i=0; i<g_tmp_basestepvector.size(); i++){
        delete g_tmp_basestepvector.at(i);
    }
//    g_tmp_basestepvector.clear();
    std::vector<base_step*>().swap(g_tmp_basestepvector);
    assert(g_tmp_basestepvector.empty());

    g_iterationcount = 0;
    g_accesscount = 0;
}

void TP_nowaitingExt::Heuristics(uint in_target_node)
{
    if(g_config.flag_heuristic == sd){
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            g_array_h[i] = g_model.matrix_nn_sd[i][in_target_node];//
        }
    }
    else if(g_config.flag_heuristic == euclidean){
        cnode* ndtarget;
        cnode* pnd;
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            ndtarget = g_model.m_ndct.m_array_node[in_target_node];
            pnd = g_model.m_ndct.m_array_node[i];
            g_array_h[i] = StraightLineDistance(pnd, ndtarget);//
        }
    }
    else{//
        for(uint i=0; i<g_model.m_ndct.m_count_node; i++){
            g_array_h[i] = 0;
        }
    }

}

quint64 TP_nowaitingExt::TernaryToUnary(uint n1, uint n2, uint n3)
{
    quint64 tmp = n1;
    tmp = (tmp << 22);
    tmp += n2;
    tmp = (tmp << 20);
    tmp += n3;
    return tmp;
}

void TP_nowaitingExt::TernaryToUnary(uint n1, uint n2, uint n3, quint64& out_bs)
{
    out_bs = n1;
    out_bs = (out_bs << 22);
    out_bs += n2;
    out_bs = (out_bs << 20);
    out_bs += n3;
}

uint TP_nowaitingExt::GetRegionIndex(Basicstep in_bs)
{
    uint rindex = (uint)(in_bs >> 42);
    return rindex;
}

uint TP_nowaitingExt::GetNodeIndex(Basicstep in_bs)
{
    uint nindex = (uint)(in_bs >> 20);
    nindex = nindex & uint(0x3fffff);
    return nindex;

}

uint TP_nowaitingExt::GetAWindowIndex(Basicstep in_bs)
{
    uint windex = (uint)in_bs;
    windex = windex & uint(0xfffff);
    return windex;
}

double TP_nowaitingExt::StraightLineDistance(cnode* ndfrom, cnode* ndto)
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

double TP_nowaitingExt::rad(double degree)
{
    return degree * PI / 180.0;
}

void TP_nowaitingExt::updateCost(nowaiting_step* in_pns)
{
    nowaiting_step* pns = in_pns;
    if ((pns->m_prestep == NULL || pns->m_length < 1e-3 || pns->m_speed < 1e-3) && (pns->m_startstep!=pns)){
        qDebug()<<QString("The nowaiting_step object is invalid. Please check.");
    }
    else{
//        nowaiting_step* pprens = (nowaiting_step*)pns->m_prestep;
////        pns->m_cost = m_weight*pow(pns->m_entw.tstart - pprens->m_entw.tstart - pns->m_length/pns->m_speed, m_index)+ (pns->m_entw.tstart - pprens->m_entw.tstart) + pprens->m_cost;
////        pns->m_cost = m_weight * pns->m_length/pns->m_speed + (pns->m_entw.tstart - pprens->m_entw.tstart) + pprens->m_cost;
//        nowaiting_step* pstartns = pns->m_startstep;
//        assert(pns->m_entw.tstart-pstartns->m_entw.tstart>-1e-3);
//        pns->m_cost = m_weight * (pns->m_entw.tstart-pstartns->m_entw.tstart) + (pns->m_entw.tstart - g_start_time);

        nowaiting_step* pprens = (nowaiting_step*)pns->m_prestep;
        nowaiting_step* pstartns = pns->m_startstep;
        assert(pns->m_entw.tstart-pstartns->m_entw.tstart>-1e-3);
        double cost = pns->m_entw.tstart - g_start_time;
        double at = pns->m_entw.tstart;
        double atpre;
        while(pns != pstartns){
            if(pns->m_speed == g_runway_speed){
                atpre = fmin(pprens->m_entw.tend,at-50);
            }
            else{
                atpre = fmin(pprens->m_entw.tend,at-pns->m_length/pns->m_speed);
            }

            cost += m_weight * (at-atpre);
            pns = pprens;
            pprens = (nowaiting_step*)pns->m_prestep;
            at = atpre;
        }
        in_pns->m_cost = cost;

    }
}






bool TP_nowaitingExt::EntryWindow_Line(TWVector& in_twv, twindow* in_ew, twindow &out_entw, uint& out_awid)
{
    uint twnum = in_twv.size();
    twindow* ptwtmp;
    for(uint i=0; i<twnum; i++){
        ptwtmp = in_twv.at(i);
        if(ptwtmp->tend > in_ew->tstart){
            double tstart = fmax(ptwtmp->tstart,in_ew->tstart);
            double tend = fmin(ptwtmp->tend,in_ew->tend);;
            if(tend > tstart){
                out_awid = i;
                out_entw.tstart = tstart;
                out_entw.tend = tend;
                return true;
            }
        }
    }
    return false;
}

bool TP_nowaitingExt::ExitWindow_Line(twindow& in_entw, double in_nnlength, twindow* in_paw, twindow* out_pew, double in_speed, uchar in_holdable)
{
    bool r= ExitWindow_Intersection(in_entw,in_nnlength,in_paw,out_pew,in_speed,in_holdable);
    return r;
}




void TP_nowaitingExt::printPathDetail(Path& rpath, QString fdir, QString fname)
{
    QFile file(QString("%1\\%2").arg(fdir).arg(fname));
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
        //time | node | region | delay | holdable? |[entw.tstart,entw.tend] | ref speed (from the pre node to the node) | length (from the pre node to the node) | prestep exit tw start | prestep exit tw end
        QString strline = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11").arg(rs->m_entrytime,0,'f',3)
                .arg(GetNodeIndex(rs->m_bs)).arg(GetRegionIndex(rs->m_bs)).arg(rs->m_entrytime - g_pveh->m_start_time)
                .arg(1).arg(rs->m_entw.tstart,0,'f',3).arg(rs->m_entw.tend,0,'f',3)
                .arg(0).arg(0).arg(rs->m_preExitTW.tstart,0,'f',3).arg(rs->m_preExitTW.tend,0,'f',3);//
        ts << strline << '\n';
        nowaiting_step* rs_next;
        for(uint i=1;i<rpath.size();i++){
            rs_next = (nowaiting_step*)rpath.at(i);
            double delay = rs_next->m_entrytime - rs->m_entrytime - rs_next->m_length/rs_next->m_speed;
            uint holdable = g_model.matrix_nn_hold_copy[GetNodeIndex(rs->m_bs)][GetNodeIndex(rs_next->m_bs)];
            //time | node | region| holdable? | [entw.tstart,entw.tend] | ref speed (from the pre node to the node) | length (from the pre node to the node)
            strline = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11").arg(rs_next->m_entrytime,0,'f',3)
                    .arg(GetNodeIndex(rs_next->m_bs)).arg(GetRegionIndex(rs_next->m_bs)).arg(delay,0,'f',3)
                    .arg(holdable).arg(rs_next->m_entw.tstart,0,'f',3).arg(rs_next->m_entw.tend,0,'f',3)
                    .arg(rs_next->m_speed).arg(rs_next->m_length,0,'f',3)
                    .arg(rs_next->m_preExitTW.tstart,0,'f',3).arg(rs_next->m_preExitTW.tend,0,'f',3);
            ts << strline << '\n';

            rs = rs_next;
        }
        file.close();
    }
}


void TP_nowaitingExt::Expand_loopless_homogeneous(nowaiting_step *in_bs)
{
    uint currid = GetRegionIndex(in_bs->m_bs);//current region id
    uint curnodeid = GetNodeIndex(in_bs->m_bs);//current node id
//    if(curnodeid==5&& currid == 3&& g_veh_id == 128){
//        int x = 0;
//    }
//    if(g_veh_id > 128){
//        int x = 0;
//    }
    uint curawid = GetAWindowIndex(in_bs->m_bs);//current free time window id
//    if (g_veh_id == 1){
//        qDebug() << QString("region %1, node %2, entw [%3, %4], cost %5").arg(currid).arg(curnodeid).arg(in_bs->m_entw.tstart).arg(in_bs->m_entw.tend)
//                    .arg(in_bs->m_cost);

//    }

    CRegion* prgn = g_model.m_rct.GetRegion(currid);//pointer to current region
    assert(prgn->m_type != Line);//
    if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
        return;
    }//
    assert((prgn->m_type == Inter) || (prgn->m_type == Runway));

    double avg_speed;
    twindow* paw;
    paw = prgn->GetTWVector().at(curawid);
    CRegion* pnb;// pointer to neighbor region
    double nnlength;//length to the neighbor node
    twindow* pew = new twindow;
    uint ndcount = prgn->m_count_node;//number of nodes contained in current region
    uint nd_id;//node id
    uint neighbor_zone_id;
    //char dir_nb;
    twindow* pnbaw = 0;
    Basicstep curbs;
    twindow entw;
    cnode* pnode ;
    g_accesscount += ndcount-1;
    //double ioa_first;
    //double ioa_second;
    uint act;//
    int isTurn;
    nowaiting_step* bs_pre;
    nowaiting_step* b_s_lane;
    bool flag_lane = false;
    bool flag_reachable = true;

    for(uint i=0; i<ndcount; i++){
        nd_id = prgn->m_vector_node.at(i);
        pnode = g_model.m_ndct.m_array_node[nd_id];
        if(nd_id == curnodeid){
            continue;
        }
//        if(nd_id == 364 && g_veh_id == 128){
//            int x=0;
//        }
        neighbor_zone_id = pnode->GetAnotherZoneId(currid);
        // loop checking
        uint loopflag = 0;
        nowaiting_step* pnwstmp  = (nowaiting_step*)in_bs->m_prestep;
        uint rgntmp = GetRegionIndex(pnwstmp->m_bs);
        CRegion* prgntmp = g_model.m_rct.GetRegion(rgntmp);
        while(rgntmp != g_start_region){
            if((rgntmp == neighbor_zone_id) && (prgntmp->m_type != Runway)){
                loopflag = 1;
                break;
            }
            pnwstmp = (nowaiting_step*)pnwstmp->m_prestep;
            rgntmp = GetRegionIndex(pnwstmp->m_bs);
            prgntmp = g_model.m_rct.GetRegion(rgntmp);
        }
        if(loopflag == 1){
            continue;
        }
        else{
            bs_pre = in_bs;//initialize current bs as the input bs for expansion

            if(neighbor_zone_id == 1000){
                continue;
            }
            pnb = g_model.m_rct.GetRegion(neighbor_zone_id);//pointer to the neighbor region
            if(g_model.matrix_nn_conn[curnodeid][nd_id] == 0 ){
                continue;
            }
            nnlength = g_model.matrix_nn_dis[curnodeid][nd_id];//length between current node and the neighbor node
            uchar holdable = g_model.matrix_nn_hold[curnodeid][nd_id];
            act = getAction(currid,curnodeid,neighbor_zone_id);
            isTurn = g_model.matrix_nn_isTurn[curnodeid][nd_id];
            avg_speed = getSpeed(currid,act,isTurn);
            //ioa_first = getIOA(in_bs->m_IOA,nnlength,avg_speed,k);



            if(ExitWindow_Intersection(in_bs->m_entw, nnlength, paw, pew, avg_speed,holdable)){
                /*if current neighbor is a lane*/
                if(pnb->m_type == Line){
//                    if(g_veh_id == 362 && neighbor_zone_id == 164){
//                        int xx = 0;
//                    }
                    uint next_node_id;
                    uint next_rgn_id;
                    flag_lane = true;
                    while(flag_lane){
                        if(pnb->m_vector_node.at(0) == nd_id){
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
                        //dir_nb = pnb->GetDirection(nd_id);
                        double preExitts = pew->tstart;
                        double preExitte = pew->tend;

                        if(!EntryWindow_Line(pnb->GetTWVector(),pew,entw,awid_line)){
                            flag_reachable = false;
                            break;//if the neighbor lane is not accessible, continue
                        }
                        else{
                            pnbaw = pnb->GetTWVector().at(awid_line);
                            assert(pnb->m_count_node == 2);

                            double nnlength_pre = nnlength;
                            double avg_speed_pre = avg_speed;
                            nnlength = g_model.matrix_nn_dis[nd_id][next_node_id];//length to the other node of the lane
                            holdable = g_model.matrix_nn_hold[nd_id][next_node_id];
                            act = getAction(neighbor_zone_id,nd_id,next_rgn_id);
                            isTurn = g_model.matrix_nn_isTurn[nd_id][next_node_id];
                            avg_speed = getSpeed(neighbor_zone_id,act,isTurn);
                            //ioa_second = getIOA(ioa_first,nnlength,avg_speed,k);
                            if(!ExitWindow_Line(entw, nnlength, pnbaw, pew, avg_speed, holdable)){
                                flag_reachable = false;//if the lane is not exit-able, continue
                                break;
                            }
                            else{
                                g_accessibleTNcount++;
                                TernaryToUnary(neighbor_zone_id, nd_id, awid_line, curbs);//update the curbs
                                b_s_lane = new nowaiting_step(curbs, entw);
                                b_s_lane->m_length = nnlength_pre;
                                b_s_lane->m_speed = avg_speed_pre;
                                b_s_lane->m_prestep = bs_pre;
                                b_s_lane->m_preExitTW.tstart = preExitts;
                                b_s_lane->m_preExitTW.tend = preExitte;
                                b_s_lane->m_startstep = bs_pre->m_startstep;
                                updateCost(b_s_lane);
                                g_tmp_basestepvector.push_back(b_s_lane);

                                bs_pre = b_s_lane;//
                                nd_id = next_node_id;
                                neighbor_zone_id = next_rgn_id;

                                pnb = g_model.m_rct.GetRegion(neighbor_zone_id);
                                if(pnb->m_type != Line){
                                    flag_lane = false;
                                }
                            }
                        }
                    }

                    if(flag_reachable){
                        // loop checking
                        uint loopflag = 0;
                        nowaiting_step* pnwstmp  = (nowaiting_step*)in_bs->m_prestep;
                        uint rgntmp = GetRegionIndex(pnwstmp->m_bs);
                        CRegion* prgntmp = g_model.m_rct.GetRegion(rgntmp);
                        while(rgntmp != g_start_region){
                            if((rgntmp == next_rgn_id) && (prgntmp->m_type != Runway)){
                                loopflag = 1;
                                break;
                            }
                            pnwstmp = (nowaiting_step*)pnwstmp->m_prestep;
                            rgntmp = GetRegionIndex(pnwstmp->m_bs);
                            prgntmp = g_model.m_rct.GetRegion(rgntmp);
                        }
                        if(loopflag == 1){
                            continue;
                        }


                        uint paircount = 0;
                        AW_ENTW_table_nowaiting wtt;
                        if(!Overlapped(pew, pnb->GetTWVector(), paircount, wtt)){
                            continue;
                        }
                        else{
                            nowaiting_step* b_s_tmp;
                            Basicstep bs;
                            uint awid;
                            twindow entrytimewindow;
                            AW_ENTW_table_nowaiting::iterator itr;
                            for(itr=wtt.begin();itr!=wtt.end();itr++){
                                g_accessibleTNcount++;
                                awid = itr->first;
                                entrytimewindow = itr->second;
                                TernaryToUnary(next_rgn_id, next_node_id, awid, bs);
                                NowaitingStepVector rsvec;
                                bool flag = false;

                                b_s_tmp = new nowaiting_step(bs,entrytimewindow);
                                b_s_tmp->m_length = nnlength;
                                b_s_tmp->m_speed = avg_speed;
                                b_s_tmp->m_prestep = bs_pre;
                                b_s_tmp->m_preExitTW.tstart = pew->tstart;
                                b_s_tmp->m_preExitTW.tend = pew->tend;
                                b_s_tmp->m_startstep = bs_pre->m_startstep;
                                updateCost(b_s_tmp);

                                if(Closed(bs,rsvec)){
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
                                assert(flag == false);
                                std::vector<uint> rsvec_open;
                                if(Opened(bs, rsvec_open)){
                                    //b_s_tmp = new robust_step(bs,entertime,ioa_second,cost1_time+cost2_time,cost1_delay+cost2_delay);
                                    assert(!rsvec_open.empty());
                                    uint count = 0;
                                    uint n = rsvec_open.size();
                                    for(uint i = 0; i<n; i++){
                                        nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsvec_open.at(i)-count);
                                        assert(prs->m_bs == bs);
                                        if(prs->Dominate(b_s_tmp)){
                                            flag = true;
                                            break;
                                        }
                                        else if(b_s_tmp->Dominate(prs)){
                                            openlistexp.erase(openlistexp.begin()+rsvec_open.at(i)-count);
                                            delete prs;
                                            count++;
                                        }
                                    }
                                    if(flag){
                                        delete b_s_tmp;
                                        continue;//
                                    }
                                }

                                assert(flag == false);

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

                    uint paircount = 0;
                    AW_ENTW_table_nowaiting wtt;
                    if(!Overlapped(pew,pnb->GetTWVector(), paircount, wtt)){
                        continue;
                    }
                    else{
                        nowaiting_step* b_s_tmp;
                        Basicstep bs;
                        uint awid;
                        twindow entrytimewindow;
                        AW_ENTW_table_nowaiting::iterator itr;
                        for(itr=wtt.begin();itr!=wtt.end();itr++){
                            g_accessibleTNcount++;
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
                            b_s_tmp->m_prestep = in_bs;
                            b_s_tmp->m_preExitTW.tstart = pew->tstart;
                            b_s_tmp->m_preExitTW.tend = pew->tend;
                            b_s_tmp->m_startstep = in_bs->m_startstep;
                            updateCost(b_s_tmp);

                            bool flag = false;
                            if(Closed(bs,rsv)){
                                assert(!rsv.empty());
                                uint n = rsv.size();
                                for(uint i=0; i<n; i++){
                                    nowaiting_step* prs = (nowaiting_step*)rsv.at(i);
                                    assert(!b_s_tmp->Dominate(prs));//now dominance is based on the cost!
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
                            if(Opened(bs, rsv_open)){
                                uint count = 0;
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
                                        openlistexp.erase(openlistexp.begin()+rsv_open.at(i)-count);
                                        delete prs;
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

                            //uint xx = openlistexp.size();
                            openlistexp.push_back(b_s_tmp);
                            //xx = openlistexp.size();
                        }
                    }
                }
                /*else if current neighbor is a stand or a buffer*/
                else{
//                    if(g_veh_id == 1 && neighbor_zone_id == 418){
//                        int xx = 0;
//                    }
                    g_accessibleTNcount++;
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
                    b_s_tmp->m_prestep = in_bs;
                    b_s_tmp->m_preExitTW.tstart = pew->tstart;
                    b_s_tmp->m_preExitTW.tend = pew->tend;
                    b_s_tmp->m_startstep = in_bs->m_startstep;
                    updateCost(b_s_tmp);

                    if(Closed(bs,rsv)){
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
                    if(Opened(bs, rsidv)){
                        uint count = 0;
                        assert(!rsidv.empty());
                        uint n = rsidv.size();
                        for(uint i=0; i<n; i++){
                            nowaiting_step* prs = (nowaiting_step*)openlistexp.at(rsidv.at(i)-count);
                            assert(prs->m_bs == bs);//
                            if(prs->Dominate(b_s_tmp)){
                                flag = true;
                                break;
                            }
                            else if(b_s_tmp->Dominate(prs)){
                                openlistexp.erase(openlistexp.begin()+rsidv.at(i)-count);
                                delete prs;
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

                    openlistexp.push_back(b_s_tmp);
                }
            }
        }
    }
    delete pew;
}


int TP_nowaitingExt::Plan_homogeneous(Vehicle *veh)
{
    g_iterationcount = 0;
    g_accessibleTNcount = 0;

    double exp_time = 0;
    double t1, t2;
    t1 = GetHighPrecisionCurrentTime();
//    if(veh->m_id == 12){
//        int xx = 0;
//    }
    g_start_region = veh->m_start_region;
    g_start_node = veh->m_start_node;
    g_start_time = veh->m_start_time;
    g_end_region = veh->m_end_region;
    g_end_node = veh->m_end_node;

    // --------------------
    Heuristics(g_end_node);
    // --------------------


    uint first_taxiway_region = g_model.m_ndct.m_array_node[g_start_node]
            ->GetAnotherZoneId(g_start_region);//

    Basicstep bs;
    twindow entw;//entry time window
    //double ioa = g_ioa;//length of the initial IOA
    //double k = 0.1;//ioa/travel time=0.1
    uint rgnid;
    uint ndid;
    uint awid;
    ndid = g_start_node;
    rgnid = first_taxiway_region;

    //twindow ew(ioaStart(ioa,g_start_time), c_timemax);//initialize the exit window from the start zone (notice: the conflict in the start zone is deliberately not considered for the current experiment)
    twindow ew;
    switch(g_WTSType){
        // ---for WTS Type 1---
    case 1:
        ew.tstart=g_start_time;
        ew.tend = g_start_time+0.1;
        break;

        // ---for WTS Tpye 2---
    case 2:
        double maxWTS;
        if(veh->m_type == ARRIV){
            maxWTS = 5*60;// 5 minutes waiting time limit at the starting position for arrival
        }
        else{
            assert(veh->m_type == DEPAR);
            maxWTS=30*60;// 30 minutes waiting time limit at the starting position for departure
        }
        ew.tstart=g_start_time;
        ew.tend = g_start_time+maxWTS;
        break;

        // ---for WTS Type 3---
    case 3:
        double delta = 5;
        ew.tstart=g_start_time;
        ew.tend = c_timemax-delta;
        break;
    }
    CRegion* prgn = g_model.m_rct.GetRegion(first_taxiway_region);//get the pointer of the next zone to be visited, i.e., the first taxiway zone
    /*tackle the first taxiway zone*/
    if((prgn->m_type == Inter) || (prgn->m_type == Runway)){//if the first taxiway zone is an intersection or a runway
        AW_ENTW_table_nowaiting wtt;
        uint num;
        if(!Overlapped(&ew,prgn->GetTWVector(),num,wtt)){//
            QString info = QString("error, the entry time window of target %1 is not properly set").arg(veh->m_id);
            qDebug(info.toAscii());
            return 1;
        }
        else{
            AW_ENTW_table_nowaiting::iterator itr;
            for(itr=wtt.begin();itr!=wtt.end();itr++){
                awid = itr->first;
                entw = itr->second;//
                entw.tend = fmin(entw.tstart+c_waitmax,entw.tend);
                bs = TernaryToUnary(rgnid, ndid, awid);
                nowaiting_step* b_s = new nowaiting_step(bs, entw);
                g_accessibleTNcount++;
                openlistexp.push_back(b_s);
                nowaiting_step* b_s_pre = new nowaiting_step(TernaryToUnary(g_start_region,0,0),twindow(0,0));
                b_s->m_prestep = b_s_pre;
                b_s->m_preExitTW.tstart = ew.tstart;
                b_s->m_preExitTW.tend = ew.tend;

                b_s->m_startstep = b_s;
                updateCost(b_s);
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

    double timeLimit = 3600;//10 seconds limit

    while(!openlistexp.empty()){

        //--- Begin time limit check -----
        t2 = GetHighPrecisionCurrentTime();
        if((t2 - t1)/1000 > timeLimit){
            return 2;
        }
        //--- End time limit check -----

        g_iterationcount++;

        uint n = openlistexp.size();

        GetMinHeuristicCostOfOpenlist(pb_s);//get least cost bs from open

        bs = pb_s->m_bs;
        curregion = GetRegionIndex(bs);
        curnode = GetNodeIndex(bs);
//        if(g_veh_id == 171){
//            int xx = 0;
//        }

        if((curregion == g_end_region) && (curnode == g_end_node)){//path found

//            if(g_veh_id == 128){
//                int xx = 0;
//            }

            Path path;
            path.push_back(pb_s);
            do{
                pb_s = (nowaiting_step*)pb_s->m_prestep;
                path.push_back(pb_s);
                bs = pb_s->m_bs;
            }while((GetRegionIndex(bs) != first_taxiway_region) || (GetNodeIndex(bs) != g_start_node));
            uint n = path.size();
            for(uint i=0; i<n;i++){
                nowaiting_step* pns = new nowaiting_step((nowaiting_step*)path.at(n - 1 - i));
                veh->m_path.push_back(pns);
            }
            //veh->m_SinglePathLenVector.push_back(n);

            status =  0;//denote path found

            break;

        }
        else{

//            Expand_homogeneous(pb_s);//preregion is used to prevent doubling back
            Expand_loopless_homogeneous(pb_s);
        }

    }

    return status;
}

int TP_nowaitingExt::SequencePlan_homogeneous()
{
    int returnvalue;
    QString fdir = QString("..\\output");
    QFile file(QString("%1\\ComputationTime.txt").arg(fdir));
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug()<< "Failed to create the computational time recording file.";
        return -1;
    }
    QTextStream textstm(&file);

    g_array_h = new double[g_model.m_ndct.m_count_node];
    g_priority = 1;
    Vehicle* veh;
    uint num = g_vehs.size();
    g_plan_num = 1000;////
    num = (uint)fmin(num,g_plan_num);
    for(uint i=0; i<num; i++){
//        if((i+1)%10==0){
//            textstm << "\n";
//        }

        veh = g_vehs.at(i);
        assert(veh->m_id == i+1);
        g_veh_id = veh->m_id;//id ready
        g_pveh = veh;
//        if(g_veh_id == 6){
//            int x = 0;
//        }

//        //------------------------------------------
//        if(g_priority%50 == 0){
//            TimeWindowSlimming(veh->m_start_time);
//        }
//        //------------------------------------------



        double t1 = GetHighPrecisionCurrentTime();
        returnvalue = Plan_homogeneous(veh);
        double t2 = GetHighPrecisionCurrentTime();
        textstm << QString("%1\n").arg(t2-t1,0,'f',1);
        if(returnvalue == 0){

            genTBST(veh);
            ctvector.push_back(t2-t1);//calclation time recoding
            itr_cnt_vector.push_back(g_iterationcount);
            UpdateTimeWindowVector_homogeneous(veh->m_path,g_model.m_rct);
            printPathDetail(veh->m_path,fdir);
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
            itr_cnt_vector.push_back(-1);
            qDebug()<<QString("failed to find feasible path for agent %1, because the Openlist is empty").arg(veh->m_id);
        }

        //itr_cnt_vector.push_back(g_iterationcount);
        ResetGlobalVariables();
        g_priority++;//denote agent's planning priority

        // ---------------------
//        QElapsedTimer t;
//        t.start();
//        while(t.elapsed()<1){
//            QCoreApplication::processEvents();
//        }
        // -------------------------------------------------

    }
    //
    PathFeasibilityCheck();
    ConflictDetection_homogeneous();
    HoldDetection();
    ResetTimeWindows(g_model.m_rct);
    delete[] g_array_h;
    qDebug() << QString("------------Planning completed-----------");

    file.close();

    return 0;
}

void TP_nowaitingExt::Experiment_homogeneous()
{
    ParsePlanFile(g_model.file_dir+"sequenceplan.txt");
    SequencePlan_homogeneous();
}

void TP_nowaitingExt::UpdateTimeWindowVector_homogeneous(Path &in_path, CRegionCollect &rct)
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
        act =  getAction(rngid,ndid,GetRegionIndex(pb_s2->m_bs));
        isTurn = g_model.matrix_nn_isTurn[ndid][GetNodeIndex(pb_s2->m_bs)];
        avg_speed = getSpeed(rngid,act,isTurn);

        tw.tstart = pb_s1->m_entrytime;
        tw.tend = pb_s2->m_entrytime+delta;

        prgn = rct.GetRegion(rngid);

        RemoveTimewindow_Inter(prgn->GetTWVector(), &tw);

    }
}

void TP_nowaitingExt::ConflictDetection_homogeneous()
{
    ExtractOccupancyInfo_homogeneous();
    CheckOccupancyInfo_homogeneous(g_model.m_rct);
    SimutaneousExchangeDetection();//
    ClearOccupancyInfo(g_model.m_rct);//
    qDebug() << "conflict detection finished.";
}

void TP_nowaitingExt::ExtractOccupancyInfo_homogeneous()
{
    double t1;
    double t2;
    uint zone1;
    uint zone2;
    uint ndin;//
    uint ndout;//
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

            prgn->m_occinfo_vector.push_back(occinfo);
            zone1 = zone2;
            t1 = pbs->m_entrytime;
            ndin = ndout;
//            if(fabs(t1-76396.5)<1e-1){
//                int x=0;
//            }
        }
    }
}

void TP_nowaitingExt::CheckOccupancyInfo_homogeneous(CRegionCollect& rct)
{
    CRegion* prgn;
    int num = 0;
    occ_info occif;
    occ_info occifnext;
    QString str;
    bool flag = false;
    for(uint i=0; i<rct.m_count_region;i++){
        prgn = rct.GetRegion(i);
        if((prgn->m_type == Buffer) || (prgn->m_type == Stand)){
            continue;
        }
        else{
            num = prgn->m_occinfo_vector.size();
            if(num < 2){
                continue;
            }
            else{
                for(int j=0; j<num-1;j++){

                    occif = prgn->m_occinfo_vector.at(j);
                    for(int k=j+1; k<num; k++){
                        occifnext = prgn->m_occinfo_vector.at(k);
                        if(!((occif.m_time_end - occifnext.m_time_start < 1e-3)
                             ||(occifnext.m_time_end - occif.m_time_start)<1e-3)){
                            str = QString("Conflict detected at Section %1: %2, %3").arg(i).arg(occif.m_id).arg(occifnext.m_id);
                            qDebug()<<str;
                            flag = true;
                            break;
                        }
                    }
                    if(flag){
                        flag = false;
                        break;
                    }
                }
            }

        }
    }
}

