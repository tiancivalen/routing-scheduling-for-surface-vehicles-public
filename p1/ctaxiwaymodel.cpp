//by Tianci Zhang

#include "ctaxiwaymodel.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
CTaxiwayModel::CTaxiwayModel()
{
    m_bInitialized = false;
    m_flag_ors = 0;//default using ors
}

CTaxiwayModel::~CTaxiwayModel()
{
    if(m_bInitialized){
        for(uint i = 0; i<m_ndct.m_count_node; i++){
            delete[] matrix_nn_conn[i];
            delete[] matrix_nn_dis[i];
            delete[] matrix_nn_sd[i];
            delete[] matrix_nn_hold[i];
            delete[] matrix_nn_hold_copy[i];
            delete[] matrix_nn_isTurn[i];
        }
        delete[] matrix_nn_conn;
        delete[] matrix_nn_dis;
        delete[] matrix_nn_sd;
        delete[] matrix_nn_hold;
        delete[] matrix_nn_hold_copy;
        delete[] matrix_nn_isTurn;
        m_bInitialized = false;
    }
}

void CTaxiwayModel::Initialize(uint flag)
{
    m_flag_ors = flag;
    file_dir = "..\\data_in\\map_Lukou\\DATA\\";
    //file_dir = "..\\data_in\\map_Heathrow\\DATA\\";
    //file_dir = "..\\data_in\\map_Pudong\\DATA\\";
    //----parst zone_id_type.txt----//
    //1. count nonempty and noncomment lines to decide zone number
    QFile file_zone(file_dir+"zone_id_type.txt");
    if(!file_zone.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open zone_id_type.txt";
        return;
    }
    QString line;
    uint line_count = 0;
    while(!file_zone.atEnd()){
        line = file_zone.readLine();
        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        line_count++;
    }
    //2. Initialise the zone collect object
    m_rct.Initialize(line_count);
    //3. Read lines from zone_id_type.txt, and fill in zone object
    //   Note that zone is numbered from 0 regardless of the type
    file_zone.seek(0);
    int zone_id;
    int cap;
    while(!file_zone.atEnd()){
        line = file_zone.readLine();
        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        zone_id = line.section('\t',0,0).toInt();
        if(flag == 0){//if ORS is used
            cap = line.section('\t',2,2).toInt();
        }
        else{//else set capacity to 1
            cap = 1;
        }
        if(line.section('\t',1,1)== "I"){
            m_rct.m_array_region[zone_id]->m_type = Inter;
            m_rct.m_array_region[zone_id]->m_num_capacity = 1;
        }
        else if(line.section('\t',1,1)== "L"){
            m_rct.m_array_region[zone_id]->m_type = Line;
            m_rct.m_array_region[zone_id]->m_num_capacity = cap;
        }
        else if(line.section('\t',1,1)=="A"){
            m_rct.m_array_region[zone_id]->m_type = Buffer;//i.e.,air buffer
        }
        else if(line.section('\t',1,1) == "R"){
            m_rct.m_array_region[zone_id]->m_type = Runway;
        }
        else if(line.section('\t',1,1) == "S"){
            m_rct.m_array_region[zone_id]->m_type = Stand;
        }
    }
    file_zone.close();
    //----parse node_position.txt----//
    //1. count nonempty and noncomment lines to decide node number
    QFile file_node(file_dir+"node_position.txt");
    if(!file_node.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open node_position.txt";
        return;
    }
    line_count = 0;
    while(!file_node.atEnd()){
        line = file_node.readLine();
        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        line_count++;
    }
    //2. Initialise the node collect object
    m_ndct.Initialize(line_count);
    //3. Read node_position.txt and fill in the node object
    int node_id;
    file_node.seek(0);
    while(!file_node.atEnd()){
        line = file_node.readLine();
        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        node_id = line.section('\t',0,0).toInt();
        m_ndct.m_array_node[node_id]->m_x = line.section('\t',1,1).toDouble();
        m_ndct.m_array_node[node_id]->m_y = line.section('\t',2,2).toDouble();
    }
    file_node.close();
    //----parse node_node_distance.txt----//
    //1. create the node connectivity martix, node distance matrix, and node static shortest distance matrix
    uint ndcount = m_ndct.m_count_node;
    matrix_nn_conn = new uchar*[ndcount];
    matrix_nn_dis = new double*[ndcount];
    matrix_nn_sd = new double*[ndcount];
    matrix_nn_hold = new uchar*[ndcount];
    matrix_nn_hold_copy = new uchar*[ndcount];
    matrix_nn_isTurn = new int*[ndcount];
    for(uint i=0; i<ndcount; i++){
        matrix_nn_conn[i] = new uchar[ndcount];
        matrix_nn_dis[i] = new double[ndcount];
        matrix_nn_sd[i] = new double[ndcount];
        matrix_nn_hold[i] = new uchar[ndcount];
        matrix_nn_hold_copy[i] = new uchar[ndcount];
        matrix_nn_isTurn[i] = new int[ndcount];
    }
    for(uint i=0; i<ndcount; i++){
        for(uint j=0; j<ndcount; j++){
            matrix_nn_conn[i][j] = 0;//initialise to nonconnected
            matrix_nn_dis[i][j] = c_infinity;
            matrix_nn_sd[i][j] = c_infinity;
            matrix_nn_hold[i][j] = 1;//holdable
            matrix_nn_hold_copy[i][j] = matrix_nn_hold[i][j];
            matrix_nn_isTurn[i][j] = -1;
        }
    }
    //2. parse node_node_distance.txt and fill in the corresponding matrix elements
    QFile file_nn_dis(file_dir+"node_node_distance.txt");
    if(!file_nn_dis.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open node_node_distance.txt";
        return;
    }
    int nd1, nd2;
    double dis;
    uchar hold;
    uchar isTurn;
    while(!file_nn_dis.atEnd()){
        line = file_nn_dis.readLine();

        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        nd1 = line.section('\t',0,0).toInt();
        nd2 = line.section('\t',1,1).toInt();
        dis = line.section('\t',2,2).toDouble();
        hold = (uchar)line.section('\t',3,3).toInt();
        isTurn = (int)line.section('\t',5,5).toInt();

        matrix_nn_conn[nd1][nd2] = 1;
        matrix_nn_dis[nd1][nd2] = dis;
        matrix_nn_sd[nd1][nd2] = dis;
        matrix_nn_conn[nd2][nd1] = 1;
        matrix_nn_dis[nd2][nd1] = dis;
        matrix_nn_sd[nd2][nd1] = dis;
        matrix_nn_hold[nd1][nd2] = hold;
        matrix_nn_hold[nd2][nd1] = hold;
        matrix_nn_hold_copy[nd1][nd2] = matrix_nn_hold[nd1][nd2];
        matrix_nn_hold_copy[nd2][nd1] = matrix_nn_hold[nd2][nd1];
        matrix_nn_isTurn[nd1][nd2] = isTurn;
        matrix_nn_isTurn[nd2][nd1] = isTurn;
    }
    //3. use Floyd algorithm to fill in matrix_nn_sd
    for(uint k=0;k<ndcount;k++){
        for(uint i=0;i<ndcount;i++){
            for(uint j=0;j<ndcount;j++){
                if(i==j){
                    continue;
                }
                if(matrix_nn_sd[i][k]+matrix_nn_sd[k][j] < matrix_nn_sd[i][j]){
                    matrix_nn_sd[i][j] = matrix_nn_sd[i][k]+matrix_nn_sd[k][j];
                }
            }
        }
    }
    file_nn_dis.close();
    //----parse direction_forbidden.txt----//
    QFile file_forbid(file_dir+"direction_forbidden.txt");
    if(!file_forbid.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open direction_forbidden.txt";
        return;
    }
    uint id_from;
    uint id_to;
    while(!file_forbid.atEnd()){
        line = file_forbid.readLine();
        if(line.at(0)=='#'){
            continue;
        }
        else if(line.isEmpty()){
            continue;
        }
        else{
            id_from = line.section('\t',0,0).toInt();
            id_to = line.section('\t',1,1).toInt();
            matrix_nn_conn[id_from][id_to] = 0;//set to nonconnected
            matrix_nn_dis[id_from][id_to] = c_infinity;
            matrix_nn_sd[id_from][id_to] = c_infinity;
        }
    }
    file_forbid.close();
    //----parse node_zone_zone.txt----//
    QFile file_nzz(file_dir+"node_zone_zone.txt");
    if(!file_nzz.open(QFile::ReadOnly | QFile::Text)){
        qDebug() << "Failed to open node_zone_zone.txt";
        return;
    }
    uint nd_id;
    uint id;
    uint id2;
    while(!file_nzz.atEnd()){
        line = file_nzz.readLine();
        if(line.at(0) == '#')
            continue;
        if(line.isEmpty())
            continue;
        nd_id = line.section('\t',0,0).toInt();
        id = line.section('\t',1,1).toInt();
        id2 = line.section('\t',2,2).toInt();
        //fill zone id to neighbouring zone array of node object
        m_ndct.m_array_node[nd_id]->m_nbzone[0] = id;
        m_ndct.m_array_node[nd_id]->m_nbzone[1] = id2;
        //fill node id to node vector of zone object
        if(id != 1000){
            m_rct.m_array_region[id]->m_vector_node.push_back(nd_id);
            m_rct.m_array_region[id]->m_count_node++;
        }
        if(id2 != 1000){
            m_rct.m_array_region[id2]->m_vector_node.push_back(nd_id);
            m_rct.m_array_region[id2]->m_count_node++;
        }
    }
    file_nzz.close();

    m_bInitialized = true;
    //check the consistency
    ConsistentCheck();
}


void CTaxiwayModel::ConsistentCheck()
{
    CRegion* prgn;
    uint nd;
    uint rg;
    for(uint i=0; i<this->m_rct.m_count_region; i++){
        prgn = m_rct.m_array_region[i];
        if((prgn->m_type == Line)){
            if(prgn->m_count_node != 2){//the total node number of a lane should be 2
                qDebug() << QString("Consistency check error: zone%1, type%2, node number%3").arg(i).arg("Line").arg(prgn->m_count_node);
            }
            else{//a lane is not directly connected to a runway or gate
                for(uint j=0; j<2; j++){
                    nd = prgn->m_vector_node.at(j);
                    rg = m_ndct.m_array_node[nd]->GetAnotherZoneId(i);
                    if(m_rct.m_array_region[rg]->m_type == Runway){
                        qDebug() << QString("Consistency check error: lane%1 is directly connected to runway%2").arg(i).arg(rg);
                    }
                    else if(m_rct.m_array_region[rg]->m_type == Stand){
                        qDebug() << QString("Consistency check error: lane%1 is directly connected to gate%2").arg(i).arg(rg);
                    }
                }

            }
        }
    }
    qDebug() << "Consistency check has finished.";
    PrintInfo();
}


void CTaxiwayModel::PrintInfo()
{
    QFile file(file_dir+"modelinfo.txt");
    if(!file.open(QFile::WriteOnly | QFile::Text)){
        qDebug() << QString("Failed to create file %1").arg("modelinfo.txt");
        return;
    }
    QTextStream ts(&file);
    CRegion* prgn;
    double nbcount = 0;
    double rgcount = 0;
    for(uint i=0; i<m_rct.m_count_region;i++){
        prgn = m_rct.m_array_region[i];
        if(prgn->m_type == Inter){
            rgcount++;
            nbcount += prgn->m_count_node;
        }
    }
    ts << "#average neibor per intersection:\n" << nbcount/rgcount;
    file.close();
}
