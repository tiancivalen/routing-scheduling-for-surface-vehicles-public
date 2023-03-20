#include "ForMilp.h"
#include "globalvariable.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
void GenerateFile(QString in_filename)
{
    if(in_filename.isEmpty()){
        qDebug("error, the nnl file name is empty");
    }
    QFile file(in_filename);
    if(!file.open(QFile::ReadOnly | QFile::Text)){
        qDebug("error, failed to open nnl file");
        return;
    }
    QFile conn_file("conn.txt");
    if(!conn_file.open(QFile::WriteOnly| QFile::Text)){
        qDebug("error,failed to create conn.txt");
        return;
    }
    QFile len_file("len.txt");
    if(!len_file.open(QFile::WriteOnly| QFile::Text)){
        qDebug("error,failed to create len.txt");
        return;
    }

    QString strline;
    QString strtemp;
    uint masternode;
    uint slavenode;
    double lngth;
    bool b;
    //获取总共的节点数
    uint count = 0;
    while(!file.atEnd()){
        strline = file.readLine();
        if(strline.isEmpty()){
            continue;
        }
        else if(strline.at(0) == '#'){
            continue;
        }
        strtemp = strline.section('\t',0,0);
        if(!strtemp.isEmpty()){
            count++;
        }
    }
    //创建二维数组，保存节点-节点连通关系和距离
    int** array_conn = new int*[count];
    for(uint i=0; i<count; i++){
        array_conn[i] = new int[count];
    }
    for(uint i=0;i<count; i++){
        for(uint j=0; j<count; j++){
            array_conn[i][j] = 0;
        }
    }
    double** array_len = new double*[count];
    for(uint i=0; i<count; i++){
        array_len[i] = new double[count];
    }
    for(uint i=0; i<count;i++){
        for(uint j=0; j<count;j++){
            array_len[i][j] = -1;
        }
    }
    //
    file.seek(0);
    while(!file.atEnd()){
        //读取一行数据，取得主节点、次节点及长度信息//
        strline = file.readLine();
        if(strline.isEmpty()){
            continue;
        }
        if(strline.at(0) == '#'){
            continue;
        }
        strtemp = strline.section('\t',0,0);
        if(!strtemp.isEmpty()){
            masternode = (uint)strtemp.toInt(&b);
            if(!b){
                qDebug("error 1 when convert node number from file\n");
            }
        }

        strtemp = strline.section('\t',1,1);
        slavenode = (uint)strtemp.toInt(&b);
        if(!b){
            qDebug("error 2 when convert node number from file\n");
        }

        strtemp = strline.section('\t',2,2);
        lngth = strtemp.toDouble(&b);
        if(!b){
            qDebug("error  when convert length from file\n");
        }
        //将信息填入矩阵对应的位置
        array_conn[masternode][slavenode] = 1;
        array_len[masternode][slavenode] = lngth;

    }
    //将矩阵写入文件
    QTextStream ts(&conn_file);
    for(uint i=0; i<count; i++){
        for(uint j=0; j<count; j++){
            ts << array_conn[i][j] << '\t';
        }
        ts << '\n';
    }
    QTextStream ts2(&len_file);
    for(uint i=0; i<count; i++){
        for(uint j=0; j<count; j++){
            if(array_len[i][j]+1 > 0.1){
                ts2 << array_len[i][j] << '\t';
            }
            //else ts2 << "Infinity" << '\t';
            else ts2 << 0 << '\t';
        }
        ts2 << '\n';
    }
    file.close();
    conn_file.close();
    len_file.close();
    for(uint i=0; i<count; i++){
        delete[] array_conn[i];
        delete[] array_len[i];
    }
    delete[] array_conn;
    delete[] array_len;
    qDebug("files for milp is ready");
}

//void GenerateFile_zonecontrol()
//{
//    GenerateFile_zonecontrol_zoneconnectivity();
//    GenerateFile_zonecontrol_commonnode();
//}

//void GenerateFile_zonecontrol_zoneconnectivity()
//{
//    uint count = rct.m_count_region;
//    int** array_conn = new int*[count];
//    for(uint i=0; i<count; i++){
//        array_conn[i] = new int[count];
//    }
//    for(uint i=0;i<count; i++){
//        for(uint j=0; j<count; j++){
//            array_conn[i][j] = 0;
//        }
//    }

//    CRegion* prgn;
//    uint nbid;
//    for(uint i=0; i<count; i++){
//        prgn = rct.GetRegion(i);
//        for(uint j=0; j < prgn->m_count_neighbor;j++){
//            nbid = prgn->m_array_neighbor[j];
//            array_conn[i][nbid]=1;
//        }
//    }

//    QFile conn_file("conn_zone.txt");
//    if(!conn_file.open(QFile::WriteOnly| QFile::Text)){
//        qDebug("error,failed to create conn.txt");
//        return;
//    }
//    QTextStream ts(&conn_file);
//    for(uint i=0; i<count; i++){
//        for(uint j=0; j<count; j++){
//            ts << array_conn[i][j] << '\t';
//        }
//        ts << '\n';
//    }

//    for(uint i=0; i<count; i++){
//        delete[] array_conn[i];
//    }
//    delete[] array_conn;
//    qDebug() << "files for zone controlled milp is ready";
//}

//void GenerateFile_zonecontrol_commonnode()
//{
//    //allocate memory for the matrix
//    uint count = rct.m_count_region;
//    int** array_comnode = new int*[count];
//    for(uint i=0; i<count; i++){
//        array_comnode[i] = new int[count];
//    }
//    for(uint i=0;i<count; i++){
//        for(uint j=0; j<count; j++){
//            array_comnode[i][j] = -1;
//        }
//    }

//    CRegion* prgn;
//    uint nbid;
//    uint nodeid;
//    for(uint i=0; i<count; i++){
//        prgn = rct.GetRegion(i);
//        for(uint j=0; j < prgn->m_count_neighbor;j++){
//            nbid = prgn->m_array_neighbor[j];
//            nodeid = prgn->GetCommonnode(nbid);
//            array_comnode[i][nbid]=nodeid;
//        }
//    }

//    QFile conn_file("common_node.txt");
//    if(!conn_file.open(QFile::WriteOnly| QFile::Text)){
//        qDebug("error,failed to create conn.txt");
//        return;
//    }
//    QTextStream ts(&conn_file);
//    for(uint i=0; i<count; i++){
//        for(uint j=0; j<count; j++){
//            ts << array_comnode[i][j] << '\t';
//        }
//        ts << '\n';
//    }

//    for(uint i=0; i<count; i++){
//        delete[] array_comnode[i];
//    }
//    delete[] array_comnode;
//    qDebug() << "common node matrix file is generated";
//}

//void Floyd(QString in_filename)
//{
//    if(in_filename.isEmpty()){
//        qDebug("error, the nnl file name is empty");
//    }
//    QFile file(in_filename);
//    if(!file.open(QFile::ReadOnly | QFile::Text)){
//        qDebug("error, failed to open nnl file");
//        return;
//    }
//    QString strline;
//    QString strtemp;
//    uint masternode;
//    uint slavenode;
//    double lngth;
//    bool b;
//    //获取总共的节点数
//    uint count = 0;
//    while(!file.atEnd()){
//        strline = file.readLine();
//        if(strline.isEmpty()){
//            continue;
//        }
//        else if(strline.at(0) == '#'){
//            continue;
//        }
//        strtemp = strline.section('\t',0,0);
//        if(!strtemp.isEmpty()){
//            count++;
//        }
//    }
//    //create the length matrix and initialize
//    double** array_len = new double*[count];
//    for(uint i=0; i<count; i++){
//        array_len[i] = new double[count];
//    }
//    for(uint i=0; i<count;i++){
//        for(uint j=0; j<count;j++){
//            array_len[i][j] = uintmax;
//        }
//    }
//    for(uint i=0;i<count;i++){//对角线元素置为0
//        array_len[i][i]=0;
//    }
//    //read length from the nnl file
//    file.seek(0);
//    while(!file.atEnd()){
//        //读取一行数据，取得主节点、次节点及长度信息//
//        strline = file.readLine();
//        if(strline.isEmpty()){
//            continue;
//        }
//        if(strline.at(0) == '#'){
//            continue;
//        }
//        strtemp = strline.section('\t',0,0);
//        if(!strtemp.isEmpty()){
//            masternode = (uint)strtemp.toInt(&b);
//            if(!b){
//                qDebug("error 1 when convert node number from file\n");
//            }
//        }

//        strtemp = strline.section('\t',1,1);
//        slavenode = (uint)strtemp.toInt(&b);
//        if(!b){
//            qDebug("error 2 when convert node number from file\n");
//        }

//        strtemp = strline.section('\t',2,2);
//        lngth = strtemp.toDouble(&b);
//        if(!b){
//            qDebug("error  when convert length from file\n");
//        }
//        //将信息填入矩阵对应的位置
//        array_len[masternode][slavenode] = lngth;
//    }
//    file.close();
//    //create the shortest distance matrix and intialize
//    double** array_sd = new double*[count];
//    for(uint i=0;i<count;i++){
//        array_sd[i] = new double[count];
//    }
//    for(uint i=0;i<count;i++){
//        for(uint j=0;j<count;j++){
//            array_sd[i][j]=array_len[i][j];
//        }
//    }
//    //Floyd algorithm
//    for(uint k=0;k<count;k++){
//        for(uint i=0;i<count;i++){
//            for(uint j=0;j<count;j++){
//                if(i==j){
//                    continue;
//                }
//                if(array_sd[i][k]+array_sd[k][j] < array_sd[i][j]){
//                    array_sd[i][j] = array_sd[i][k]+array_sd[k][j];
//                }
//            }
//        }
//    }
//    //write shortest distances into file
//    QFile file_out("shortest_distance.txt");
//    if(!file_out.open(QFile::WriteOnly | QFile::Text)){
//        qDebug() << "failed to create shortest_distance.txt";
//    }
//    else{
//        QTextStream ts(&file_out);
//        for(uint i=0;i<count;i++){
//            for(uint j=0;j<count;j++){
//                ts << array_sd[i][j] << '\t';
//            }
//            ts << '\n';
//        }
//        file_out.close();
//    }
//    //release memory
//    for(uint i=0;i<count;i++){
//        delete[] array_len[i];
//        delete[] array_sd[i];
//    }
//    delete[] array_len;
//    delete[] array_sd;
//    qDebug() << "shortest distances between all pairs of nodes are generated.";

//}

//void GenerateFile_RegionType()
//{
//    QFile file("RegionType.txt");
//    if(!file.open(QFile::WriteOnly | QFile::Text)){
//        qDebug() << "Failed to create RegionType.txt";
//        return;
//    }
//    QTextStream ts(&file);
//    CRegion* prgn;
//    for(uint i=0; i<rct.m_count_region; i++){
//        prgn = rct.GetRegion(i);
//        switch(prgn->m_type){
//        case Inter:
//            ts << 1 << '\t';
//            break;
//        case Line:
//            ts << 2 << '\t';
//            break;
//        case Buffer:
//            ts << 3 << '\t';
//            break;
//        case BufferInter:
//            ts << 4 << '\t';
//            break;
//        default:
//            break;
//        }
//    }
//    file.close();
//    qDebug() << "RegionType.txt is ready.";
//}

//void GenerateFile_NextRegion()
//{
//    QFile file("NextRegion.txt");
//    if(!file.open(QFile::WriteOnly | QFile::Text)){
//        qDebug() << "Failed to create RegionType.txt";
//        return;
//    }
//    QTextStream ts(&file);
//    uint region_count = rct.m_count_region;
//    uint **matrix_nregion = new uint*[region_count];
//    for(uint i=0; i<region_count; i++){
//        matrix_nregion[i] = new uint[g_nodecount];
//    }

//    CRegion* prgn;
//    uint nr_id;
//    for(uint i=0; i<region_count; i++){
//        prgn = rct.GetRegion(i);
//        for(uint j=0;j<g_nodecount;j++){
//            if(prgn->GetNeighbor(j,nr_id)){
//                matrix_nregion[i][j] = nr_id;
//            }
//            else{
//                matrix_nregion[i][j]=1000;
//            }
//        }
//    }
//    for(uint i=0; i< region_count; i++){
//        for(uint j=0; j<g_nodecount; j++){
//            ts << matrix_nregion[i][j] << '\t';
//        }
//        ts << '\n';
//    }

//    file.close();
//    for(uint i=0; i<region_count; i++){
//        delete[] matrix_nregion[i];
//    }
//    delete[] matrix_nregion;
//    qDebug() << "NextRegion.txt is ready.";
//}
