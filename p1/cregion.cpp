//by Tianci Zhang

#include "cregion.h"
#include "twindow.h"
#include "globalvariable.h"
#include <assert.h>
CRegion::CRegion()
{
    m_type = Undefined;
    m_num_capacity = 1;
    m_vector_node.clear();
    m_count_node = 0;
    twindow* ptw = new twindow(0, c_timemax);
    m_forward_ftwv.push_back(ptw);
    twindow* ptwrev = new twindow(0, c_timemax);
    m_reverse_ftwv.push_back(ptwrev);
}
CRegion::~CRegion()
{
    for(uint i=0; i<m_forward_ftwv.size(); i++){
        delete m_forward_ftwv.at(i);
    }
    for(uint i=0; i<m_reverse_ftwv.size();i++){
        delete m_reverse_ftwv.at(i);
    }
    m_forward_ftwv.clear();
    m_reverse_ftwv.clear();
    m_vector_node.clear();
    m_count_node = 0;
    m_occinfo_vector.clear();
}




TWVector& CRegion::GetTWVector(char direction)
{
    if(direction == 0){
        return m_forward_ftwv;
    }
    else{
        return m_reverse_ftwv;
    }

}

char CRegion::GetDirection(uint in_enternode)
{
    if(m_type != Line){
        qDebug("warning, GetAnotherNode() is only meaningful with Line type");
        return 0;
    }
    assert(m_count_node == 2);
    uint ndid = m_vector_node.at(0);
    uint anothernode;
    if(ndid == in_enternode){
        anothernode = m_vector_node.at(1);
    }
    else{
        anothernode = ndid;
    }
    if(in_enternode < anothernode){
        return 0;
    }
    else{
        return 1;
    }
}
uchar CRegion::GetCapacity()
{
    return m_num_capacity;
}
