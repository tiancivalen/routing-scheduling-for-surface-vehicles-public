//by Tianci Zhang

#include "cnodecollect.h"

CNodeCollect::CNodeCollect()
{
    m_bInitialized = false;
}

CNodeCollect::~CNodeCollect()
{
    if(m_bInitialized){
        for(uint i=0; i<m_count_node; i++){
            delete m_array_node[i];
        }
        delete[] m_array_node;
        m_bInitialized = false;
    }
}

void CNodeCollect::Initialize(uint in_count)
{
    m_count_node = in_count;
    m_array_node = new cnode*[in_count];
    for(uint i=0;i<in_count;i++){
        m_array_node[i] = new cnode;
        m_array_node[i]->m_id = i;
    }
    m_bInitialized = true;
}
