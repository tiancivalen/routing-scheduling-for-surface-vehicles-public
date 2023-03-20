//by Tianci Zhang

#include "cregioncollect.h"

CRegionCollect::CRegionCollect()
{
    m_count_region = 0;
    m_array_region = 0;
    m_bInitialized = false;
}
CRegionCollect::~CRegionCollect()
{
    if(m_bInitialized){
        for(uint i=0;i<m_count_region;i++){
            delete m_array_region[i];
        }
        delete[] m_array_region;
        m_bInitialized = false;
    }

}


void CRegionCollect::Initialize(uint in_count)
{
    m_count_region = in_count;
    m_array_region = new CRegion*[in_count];
    for(uint i = 0; i < in_count; i++){
        m_array_region[i] = new CRegion;
        m_array_region[i]->m_id = i;
    }
    m_bInitialized = true;
}


CRegion* CRegionCollect::GetRegion(uint rid)
{
    if(rid > m_count_region){
        qDebug("error, region index out of range");
        return 0;
    }
    else{
        return m_array_region[rid];
    }
}
