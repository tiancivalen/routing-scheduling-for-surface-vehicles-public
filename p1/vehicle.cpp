//by Tianci Zhang

#include "vehicle.h"

Vehicle::Vehicle()
{
    m_type = Unknown;
}

Vehicle::~Vehicle()
{
//    for(uint i=0; i<m_path.size(); i++){
//        delete m_path.at(i);
//    }
    m_path.clear();
//    for(uint i=0; i<m_rPath.size(); i++){
//        delete m_rPath.at(i);
//    }
    m_rPath.clear();
//    for(uint i=0;i<m_rPath_all.size();i++){
//        delete m_rPath_all.at(i);
//    }
    m_rPath_all.clear();
    m_SinglePathLenVector.clear();
}
