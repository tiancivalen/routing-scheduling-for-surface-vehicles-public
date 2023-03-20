//by Tianci Zhang

#ifndef CREGION_H
#define CREGION_H
#include "common.h"
#include <vector>
extern uint DualToUnary(uint n1, uint n2);

enum RegionType{Undefined = 0, Inter, Line, Buffer, Runway, Stand};//intersection, linesegment, air-buffer, runway and stand

class CRegion
{
public:
    CRegion();
    ~CRegion();
    uchar GetCapacity();
    TWVector& GetTWVector(char direction=0);
    char GetDirection(uint in_enternode);

public:
    //static member
    uint m_id;
    RegionType m_type;
    std::vector<uint> m_vector_node;
    uint  m_count_node;
    uchar m_num_capacity;
    //dynamic member
    TWVector m_forward_ftwv;
    TWVector m_reverse_ftwv;
public:
    OccInfoVector m_occinfo_vector;
};

#endif // CREGION_H
