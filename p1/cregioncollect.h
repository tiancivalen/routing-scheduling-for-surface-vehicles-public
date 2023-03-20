//by Tianci Zhang

#ifndef CREGIONCOLLECT_H
#define CREGIONCOLLECT_H
#include "cregion.h"
class CRegionCollect
{
public:
    CRegionCollect();
    ~CRegionCollect();
    void Initialize(uint in_count);
    CRegion* GetRegion(uint rid);
public:
    uint m_count_region;
    CRegion** m_array_region;
private:
    bool m_bInitialized;
};

#endif // CREGIONCOLLECT_H
