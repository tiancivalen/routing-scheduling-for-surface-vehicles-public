//by Tianci Zhang

#ifndef CNODECOLLECT_H
#define CNODECOLLECT_H
#include <qglobal.h>
#include "cnode.h"
class CNodeCollect
{
public:
    CNodeCollect();
    ~CNodeCollect();
    void Initialize(uint in_count);
public:
    uint m_count_node;
    cnode** m_array_node;
private:
    bool m_bInitialized;
};

#endif // CNODECOLLECT_H
