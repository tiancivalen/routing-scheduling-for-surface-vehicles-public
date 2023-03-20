//by Tianci Zhang

#ifndef CNODE_H
#define CNODE_H
#include <qglobal.h>

class cnode
{
public:
    cnode();
    cnode(double x, double y);
    uint GetAnotherZoneId(uint in_id);
public:
    uint m_id;
    double m_x;
    double m_y;
    uint m_nbzone[2];
};

#endif // CNODE_H
