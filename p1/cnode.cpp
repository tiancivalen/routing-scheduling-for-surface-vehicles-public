//by Tianci Zhang

#include "cnode.h"
#include <assert.h>
cnode::cnode()
{
}
cnode::cnode(double x, double y)
{
    m_x = x;
    m_y = y;
}


uint cnode::GetAnotherZoneId(uint in_id)
{
    if(m_nbzone[0] == in_id){
        return m_nbzone[1];
    }
    else{
        assert(m_nbzone[1] == in_id);
        return m_nbzone[0];
    }
}
