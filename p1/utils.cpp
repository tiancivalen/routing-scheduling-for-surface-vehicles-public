//by Tianci Zhang

#include "utils.h"

void TernaryToUnary(uint n1, uint n2, uint n3, quint64& out_bs)
{
    out_bs = n1;
    out_bs = (out_bs << 22);
    out_bs += n2;
    out_bs = (out_bs << 20);
    out_bs += n3;
}

quint64 TernaryToUnary(uint n1, uint n2, uint n3)
{
    quint64 tmp = n1;
    tmp = (tmp << 22);
    tmp += n2;
    tmp = (tmp << 20);
    tmp += n3;
    return tmp;
}
