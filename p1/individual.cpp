#include "individual.h"

Individual::Individual()
{
}

Individual::~Individual(){
    delete m_chromosome;
}
