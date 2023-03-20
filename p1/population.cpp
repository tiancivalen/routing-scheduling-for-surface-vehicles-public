#include "population.h"

Population::Population(uint in_size)
{
    m_number = in_size;
    m_indiv  = new Individual*[m_number];
}

Population::~Population()
{
    delete[] m_indiv;
}

void Population::Clear()
{
    for(uint i=0; i<m_number;i++){
        delete m_indiv[i];
    }
}
