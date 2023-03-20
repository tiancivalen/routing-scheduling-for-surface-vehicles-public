#ifndef POPULATION_H
#define POPULATION_H
#include <qglobal.h>
#include <vector>
#include "individual.h"

class Population
{
public:
    Population(uint in_size=10);
    ~Population();
    void Clear();
public:
    uint m_number;
    Individual** m_indiv;
    double m_popufitness;
};

#endif // POPULATION_H
