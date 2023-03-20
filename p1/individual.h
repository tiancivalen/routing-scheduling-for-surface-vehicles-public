#ifndef INDIVIDUAL_H
#define INDIVIDUAL_H
#include <qglobal.h>
class Individual
{
public:
    Individual();
    ~Individual();
public:
    uint m_length;
    uint* m_chromosome;
    double m_fitness;


};

#endif // INDIVIDUAL_H
