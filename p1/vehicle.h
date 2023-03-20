//by Tianci Zhang

#ifndef VEHICLE_H
#define VEHICLE_H
#include <qglobal.h>
#include <vector>
#include "common.h"
#include "robust_step.h"
enum VehicleType{Unknown,DEPAR,ARRIV};//
class Vehicle
{
public:
    Vehicle();
    ~Vehicle();
public:
    VehicleType m_type;
    uint m_id;
    uint m_start_region;
    uint m_start_node;
    uint m_end_region;
    uint m_end_node;
    double m_start_time;
    Path m_path;
    rPath m_rPath;
    rPath m_rPath_all;
    std::vector<uint> m_SinglePathLenVector;
    uint m_runway;//landing/taking off runway id
};

typedef std::vector<Vehicle*>            Vehicles;

#endif // VEHICLE_H
