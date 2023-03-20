//by Tianci Zhang

#ifndef GLOBALVARIABLE_H
#define GLOBALVARIABLE_H
#include "common.h"
#include "vehicle.h"
#include <QMutex>
#include "base_step.h"
#include "ctaxiwaymodel.h"
#include "cconfig.h"
//
const    double c_twlengthmin = 2;
const    double c_timemax = 268435456;
const    double c_waitmax = 268435456;
const    double EARTH_RADIUS =  6378.137;
const    double PI = 3.1415926;
//
extern Closelist                closelist;//close list for the algorithm
extern OpenlistForExpand        openlistexp;//openlist for the algorithm
extern double                   g_speed;//aircraft taxiing speed
extern const double             g_runway_speed;
extern double                   g_speed_lb;
extern uint                     g_objcount;//order of current planned agent
extern uint                     g_priority;
extern double*                  g_array_h;//the heuristic array
//
extern uint                     g_iterationcount;//the iteration times for each agent
extern uint                     g_accessibleTNcount;
extern ComputationTimeVector    ctvector;//record the computational time of each agent
extern StepLengthVector         slvector;//record the step number of each path
extern IterationCountVector     itr_cnt_vector;
extern std::vector<double>      g_starting_wait_vector;//record the time waiting at starting position
extern std::vector<double>      g_avg_timewindows_vector;
extern std::vector<double>      g_expandtime_vector;
extern std::vector<double>      g_accesscount_vector;
extern double                   g_accesscount;
//
extern uint g_start_region;
extern double g_start_time;
extern uint g_start_node;
extern uint g_end_region;
extern uint g_end_node;

//
extern Vehicles g_vehs;

extern CTaxiwayModel g_model;
extern CConfig g_config;

extern bool b_ga_clicked;
extern QMutex g_mutex;

extern std::vector<base_step*> g_tmp_basestepvector;
extern std::vector<base_step*> g_partialpathvector;

//for debug
extern uint g_veh_id;

extern std::vector<int> g_flag_failure;


//
extern uint g_plan_num;
extern double g_speed_turn;

#endif // GLOBALVARIABLE_H
