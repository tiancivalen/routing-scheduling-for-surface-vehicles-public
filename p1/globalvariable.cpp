//by Tianci Zhang

#include "globalvariable.h"

//live for the whole program
double                 g_speed = 8;
double                 g_speed_turn = 5.14;
const double g_runway_speed = 5*g_speed;
double                 g_speed_lb = g_speed_turn;


//live within a sequence
uint                    g_objcount = 0;
uint                    g_priority;
ComputationTimeVector   ctvector;
StepLengthVector        slvector;
std::vector<double>     g_starting_wait_vector;
IterationCountVector    itr_cnt_vector;
std::vector<double>     g_avg_timewindows_vector;
std::vector<double>     g_expandtime_vector;
std::vector<double>      g_accesscount_vector;
//live within a plan process
Closelist               closelist;
OpenlistForExpand       openlistexp;
double*                 g_array_h;
uint                    g_iterationcount =0;
uint                    g_accessibleTNcount = 0;
double                   g_accesscount;
//
uint g_start_region;
double g_start_time;
uint g_start_node;
uint g_end_region;
uint g_end_node;
//
Vehicles g_vehs;
CTaxiwayModel g_model;
CConfig g_config;

bool b_ga_clicked = false;
QMutex g_mutex;

std::vector<base_step*> g_tmp_basestepvector;

uint g_veh_id;


std::vector<int> g_flag_failure;


//
uint g_plan_num = 50;

Vehicle* g_pveh;

std::vector<base_step*> g_partialpathvector;


