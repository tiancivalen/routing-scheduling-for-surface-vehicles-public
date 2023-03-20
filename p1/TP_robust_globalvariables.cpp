//by Tianci Zhang

#include "TP_robust_globalvariables.h"
#include "globalvariable.h"

//-------------changable params--------------------
double g_frequency = 0.5;

double g_pathnumber = 1;

double g_ratio_range = 1.5;

uint g_WTSType = 3;

//-------------------------------------------------

double g_ratio_density = 1/(g_frequency*2);
double g_k = 0.1;
double g_ioa = 4;
bool   g_flag_startdelay = true;

//-------------------------------------------------


//-----------fixed params--------------------------
const double g_w = 1;               //weight of taxi time
const double g_w_delay = 1;         //weight of delay
const double g_len_plane = 50;
const double g_sep = 2*50/g_speed;
//-------------------------------------------------


Vehicles g_vehs4comp;
