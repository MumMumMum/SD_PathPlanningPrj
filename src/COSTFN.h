#ifndef COSTFN_H
#define COSTFN_H
using namespace std;
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "HELPER.h"




    
    
class cost_function{
public:
double collision_cost(vector<double>trajectory,Vehicle targetVeh);
double total_acceleration_cost(vector<double>trajectory);
double max_acceleration_cost(vector<double>trajectory);
double efficiency_cost(vector<double>trajectory);
double total_jerk_cost(vector<double>trajectory);
double buffer_cost(vector<double>trajectory,Vehicle targetVeh);
double s_diff_cost(vector<double>trajectory,Vehicle targetVeh);
double d_diff_cost(vector<double>trajectory,Vehicle targetVeh);
double speed_limit_cost(vector<double>trajectory);
double max_jerk_cost(vector<double>trajectory);
double stay_in_lane(vector<double>trajectory);
double time_diff_cost(vector<double>trajectory);

};

#endif
