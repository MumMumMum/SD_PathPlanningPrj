#ifndef PTG_H
#define PTG_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "HELPER.h"
#include "CONSTANT.h"
#include "COSTFN.h"

using namespace std;
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

extern const vector<double> SIGMA_S; //= { 1., .01, .001 };
extern const vector<double> SIGMA_D ;//= {0.5, .01, .001 };
extern Vehicle_SD sd_veh;
extern map<int,Vehicle> vehicle_list;
extern wayPoints waypoint_ds;
extern vector<double> map_waypoints_x;
extern  vector<double> map_waypoints_y;
extern  vector<double> map_waypoints_s;

class PTG{


public:

//bool too_close;
float ref_velocity;
int best_lane;
int state_FSM;
//PTG();
	
	
//virtual ~PTG();

struct S_D {
		vector<double> S;
		vector<double> D;
};

vector<double> PTG_Solve(vector<double> start_state, int target_id,vector<double> delta);
vector< vector<double>>  perturb_goal(vector<double> target_state,double t);
vector<double> JMT(vector< double> start, vector <double> end, double T);
double calculate_cost(vector<double> trajectory,Vehicle targetVeh);
PTG::S_D  build_trajectory(vector<double> trajectory);
//vector<double> GenerateWaypoint( int lane ,float velocity);
vector<double> GenerateWaypoint( void);
bool CheckStateChange(void);
void SetBestLane(void);
bool check_safe(int lane);
//bool setAccerate(void);
void StateManagement(void);
};



































#endif
