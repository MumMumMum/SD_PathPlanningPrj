#ifndef HELPER_H
#define HELPER_H

#include <map>
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "CONSTANT.h"
#include "spline.h"


using namespace std;



//===========================================================================================//
struct Vehicle_SD{
	double v;
	double x;
	double y;
	double yaw;
	int state_FM;
	vector<double> start_state;
	int lane;
	int state;
};

struct wayPoints{
	vector<double> previous_path_x;
	vector<double> previous_path_y;
	double previous_path_size;
	double end_s;
	double end_d;
	
};



//===========================================================================================//
class Vehicle{
public:	vector<double>  start_state;
	int _id;
	double _x;
	double _y;
	double _v;
	int lane;
	bool in_front;
	Vehicle(int id,double x,double y,double vx, double vy, double s, double d);
	Vehicle();
	vector<double> Get_state_At(double t);
	
	virtual ~Vehicle();
		

};

//===========================================================================================//












//===========================================================================================//

/*"""
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """*/



static vector<double> Get_state_derivative(double v, double s, double d, double t){
	vector<double> state;
	double s_next = s+v*(t);             // velocit = dist/time 
	double s_next1 = s_next+v*(dt);      // velocit = dist/time 
	double s_dot = abs(s - s_next);      //acc = rate of change of velocity 
	double s_dot1 = abs(s_next - s_next1);//acc = rate of change of velocity 
	double s_dot_dot = abs(s_dot1 - s_dot);//s_dot_dot is rate of change of acc;
	double d_dot = 0.0; // assuming we are mainting same d over the time dt
	double d_dot_dot = 0.0;
	//cout<<"v:"<<v << " s: "<< s <<" s_dot: "<< s_dot <<" s_dot1: "<<s_dot1<<"s_dot_Dot:"<<s_dot_dot<<" d: "<<d<<endl;
	state.push_back(s);
	state.push_back(s_dot);
	state.push_back(s_dot_dot);
	state.push_back(d);
	state.push_back(d_dot);
	state.push_back(d_dot_dot);
	//cout<<"6 states are s:"<<state[0] <<" s_dot: "<< state[1]  <<" s_dot1: "<<state[2] <<" d:"<<state[3]<<endl;
 	return state;

}


//===========================================================================================//




//===========================================================================================//
static vector<double> Add_vector(vector<double> src1,vector<double> src2){
	vector<double> temp;	
	for (int i = 0; i < src1.size(); i++){
		temp.push_back(src1[i]+src2[i]);
	}
	return temp;
}



//===========================================================================================//













//===========================================================================================//

// returns f(t) function using coeff vector
static double coefficients_to_time_function(vector<double> coefficients, int N) {
	// Returns a function of time given coefficients
	double total = 0.0;
	for (size_t i = 0; i < coefficients.size(); ++i) {
		total += coefficients[i] * pow(N, i);
	}
	return total;
}

//===========================================================================================//



//===========================================================================================//

// please check next_degree working
static vector<double> differentiate_polynomial(vector<double> coefficients) {
	// given a vector of coefficients, returns 
	vector<double> out;
	int next_degree;
	double result;
	//cout<<"Input Coeff for diff"<<endl;
	//cout<<" s 0: "<<coefficients[0]<<"  s 1 : "<<coefficients[1]
	//	<<"  s 2 : "<<coefficients[2]<<endl;
	//cout<<" s 3: "<<coefficients[3]<<"  s 4 : "<<coefficients[4]
	//	<<"  s 5 : "<<coefficients[5]<<endl;
	for (size_t i = 1; i < coefficients.size(); ++i) {
		next_degree = i + 1;
		result = next_degree * coefficients[i];
		//result = i * coefficients[i];
		out.push_back(result);
		//cout<<" Out : "<<i<<"coeff : "<<out[i]<<endl;
	}
	return out;
}

//===========================================================================================//



//===========================================================================================//

static vector<double> get_f_and_N_derivatives(vector<double> coefficients, int N=3 ){
	
	vector<double> coefficients_d, out;
	double a;
	//cout<<"len input coeff size:"<<coefficients.size()<<endl;
	out.push_back(coefficients_to_time_function(coefficients, N ));
	//cout<<"(In get f and N )out 0: "<<out[0]<<endl;
	coefficients_d = coefficients;
	for (size_t i = 0; i < N; ++i) {
		coefficients_d = differentiate_polynomial(coefficients_d);
		a = coefficients_to_time_function(coefficients_d, N);
		out.push_back(a);
		//cout<<"(In get f and N )out i: "<<out[i]<<endl;
	}
	return out;

}

//===========================================================================================//



//===========================================================================================//

static double nearest_approach_to_any_vehicle(vector<double> trajectory,Vehicle targetVeh) {
	// returns closest distance to any vehicle

	double T_, s_time, d_time,  b, c, e, t_;
	//a = std::numeric_limits<double>::max();
	T_ = trajectory[12];

	vector<double> S, D; // TODO better way to do this
	S = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	D = { trajectory[6], trajectory[7], trajectory[8], trajectory[9], trajectory[10], trajectory[11] };


	t_ = T_ / 100;
	s_time = coefficients_to_time_function(S, t_);
	d_time = coefficients_to_time_function(D, t_);

	vector<double> state = targetVeh.Get_state_At(t_);

	b = pow((s_time - state[0]), 2);
	c = pow((d_time - state[2]), 2);
	e = sqrt(b + c);
	return e;

}

//===========================================================================================//



//===========================================================================================//

static vector<double> velocities_for_trajectory(vector<double> traj) {
  // given a trajectory (a vector of positions), return the average velocity between each pair as a vector
  // also can be used to find accelerations from velocities, jerks from accelerations, etc.
  // (i.e. discrete derivatives)
  vector<double> velocities;
  for (int i = 1; i < traj.size(); i++) {
    velocities.push_back((traj[i] - traj[i-1]) / dt);
  }
  return velocities;
}

//===========================================================================================//




//===========================================================================================//

// Returns the lane # (0, 1, 2) given a frenet D value
static int getLaneNumber(double d) {
  if (d >= 0 && d < LANE_WIDTH) return 0;
  if (d >= LANE_WIDTH && d < LANE_WIDTH * 2) return 1;
  if (d >= LANE_WIDTH * 2 && d < LANE_WIDTH * 3) return 2;

  return -1;  // out of lane boundaries
}

// Returns the center of lane D value for a lane
static int getLaneFrenet(int lane, int offset) {
  return 2 + 4 * lane + offset;
}

//===========================================================================================//




//===========================================================================================//



static double logistic (double x){

   
    return 2.0 / (1 + exp(-x)) - 1.0;


}

static double sigmoid (double x){

   
    return 1.0 / (1 + exp(-x)) ;
}


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
static double deg2rad(double x) { return x * pi() / 180; }

static double rad2deg(double x) { return x * 180 / pi(); }

//===========================================================================================//




//===========================================================================================//

// Transform from Frenet s,d coordinates to Cartesian x,y
static vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}


//===========================================================================================//

 

//===========================================================================================//
extern  map<int,Vehicle> vehicle_list;
extern Vehicle_SD sd_veh;
extern  vector<int > veh_id_front;
extern const vector<double> SIGMA_S; 
extern const vector<double> SIGMA_D; 
extern wayPoints waypoint_ds;
//extern mapPoints map_points;
//===========================================================================================//
#endif

