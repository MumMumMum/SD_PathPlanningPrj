#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "HELPER.h"
#include <cmath>
using namespace std;



	
Vehicle::Vehicle(int id,double x,double y,double vx, double vy, double s, double d){
	this->_id = id;
	this->_x = x;
	this->_y = y;
	this->_v = sqrt((vx*vx)+ (vy*vy)); //velocity using vector addition
	this->start_state = Get_state_derivative( _v, s, d, dt);
	this-> lane = getLaneNumber(d);
	if(sd_veh.start_state[0] < s){
                this->in_front =  true;
		veh_id_front.push_back(id);}
	else
		this->in_front =  false;
	std::sort(veh_id_front.begin(),veh_id_front.end());
}


Vehicle::Vehicle(){
}

Vehicle::~Vehicle() {}

// Gives vector of 3 s state and 3 d states at any give T. 
vector<double> Vehicle::Get_state_At(double t){
	vector<double> state;
	double s0 = start_state[0];
	double s1 = start_state[1];
	double s2 = start_state[2];
	double d0 = start_state[3];
	double d1 = start_state[4];
	double d2 = start_state[5];

	double temp = s0 +(s1*t)+0.5*(s2*t*t);
	//state[0] = (temp);
	state.push_back(temp);
	temp = s1+s2*t;
	//state[1] = (temp);
	state.push_back(temp);

	temp = s2;
	state.push_back(temp);
	//state[2] = (temp);


	temp = d0 +(d1*t)+0.5*(d2*t*t);
	state.push_back(temp);
	//state[3] = (temp);

	temp = d1+d2*t;
	state.push_back(temp);
	//state[4] = (temp);

	temp = d2;
	state.push_back(temp);
	//state[5] = (temp);
	 
	return state;

}




/*"""
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """*/


/*static double logistic (double x){

   
    return 2.0 / (1 + exp(-x)) - 1.0;
}

static double sigmoid (double x){

   
    return 1.0 / (1 + exp(-x)) ;
}*/
