#include "COSTFN.h"

extern const vector<double> SIGMA_S ;
extern const vector<double> SIGMA_D ;

//==================================================================//
/* """
    Binary cost function which penalizes collisions.
    """*/
double cost_function::collision_cost(vector<double>trajectory,Vehicle targetVeh){
	
	double a = nearest_approach_to_any_vehicle(trajectory, targetVeh);
	cout<<"nearest vehi dis a: "<<a <<endl;
	double b = 30 * VEHICLE_RADIUS;
	//cout << b << endl;
	if (a < b) { 
		// cout << a << endl; 
		return 1.0; 
	}
	else { return 0.0; }
}
//==================================================================//




//==================================================================//
double cost_function::max_acceleration_cost(vector<double>trajectory){

	// Penalize higher average acceleration
	vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
	vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
	double total = 0;
	for (double s_ddot: s_ddot_traj) {
		total += s_ddot;
	}
	double avg_accel = total / s_ddot_traj.size();
	return logistic(avg_accel / MAX_ACCEL );
}
//==================================================================//




//==================================================================//
double cost_function::total_acceleration_cost(vector<double>trajectory){
	// Penalize exceeding MAX_INSTANTANEOUS_ACCEL
	vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
	vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
	for (double s_ddot : s_ddot_traj) {
		if (s_ddot > EXPECTED_ACC_IN_ONE_SEC) {
			return 1;
		}
	}
	return 0;
}
//==================================================================//




//==================================================================//
double cost_function::efficiency_cost(vector<double>trajectory){
// Rewards high average speeds.
	vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
	double total = 0;
	for (double s_dot: s_dot_traj) {
		total += s_dot;
	}
	double avg_vel = total / s_dot_traj.size();
	return logistic(2 * (SPEED_LIMIT - avg_vel) / avg_vel);
	
	
}


//==================================================================//






//==================================================================//
double cost_function::total_jerk_cost(vector<double>trajectory){
	// Penalize higher average jerk
	vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
	vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
	vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
	double total = 0;
	for (double s_dddot: s_dddot_traj) {
		total += s_dddot;
	}
	double avg_jerk = total / s_dddot_traj.size();
	return logistic(avg_jerk / EXPECTED_JERK_IN_ONE_SEC );
}
//==================================================================//



//==================================================================//

double cost_function::max_jerk_cost(vector<double>trajectory){

// Penalize exceeding MAX_INSTANTANEOUS_JERK
  vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
  vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
  vector<double> s_ddot_traj = velocities_for_trajectory(s_dot_traj);
  vector<double> s_dddot_traj = velocities_for_trajectory(s_ddot_traj);
  for (double s_dddot : s_dddot_traj) {
    if (s_dddot > MAX_JERK) {
      return 1;
    }
  }
return 0;

}

//==================================================================//


//==================================================================//
/* """
    Penalizes getting close to other vehicles.
    """*/
double cost_function::buffer_cost(vector<double>trajectory,Vehicle targetVeh){

double nearest = nearest_approach_to_any_vehicle(trajectory, targetVeh);
    return logistic(2*VEHICLE_RADIUS / nearest);
}
//==================================================================//


//==================================================================//
/*
    Penalizes trajectories whose s coordinate (and derivatives) 
    differ from the goal.
*/


double cost_function::s_diff_cost(vector<double>trajectory,Vehicle targetVeh){
	vector<double> S, S_coefficients; // TODO better way to do this
	S = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
	double _T = trajectory[12];
	double cost = 0;
	double difference;

	vector<double> target = targetVeh.Get_state_At(_T);
	// get upto 2 derivative N =2
	S_coefficients = get_f_and_N_derivatives(S, 2);
	cout<<"T: "<< _T << endl;
	//cout<<"S_coefficients size: "<<S_coefficients.size() << endl;
	for (size_t i = 0; i < S_coefficients.size() ; ++i) {
		// actual - expected
		difference = fabs(S_coefficients[i] - targetVeh.start_state[i]);
		cost += logistic(difference / SIGMA_S[i]);
	}

	return cost;
}
//==================================================================//


//==================================================================//
/*
    Penalizes trajectories whose d coordinate (and derivatives) 
    differ from the goal.
*/

double cost_function::d_diff_cost(vector<double>trajectory,Vehicle targetVeh){

    
// This should work better
       /* vector<double> d_coeffs, d_dot_coeffs,d_ddot_coeffs,d_coefficients; // TODO better way to do this
        d_coeffs = { trajectory[6], trajectory[7], trajectory[8], trajectory[9], trajectory[10], trajectory[11] };
        double _T = trajectory[12];
	double cost = 0;
	double difference;

	vector<double> target = targetVeh.Get_state_At(_T);
	// get upto 2 derivative N =2
	d_coefficients = get_f_and_N_derivatives(d_coeffs, 2);
	cout<<"T: "<< _T << endl;
	//cout<<"d_coefficients size: "<<d_coefficients.size() << endl;
	for (size_t i = 0; i < d_coefficients.size() ; ++i) {
		// actual - expected
		difference = fabs(d_coefficients[i] - targetVeh.start_state[i+3]);
		cost += logistic(difference / SIGMA_D[i]);
	}

	return cost;*/


  vector<double> d_coeffs;
  double d1, d2, d3, d_dot1, d_dot2, d_ddot, cost = 0;
  d_coeffs = { trajectory[6], trajectory[7], trajectory[8], trajectory[9], trajectory[10], trajectory[11] };
  d1 = d_coeffs[5];
  d2 = d_coeffs[4];
  d3 = d_coeffs[3];
  d_dot1 = (d1 - d2) / dt;
  d_dot2 = (d2 - d3) / dt;
  d_ddot = (d_dot1 - d_dot2) / dt;
  cost += fabs(d1 - targetVeh.start_state[3]) / SIGMA_D[0];
  cost += fabs(d_dot1 - targetVeh.start_state[4]) / SIGMA_D[1];
  cost += fabs(d_ddot - targetVeh.start_state[5]) / SIGMA_D[2];
  return logistic(cost);
   
}
//==================================================================//


//==================================================================//

double cost_function::speed_limit_cost(vector<double>trajectory){
// Penalty if ds/dt for any two points in trajectory is greater than SPEED_LIMIT
vector<double> s_traj = { trajectory[0], trajectory[1], trajectory[2], trajectory[3], trajectory[4], trajectory[5] };
vector<double> s_dot_traj = velocities_for_trajectory(s_traj);
for (double s_dot : s_dot_traj) {
	if (s_dot > SPEED_LIMIT) {
	    return 1;
	}
}
return 0; 
}
//==================================================================//




//==================================================================//
double cost_function::stay_in_lane(vector<double>trajectory){
// penalize not shooting for middle lane (d = 6)
vector<double> d_traj = { trajectory[6], trajectory[7], trajectory[8], trajectory[9], trajectory[10], trajectory[11] };
double end_d = d_traj[d_traj.size()-1];
return logistic(pow(end_d-6, 2));
}
//==================================================================//




//==================================================================//


double cost_function::time_diff_cost(vector<double>trajectory){


 /*Penalizes trajectories that span a duration which is longer or 
    shorter than the duration requested.*/
//trajectory has 6 s coeff, 6 d coeff , and t so index is 12
	double t = trajectory[12];

    	return logistic(float(abs(t-T_diff)) / T_diff);

}
//==================================================================//
