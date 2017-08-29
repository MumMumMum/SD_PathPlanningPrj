#include "PTG.h"
#include <iostream>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
default_random_engine generator;
using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


vector<double> PTG::PTG_Solve(vector<double> start_state, int target_id, vector<double> delta){
	
	Vehicle targetVeh = vehicle_list[target_id];
	cout<<"target veh id :"<<targetVeh._id<<" targetVeh s: "<<targetVeh.start_state[0]<<"targetVeh d: "<<targetVeh.start_state      	[3]<<endl<<endl;
        cout<<"SD Veh :"<<" SD s: "<<sd_veh.start_state[0]<<"SD d: "<<sd_veh.start_state[3]<<endl<<endl;
	//generate alternative goals for space around target vehicle
	vector< vector<double>> goals;
	vector< vector<double>> all_goals;
	vector<double> target_state ;
	double timestep = 0.1;
	double t = T - 4 * timestep;
	vector<double> state;
		while(t <= (T + 4 * timestep)){
			cout<<"t:"<<t<<endl;
			state = targetVeh.Get_state_At(t);
			//cout<<"target veh state at t s :"<<state[0]<<" s_"<<state[1]<<" s__:"<< state[2] <<" d: "<<state[3]<<endl;
			target_state = Add_vector(state ,delta);
			//cout<<"target state + delta  s :"<<target_state[0]<<" s_"<<target_state[1]<<" s__:"<< target_state[2] 
			//<<" d: "<<target_state[3]<<endl;
		        goals = perturb_goal(target_state,t);
			//cout<<"goals d  "<<goals[6][3]<<endl;
			all_goals.insert(std::end(all_goals), std::begin(goals), std::end(goals));
			//cout<<"goals size"<<all_goals.size()<<endl;
			t += 	timestep;
		}
	//Now we start JTM ,
	vector< vector<double>> trajectories;
	vector<double> s_coefficients,d_coefficients,start_state_s,start_state_d,end_state_s,end_state_d;
	trajectories.resize(all_goals.size());
	start_state_s = {start_state[0],start_state[1], start_state[2] };
	start_state_d = {start_state[3],start_state[4], start_state[5] };
	vector<double> temp;
	for(int i = 0 ; i < all_goals.size();i++){
			
		temp = all_goals[i];
		
		end_state_s = { temp[0], temp[1], temp[2] };
		end_state_d = { temp[3], temp[4], temp[5] };
		t = temp[6];
		//cout<<"t: "<<t<<endl;
		s_coefficients = JMT(start_state_s, end_state_s, t);
        	d_coefficients = JMT(start_state_d, end_state_d, t);
		
		trajectories[i].insert(end(trajectories[i]), begin(s_coefficients), end(s_coefficients));
		trajectories[i].insert(end(trajectories[i]), begin(d_coefficients), end(d_coefficients));
		trajectories[i].push_back(t);
		/*cout<<"traje size:"<<trajectories[i].size()<<endl;
		cout<<"Traj size :" <<trajectories.size()<< endl;
		cout<<"trajectories[i] s : "<<trajectories[i][0]<<" trajectories[i] s' : "<<trajectories[i][1]
		<<" trajectories[i] s'' : "<<trajectories[i][2]<<endl;

		cout<<"trajectories[i] D : "<<trajectories[i][6]<<" trajectories[i] D' : "<<trajectories[i][7]
		<<" trajectories[i] d'' : "<<trajectories[i][8]<< " T :"<<trajectories[i][12]<< endl;*/

        }

	// Apply cost function to find the best Traj
	double min_cost = std::numeric_limits<double>::max();
	double cost;
	vector<double> best_trajectory;
	
	for (size_t i = 0; i < trajectories.size(); ++i) {

		cost = calculate_cost(trajectories[i], targetVeh);
		if (cost < min_cost) {
			min_cost = cost;
			best_trajectory = trajectories[i];
		}
		
	}

	return best_trajectory;


}

double PTG::calculate_cost(vector<double> trajectory,Vehicle targetVeh) {
	double cost = 0;

	cost_function cf;
	
	cost += 1 * cf.collision_cost(trajectory,targetVeh);
	cost += 1 * cf.total_acceleration_cost(trajectory);
	cost += 1 * cf.max_acceleration_cost(trajectory);
	cost += 1 * cf.efficiency_cost(trajectory);
	cost += 1 * cf.total_jerk_cost(trajectory);
	cost += 1 * cf.buffer_cost(trajectory,targetVeh);
	cost += 1 * cf.s_diff_cost(trajectory,targetVeh);
	cost += 1 * cf.d_diff_cost(trajectory,targetVeh);
	//cost += 1 * speed_limit_cost(trajectory);
	cost += 1 * cf.max_jerk_cost(trajectory);
	//cost += 1 * cf.stay_in_lane(trajectory);

	return cost;
}



vector<double> PTG::JMT(vector< double> start, vector <double> end, double T)
{
	/*
	Calculate the Jerk Minimizing Trajectory that connects the initial state
	to the final state in time T.
	INPUTS
	start - the vehicles start location given as a length three array
	corresponding to initial values of [s, s_dot, s_double_dot]
	end   - the desired end state for vehicle. Like "start" this is a
	length three array.
	T     - The duration, in seconds, over which this maneuver should occur.
	OUTPUT
	an array of length 6, each value corresponding to a coefficent in the polynomial
	s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
	EXAMPLE
	> JMT( [0, 10, 0], [10, 10, 0], 1)
	[0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
	*/

	const double s_i = start[0];
	const double s_i_dot = start[1];
	const double s_i_dot_dot = start[2] *0.5;

	const double s_f = end[0];
	const double s_f_dot = end[1];
	const double s_f_dot_dot = end[2];

	const double s_r_1 = s_f - (s_i + s_i_dot * T + (s_i_dot_dot * T * T) * 0.5);
	const double s_r_2 = s_f_dot - (s_i_dot + s_i_dot_dot * T);
	const double s_r_3 = s_f_dot_dot - s_i_dot_dot;
			
	Eigen::VectorXd t_1(3), t_2(3), t_3(3);
	t_1 << pow(T, 3), pow(T, 4), pow(T, 5);
	t_2 << 3 * pow(T, 2), 4 * pow(T, 3), 5 * pow(T, 4);
	t_3 << 6 * pow(T, 1), 12 * pow(T, 2), 20 * pow(T, 3);

	Eigen::MatrixXd s_matrix(3, 1), T_matrix(3, 3);
	s_matrix << s_r_1, s_r_2, s_r_3;
	T_matrix.row(0) = t_1;
	T_matrix.row(1) = t_2;
	T_matrix.row(2) = t_3;

	Eigen::MatrixXd T_inverse = T_matrix.inverse();

	Eigen::MatrixXd output(3, 1);
	output = T_inverse * s_matrix;

	double a_3 = double(output.data()[0]);
	double a_4 = double(output.data()[1]);
	double a_5 = double(output.data()[2]);

	vector<double> results;
	results = { s_i, s_i_dot, s_i_dot_dot, a_3, a_4, a_5 };
	//cout << "in jtm///////;"<<output << endl;
	return results;

}
    



vector< vector<double>>  PTG::perturb_goal(vector<double> target_state,double t){
	vector< vector<double>> goals;	
	vector<double> new_goal;
	goals.clear();
	
	cout<<"target s "<<target_state[0]<<"target d "<<target_state[3]<<  endl;
	for(int K = 0 ; K < N_SAMPLES; K++ ){
		new_goal.clear();
		for (size_t i = 0; i < 3; ++i) {

			normal_distribution<double> distribution(target_state[i], SIGMA_S[i]);
			new_goal.push_back(distribution(generator));
		}
		for (size_t i = 0; i < 3; ++i) {
			//cout<<"target_state[i+3]:"<<target_state[i+3]<<endl;
			normal_distribution<double> distribution2(target_state[i+3], SIGMA_D[i]);
			new_goal.push_back(distribution2(generator));
			//cout<<"new_goal[i+3]:"<<new_goal[i+3]<<endl;
		}

		//cout << "new goal s\t" << new_goal[0] << " \tnew goal d\t" << new_goal[3] << endl;
		new_goal.push_back( t);
		//cout<<"new_goal size :: "<<new_goal.size()<<endl;
		goals.push_back((new_goal));
		cout << " s : " << goals[K][0] << " d: " << goals[K][3] << endl;	
	}
	
	return goals;
}

PTG::S_D  PTG::build_trajectory(vector<double> trajectory) {

	vector<double> S, D, X, Y; // TODO refactor using S_D struct
	S_D S_D_;


	for (size_t i = 0; i < 6; ++i) {
		S.push_back(trajectory[i]);
		D.push_back(trajectory[6 + i]);
		//cout << "S" << i << "\t"<<  S[i] << " \t D[i] \t" << D[i] << endl;
	}
	
	S_D_.S.clear();
	S_D_.D.clear();

	double time = 0;

	/*double time = 0;
	if (our_path->ref_velocity > 20) {
		time = 0;
	}
	if (our_path->ref_velocity > 30) {
		time = .7;
	}
	if (our_path->ref_velocity > 40) {
		time = .9;
	}

	if (our_path->lane_change_state == true) {
		time = 2.9;
		if (our_path->ref_velocity > 40) {
			time = 3.7;
		}
		if (our_path->ref_velocity > 43) {
			time = 3.8;
		}
	}*/
	





	double end_time = (trajectory[12]) ;

	cout << "end_time\t" << end_time << endl;
	while (time <= end_time) {
		S_D_.S.push_back(coefficients_to_time_function(S, time));
		S_D_.D.push_back(coefficients_to_time_function(D, time));
		time += 0.02;
	}
	return S_D_;

}


/*vector<double> PTG::GenerateWaypoint( int lane ,float velocity){
  // interleaved (x,y) path points we will return for path plan
  vector<double> path;

  // create a list of widely spaced waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill
  // it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, and yaw states
  // either we will reference the starting point as where the car is
  // or at the previous path's end point
  double ref_x = sd_veh.x;
  double ref_y = sd_veh.y;
  double ref_yaw = deg2rad(sd_veh.yaw);
  cout<<"ref_x: "<<  ref_x<<" ref_y: "<<  ref_y<<" ref_yaw : "<<  ref_yaw<<endl<<endl;
  // if previous path is almost empty, use the car as starting reference
  if (waypoint_ds.previous_path_size < 2) {
    double prev_car_x = sd_veh.x - cos(deg2rad(sd_veh.yaw));
    double prev_car_y = sd_veh.y - sin(deg2rad(sd_veh.yaw));

    // use two points that make the path tangent to the car
    ptsx.push_back(prev_car_x);
    ptsx.push_back(sd_veh.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(sd_veh.y);
  } else {  // use the previous path's end point as the starting reference
    // redefine reference state as previous path end point
    ref_x = waypoint_ds.previous_path_x[waypoint_ds.previous_path_size - 1];
    ref_y = waypoint_ds.previous_path_y[waypoint_ds.previous_path_size - 1];

    // and second to last end point
    double ref_x_prev = waypoint_ds.previous_path_x[waypoint_ds.previous_path_size - 2];
    double ref_y_prev = waypoint_ds.previous_path_y[waypoint_ds.previous_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's
    // end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  for (int i = 0; i < waypoint_ds.previous_path_size; i++) {
    path.push_back(waypoint_ds.previous_path_x[i]);
    path.push_back(waypoint_ds.previous_path_y[i]);
  }

  // use frenet to evenly add 30m spaced points (30m, 60m, 90m) ahead of starting reference
  //int lane = getLaneFrenet(this->car_lane, 0);
  int lane = 6;

  //PTG::S_D sd = build_trajectory(trajectorybest) ;
  vector<double> S = sd.S;
  vector<double> D = sd.D;
  cout<<"size of vector S and D for path"<<S.size()<<endl<<endl<<endl;
  cout<<"size of vector S and D for path"<<S [ SPLINE_SPACING]<<endl<<endl<<endl;
  cout<<"size of vector S and D for path"<<S [ SPLINE_SPACING+1]<<endl<<endl<<endl;
  cout<<"size of vector S and D for path"<<S [ SPLINE_SPACING+2]<<endl<<endl<<endl;


	for(int i = 0; i <= HORIZON_DIST - waypoint_ds.previous_path_size; i++ ){
		vector<double> next_wp0 = getXY( S[i], lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		ptsx.push_back(next_wp0[0]+ref_x);
		ptsy.push_back(next_wp0[1]+ref_y);
		path.push_back(next_wp0[0]+ref_x);
		path.push_back(next_wp0[0]+ref_y);
	}

 
      return path;
 

}*/

//vector<double> PTG::GenerateWaypoint( int lane ,float velocity){
vector<double> PTG::GenerateWaypoint( void){
  // interleaved (x,y) path points we will return for path plan
  vector<double> path;

  // create a list of widely spaced waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill
  // it in with more points that control speed
  vector<double> ptsx;
  vector<double> ptsy;

  // reference x, y, and yaw states
  // either we will reference the starting point as where the car is
  // or at the previous path's end point
  double ref_x = sd_veh.x;
  double ref_y = sd_veh.y;
  double ref_yaw = deg2rad(sd_veh.yaw);
  cout<<"ref_x: "<<  ref_x<<" ref_y: "<<  ref_y<<" ref_yaw : "<<  ref_yaw<<endl<<endl;
  // if previous path is almost empty, use the car as starting reference
  if (waypoint_ds.previous_path_size < 2) {
    double prev_car_x = sd_veh.x - cos(deg2rad(sd_veh.yaw));
    double prev_car_y = sd_veh.y - sin(deg2rad(sd_veh.yaw));

    // use two points that make the path tangent to the car
    ptsx.push_back(prev_car_x);
    ptsx.push_back(sd_veh.x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(sd_veh.y);
  } else {  // use the previous path's end point as the starting reference
    // redefine reference state as previous path end point
    ref_x = waypoint_ds.previous_path_x[waypoint_ds.previous_path_size - 1];
    ref_y = waypoint_ds.previous_path_y[waypoint_ds.previous_path_size - 1];

    // and second to last end point
    double ref_x_prev = waypoint_ds.previous_path_x[waypoint_ds.previous_path_size - 2];
    double ref_y_prev = waypoint_ds.previous_path_y[waypoint_ds.previous_path_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    // use two points that make the path tangent to the previous path's
    // end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }

  // use frenet to evenly add 30m spaced points (30m, 60m, 90m) ahead of starting reference
  int d = getLaneFrenet(best_lane, 0);
  cout <<"From best lane d: "<<d<<endl;

 

  vector<double> next_wp0 = getXY(sd_veh.start_state[0] + (SPLINE_SPACING *1), d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp1 = getXY(sd_veh.start_state[0] + (SPLINE_SPACING *2), d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
  vector<double> next_wp2 = getXY(sd_veh.start_state[0] + (SPLINE_SPACING *3), d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);
  // NOTE: these 5 (x,y) points are our "spline anchor points"; the
  // spline will be fitted to these, but later we will generate a set
  // of "in between points" on the spline for following

  // translate and rotate all points to the car's
  // local coordinate system; set the origin of points
  // to 0,0 (translate) and the rotation to 0 degrees
  for (int i = 0; i < ptsx.size(); i++) {
    // shift car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

		/*for (auto x : ptsx)
			std::cout << "x value is: " <<  x << endl;
		std::cout << std::endl;

		for (auto y : ptsy)
			std::cout << "y value is: " << y << endl;
		std::cout << std::endl;*/

  // create a spline
  tk::spline s;

  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);

  // fill in with the remainder of the previous path that the
  // simulator reports (dots not consumed pac-man like!)
  for (int i = 0; i < waypoint_ds.previous_path_size; i++) {
    path.push_back(waypoint_ds.previous_path_x[i]);
    path.push_back(waypoint_ds.previous_path_y[i]);
  }

  // calculate how to break up the spline points so that we travel
  // at our desired reference velocity
  double target_x = SPLINE_SPACING;  // horizon 30 meters, 0.6s (30 * 0.02)
  double target_y = s(target_x);
  double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

  double x_add_on = 0;  // start at origin (local car coordinates)

  // fill up the rest of our path plan after filling it with
  // the previous points, here we will always output 50 points
  // (1 meter distance between points)
  // Note: 2.24 is our conversion factor, 1 m/s = 2.24 mph
  for (int i = 1; i <= HORIZON_DIST - waypoint_ds.previous_path_size; i++) {
    double N = (target_dist / (SECS_PER_TICK * ref_velocity / MPH_TO_METERS_PER_SEC));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;  // shift to next point

    double x_ref = x_point;
    double y_ref = y_point;

    // rotate and translate points back to global coordinates
    // to reverse earlier translate/rotate to local car coordinates
    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    path.push_back(x_point);
    path.push_back(y_point);
  }

  return path;
}


bool  PTG::CheckStateChange(void){
	bool st_change=false;
	
	float sdveh_s = 0;
	if (waypoint_ds.previous_path_size > 2){
	  sdveh_s = waypoint_ds.end_s;
      	  cout<<"end_path_s"<<sdveh_s<<endl;
		//sdveh_s = sd_veh.start_state[0];
	}
	
	for (int i = 0 ; i< vehicle_list.size();i++){
		Vehicle veh = vehicle_list[i];
		float veh_d = veh.start_state[3];
		float veh_s = veh.start_state[0];
		float veh_v = veh._v;
		int veh_lane  = getLaneNumber(veh_d);
		int sdveh_lane  = getLaneNumber(sd_veh.start_state[3]);
		
		if(veh_lane == sdveh_lane ){
		       // project s at next dt time
			veh_s += ((waypoint_ds.previous_path_size/3)*dt*veh_v);
			
			//checks if time is good to start to decelarate
			if(veh_s >= sdveh_s && ((veh_s - sdveh_s) <= buffer) ){
				cout<<"BM::Detc close:"<<endl<<endl<<endl;
				st_change = true;
				return st_change ;	
			}
		}
	
	} 
	
}

void  PTG::SetBestLane(){

	int cr_lane = best_lane;
	cout<<"SBL::for lane"<<cr_lane<<endl;
	bool safe = false;
	if(cr_lane == lane_left){
		safe = check_safe(lane_middle);
		if(safe){cout<<"change lane middle done!!!"<<endl;best_lane = lane_middle;}
	}
	else if (cr_lane == lane_right){
		safe = check_safe(lane_middle);
		if(safe){cout<<"change lane middle done!!!"<<endl;best_lane = lane_middle;}
	}
	else
	{
		
		safe = check_safe(lane_left);
		if(safe){best_lane = lane_left;cout<<"change lane left done!!!"<<endl;}
		else{
		   cout<<"left lane test failed, checking rt lane"<<endl;
		   safe = check_safe(lane_right);
		   if(safe){best_lane = lane_right;cout<<"change lane right done!!!"<<endl;}
		   else{cout<<"total fail lane change"<<endl;}
		}
	}
	//cout<<"SBL:too_close: "<<too_close<<endl;
}

bool PTG::check_safe(int lane){

	float sdveh_s = 0;
	bool safe = true;
	if (waypoint_ds.previous_path_size > 2){
	  sdveh_s = waypoint_ds.end_s;
      	  
	}
	cout<<"check safe  Lane function for lane:  "<<lane<<endl;
	for (int i = 0 ; i< vehicle_list.size();i++){
		Vehicle veh = vehicle_list[i];
		float veh_d = veh.start_state[3];
		float veh_s = veh.start_state[0];
		float veh_v = veh._v;
		int veh_lane  = getLaneNumber(veh_d);
		
		if(veh_lane == lane){
		// project s at next dt time
			veh_s += ((waypoint_ds.previous_path_size)*dt*veh_v);
			if((veh_s >= sdveh_s && ((veh_s - sdveh_s) <= buffer)) || (veh_s <= sdveh_s && ((sdveh_s  - veh_s) <= buffer)) ){
				safe =  false;
			}
		}
	}
	return safe;
}

//Set Acc returns true if veh dist in lane > buffer 

bool PTG::setAccerate(void){
bool acc = true;
float sdveh_s = 0;
	bool safe = true;
	if (waypoint_ds.previous_path_size > 2){
	  sdveh_s = waypoint_ds.end_s;
      	  
	}
	int sdveh_lane  = getLaneNumber(sd_veh.start_state[3]);
	cout<<"check acc   function for SDVEh lane:  "<<sdveh_lane<<endl;
	for (int i = 0 ; i< vehicle_list.size();i++){
		Vehicle veh = vehicle_list[i];
		float veh_d = veh.start_state[3];
		float veh_s = veh.start_state[0];
		float veh_v = veh._v;
		int veh_lane  = getLaneNumber(veh_d);
		
		//cout<<"veh d: "<<veh_d<<" veh lane "<<veh_lane <<endl;
		//cout<<"veh s: "<<veh_s<<endl;
		//cout<<"v: "<<veh_v<<endl;
		//cout<<"SD veh S:"<<sdveh_s<<endl;
		if(veh_lane == sdveh_lane){
		        // project s at next dt time
			veh_s += ((waypoint_ds.previous_path_size/2)*dt*veh_v);
			//cout<<"veh lane : "<<lane<<endl;
			//cout<<"veh s in future: "<<veh_s<<endl;
			//cout<<"veh s diff: "<<(veh_s - sdveh_s)<<endl;
			//cout<<"Ref vel true: diff in S"<<(veh_s - sdveh_s) <<endl<<endl<<endl;
			//cout<<"Ref vel true: diff in S"<<(sdveh_s  - veh_s) <<endl<<endl<<endl;
			if((veh_s >= sdveh_s && ((veh_s - sdveh_s) <= buffer))){
				
				acc =  false;
			}
		}
		
	//cout<<"======================================================="<<endl;
	}
return acc;
}


void PTG::StateManagement(void){


float ref_vel = sd_veh.v;
int sdveh_lane  = getLaneNumber(sd_veh.start_state[3]);
cout<<"SM::ref vel : "<<ref_vel<<endl;
cout<<"SM::state_FSM : "<<state_FSM<<endl;
cout<<" SM::sdveh_lane in "<<sdveh_lane<<endl;
if(state_FSM == KL){

bool stateChange = CheckStateChange();
if(stateChange) {state_FSM = PRP_LC;ref_vel -= (.224*9);}
else {state_FSM = KL;ref_vel += (.224*5);}

}
else if(state_FSM == PRP_LC ){
ref_vel -= (.224*9);
if(ref_vel <= 35){
  state_FSM = LC;
  cout<<"SM::state_FSM : "<<state_FSM<<endl;
  }
}
else if(state_FSM == LC){
ref_vel -= (.224*9);
SetBestLane();
state_FSM = LC_Done;
}
else if(state_FSM == LC_Done){
ref_vel -= (.224*9);

cout<<" SM::sdveh_lane in LC DONE st"<<sdveh_lane<<endl;
if(sdveh_lane == best_lane)
state_FSM = KL;
}
else{

}


if(ref_vel >=  SPEED_LIMIT) ref_vel = SPEED_LIMIT-1;
        ref_velocity = ref_vel; 

}
