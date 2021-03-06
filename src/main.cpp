#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "HELPER.h"
#include "PTG.h"
#include <map>


using namespace std;

// for convenience
using json = nlohmann::json;



// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

map<int,Vehicle> vehicle_list;
vector<int > veh_id_front;
const vector<double> SIGMA_S = { 5.0, .1, .01 };
const vector<double> SIGMA_D = {0.2, .1, .1 };
Vehicle_SD sd_veh;
wayPoints waypoint_ds;


vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;
  

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
 
  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

PTG ptg;
ptg.ref_velocity = 0;
ptg.best_lane = lane_middle;
ptg.state_FSM = KL;

h.onMessage([&]	(uWS::WebSocket<uWS::SERVER> ws, char 	*data, size_t length,uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    auto sdata = string(data).substr(0, length);
    cout << sdata << endl;
    cout<<endl;
    cout<<endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
       
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
		 
		sd_veh.x = car_x;
		sd_veh.y = car_y;
		sd_veh.v = car_speed;
		sd_veh.yaw = car_yaw;
		sd_veh.lane = 1;//getLaneNumber(car_d);
		//cout<<"veh d: "<<car_d<<"  veh lane : "<< sd_veh.lane<<endl;
		//cout<<endl;
		sd_veh.state = KL;

		sd_veh.start_state = Get_state_derivative( car_speed, car_s, car_d,dt );
		//cout<<"SD veh start state is : S:"<<sd_veh.start_state[0]<<" s-dot: "
		//<<sd_veh.start_state[1]<<" s-dot-dot: "<<sd_veh.start_state[2]<<" d"<<sd_veh.start_state[3]<<endl;
		//cout<<endl;
          	// Previous path data given to the Planner
		// changed from auto to vector<double> for prev x and y
          	vector<double> previous_path_x = j[1]["previous_path_x"];
          	vector<double> previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
		//cout<<"end_path_s: "<<end_path_s<<endl;

		waypoint_ds.previous_path_x = (previous_path_x);
		waypoint_ds.previous_path_y = previous_path_y;
		waypoint_ds.end_s = end_path_s;
		waypoint_ds.end_d = end_path_d;
		//cout<<"end_path_s: "<<waypoint_ds.end_s<<endl;
		waypoint_ds.previous_path_size = previous_path_y.size();
		

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];
		//cout<<"sensor:"<<sensor_fusion.size()<<endl;
		
		vehicle_list.clear();
		
		for(int i = 0; i < sensor_fusion.size(); i++ ){
			auto sensor_state = sensor_fusion[i];
			int id = sensor_state[0];
			double x = sensor_state[1];
			double y = sensor_state[2];
			double vx = sensor_state[3];
			double vy = sensor_state[4];
			double s = sensor_state[5];
			double d = sensor_state[6];
			Vehicle veh = Vehicle(id,x,y,vx,vy,s,d);
			vehicle_list.insert(std::pair<int,Vehicle>(id, veh));			
			}
		
		
		ptg.StateManagement();
		vector<double> path = ptg.GenerateWaypoint( );
		json msgJson;
		vector<double> next_x_vals;
          	vector<double> next_y_vals;
		//printf("hari om");


          	//.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
		// the points in the simulator are connected by a Green line
		for (size_t i = 0; i < path.size(); i++) {
			if (i % 2 == 0) {
				next_x_vals.push_back(path[i]);
			} else {
				next_y_vals.push_back(path[i]);
			}
		}


		msgJson["next_x"] = next_x_vals;
		msgJson["next_y"] = next_y_vals;
		std::cout << "!!!===============msg done:=========================================!!!"<<std::endl;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
















































































