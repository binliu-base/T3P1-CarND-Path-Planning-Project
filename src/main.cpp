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
//added by binliu 170729
#include "vehicle.h"
#include "road.h"
#include "spline.h"
#include "polyTrajectoryGenerator.h"
//end add

using namespace std;

// bool Tools::trace_tag = false;
// string Tools::tracelog = "trace.MY.log";
// ofstream Tools::traceStream = ofstream(Tools::tracelog.data(), ios::app);

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

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

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

//added by binliu 170729
//all traffic in lane (besides ego) follow these speeds
vector<int> LANE_SPEEDS = {49,49,49}; 

//Number of available "cells" which should have traffic
double TRAFFIC_DENSITY   = 0.15;

// At each timestep, ego can set acceleration to value between 
// -MAX_ACCEL and MAX_ACCEL
int MAX_ACCEL = 2;

//impacts default behavior for most states
int SPEED_LIMIT = 49.75;

// These affect the visualization
int FRAMES_PER_SECOND = 50;
int AMOUNT_OF_ROAD_VISIBLE = 40;

int timestep = 0;
int ourlane = -1; 
//end add

// Transform from global Cartesian x,y to local car coordinates x,y
// where x is pointing to the positive x axis and y is deviation from the car's path
vector<double> getLocalXY(double car_x, double car_y, double theta, double wx, double wy) {
  vector<double> results;

  // convert to local coordinates
  float deltax = (wx - car_x);
  float deltay = (wy - car_y);
  results.push_back(deltax*cos(theta) + deltay*sin(theta));
  results.push_back(-deltax*sin(theta) + deltay*cos(theta));
  return results;
}

void fit_spline_segment(double car_s, vector<double> const &map_waypoints_s, vector<double> const &map_waypoints_x, vector<double> const &map_waypoints_y, vector<double> const &map_waypoints_dx, vector<double> const &map_waypoints_dy, vector<double> &waypoints_segment_s, vector<double> &waypoints_segment_s_worldSpace, vector<double> &map_waypoints_x_upsampled, vector<double> &map_waypoints_y_upsampled, vector<double> &map_waypoints_s_upsampled, vector<double> &map_waypoints_dx_upsampled, vector<double> &map_waypoints_dy_upsampled, tk::spline &spline_fit_s_to_x, tk::spline &spline_fit_s_to_y, tk::spline &spline_fit_s_to_dx, tk::spline &spline_fit_s_to_dy) {
  // get 10 previous and 15 next waypoints
  vector<double> waypoints_segment_x, waypoints_segment_y, waypoints_segment_dx, waypoints_segment_dy;
  vector<int> wp_indeces;
  const int lower_wp_i = 9;
  const int upper_wp_i = 15;
  int prev_wp = -1;
  while(car_s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
          prev_wp++;
  for (int i = lower_wp_i; i > 0; i--) {
    if (prev_wp - i < 0)
      wp_indeces.push_back(map_waypoints_s.size() + (prev_wp - i));
    else
      wp_indeces.push_back((prev_wp - i) % map_waypoints_s.size());
  }
  wp_indeces.push_back(prev_wp);
  for (int i = 1; i < upper_wp_i; i++)
    wp_indeces.push_back((prev_wp + i) % map_waypoints_s.size());

  // FILL NEW SEGMENT WAYPOINTS
  const double max_s = 6945.554;
  bool crossed_through_zero = false;
  double seg_start_s = map_waypoints_s[wp_indeces[0]];
  for (int i = 0; i < wp_indeces.size(); i++) {
    int cur_wp_i = wp_indeces[i];
    waypoints_segment_x.push_back(map_waypoints_x[cur_wp_i]);
    waypoints_segment_y.push_back(map_waypoints_y[cur_wp_i]);
    waypoints_segment_dx.push_back(map_waypoints_dx[cur_wp_i]);
    waypoints_segment_dy.push_back(map_waypoints_dy[cur_wp_i]);
    // need special treatment of segments that cross over the end/beginning of lap
    if (i > 0) {
      if (cur_wp_i < wp_indeces[i-1])
        crossed_through_zero = true;
    }
    waypoints_segment_s_worldSpace.push_back(map_waypoints_s[cur_wp_i]);
    if (crossed_through_zero)
      waypoints_segment_s.push_back(abs(seg_start_s - max_s) + map_waypoints_s[cur_wp_i]);
    else
      waypoints_segment_s.push_back(map_waypoints_s[cur_wp_i] - seg_start_s);
  }

  spline_fit_s_to_x.set_points(waypoints_segment_s, waypoints_segment_x);
  spline_fit_s_to_y.set_points(waypoints_segment_s, waypoints_segment_y);
  spline_fit_s_to_dx.set_points(waypoints_segment_s, waypoints_segment_dx);
  spline_fit_s_to_dy.set_points(waypoints_segment_s, waypoints_segment_dy);

  const int samples = int(waypoints_segment_s[waypoints_segment_s.size()-1]);
  map_waypoints_x_upsampled.reserve(samples);
  map_waypoints_y_upsampled.reserve(samples);
  map_waypoints_s_upsampled.reserve(samples);
  map_waypoints_dx_upsampled.reserve(samples);
  map_waypoints_dy_upsampled.reserve(samples);
  for (int i = 0; i < samples; i++) {
    map_waypoints_x_upsampled.push_back(spline_fit_s_to_x(i));
    map_waypoints_y_upsampled.push_back(spline_fit_s_to_y(i));
    map_waypoints_s_upsampled.push_back(i);
    map_waypoints_dx_upsampled.push_back(spline_fit_s_to_dx(i));
    map_waypoints_dy_upsampled.push_back(spline_fit_s_to_dy(i));
  }
}


  // converts world space s coordinate to local space based on provided mapping
double get_local_s(double world_s, vector<double> const &waypoints_segment_s_worldSpace, vector<double> const &waypoints_segment_s) {
  int prev_wp = 0;
  // special case: first wp in list is larger than s. Meaning we are crossing over 0 somewhere.
  // go to index with value zero first and search from there.
  if (waypoints_segment_s_worldSpace[0] > world_s) {
      while (waypoints_segment_s_worldSpace[prev_wp] != 0.0)
          prev_wp += 1;
  }
  while ((waypoints_segment_s_worldSpace[prev_wp+1] < world_s) && (waypoints_segment_s_worldSpace[prev_wp+1] != 0))
      prev_wp += 1;
  double diff_world = world_s - waypoints_segment_s_worldSpace[prev_wp];
  double s_local = waypoints_segment_s[prev_wp] + diff_world;
  return s_local;
}



int main() {
  uWS::Hub h;

  // set up logging
  string log_file = "../data/logger.csv";
  // ofstream out_log(log_file.c_str(), ofstream::out);
  // out_log << "t,x,y,vd,xyd,nd,d,st" << endl;
//   static ofstream out_log = ofstream(log_file.data(), ios::app);
  static ofstream out_log = ofstream(log_file.data());

  out_log << "t,x,y,yaw,car_s,car_local_s,prev_car_s[1],prev_car_s[2],car_d,prev_car_d[1],prev_car_d[2]" << endl;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const static double nextd = 6.;  
  double max_s = 6945.554;
  // max speed ~ 49.75MPH
  const static double inc_max = 0.4425;
  const static double dist_inc = inc_max;  

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

//added by binliu 170729
  	Road road = Road(SPEED_LIMIT, TRAFFIC_DENSITY, LANE_SPEEDS);
	road.update_width = AMOUNT_OF_ROAD_VISIBLE;	  
	int goal_s = max_s;	
	int goal_lane = 0;	
	int num_lanes = LANE_SPEEDS.size();
	vector<int> ego_config = {SPEED_LIMIT,num_lanes,goal_s,goal_lane,MAX_ACCEL};	
	road.add_ego(0,0, ego_config);	
	Vehicle ego = road.get_ego();
	PolyTrajectoryGenerator PTG;	
	
//end add	  

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&road, &ego, &PTG](uWS::WebSocket<uWS::SERVER> * ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

			int path_size = previous_path_x.size();
			int num_points = 50;			  

          // ###################################################  
          // TEST - Just go straight
          // ###################################################
//          double dist_inc = 0.5;
//          for(int i = 0; i < 50; i++)
//          {
//            next_x_vals.push_back(car_x+(dist_inc*i)*cos(deg2rad(car_yaw)));
//            next_y_vals.push_back(car_y+(dist_inc*i)*sin(deg2rad(car_yaw)));
//          }
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - Just go straight
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^	
		  

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds

			// figure out current lane
			// 0: left, 1: middle, 2: right
			ego.lane = 0;
			if (car_d > 8) ego.lane = 2;
			else if (car_d > 4) ego.lane = 1;

			// update actual position
			ego.set_frenet_pos(car_s, car_d);
			ego.car_x = car_x;
			ego.car_y = car_y;			
			ego.car_yaw = car_yaw;						

			// ###################################################  
			// PATH PLANNING 
			// ###################################################			
			int horizon = 200;
			int update_interval = 50; // update every second
			
			if (path_size < horizon - update_interval) {
				cout << endl;
				cout << "PATH UPDATE" << endl;
				
				// extract surrounding waypoints and fit a spline
				vector<double> waypoints_segment_s;
				vector<double> waypoints_segment_s_worldSpace;
				// TODO: Clean most (all?) of these up! Change signature of fit_spline_segment as well.
				vector<double> map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled, map_waypoints_dx_upsampled, map_waypoints_dy_upsampled;
				tk::spline spline_fit_s_to_x;
				tk::spline spline_fit_s_to_y;
				tk::spline spline_fit_s_to_dx;
				tk::spline spline_fit_s_to_dy;
				fit_spline_segment(car_s, map_waypoints_s, map_waypoints_x, map_waypoints_y, map_waypoints_dx, map_waypoints_dy, waypoints_segment_s, waypoints_segment_s_worldSpace, map_waypoints_x_upsampled, map_waypoints_y_upsampled, map_waypoints_s_upsampled, map_waypoints_dx_upsampled, map_waypoints_dy_upsampled, spline_fit_s_to_x, spline_fit_s_to_y, spline_fit_s_to_dx, spline_fit_s_to_dy);
				
				// convert current car_s into our local Frenet space
				double car_local_s = get_local_s(car_s, waypoints_segment_s_worldSpace, waypoints_segment_s);
				// convert sensor fusion data into local Frenet space
				for (int i = 0; i < sensor_fusion.size(); i++) {
				sensor_fusion[i][5] = get_local_s(sensor_fusion[i][5], waypoints_segment_s_worldSpace, waypoints_segment_s);
				}
				// turn sensor fusion data into Vehicle objects
				vector<Vehicle> envir_vehicles(sensor_fusion.size());
				for (int i = 0; i < sensor_fusion.size(); i++) {
				envir_vehicles[i].set_frenet_pos(sensor_fusion[i][5], sensor_fusion[i][6]);
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double velocity_per_timestep = sqrt(pow(vx, 2) + pow(vy, 2)) / 50.0;
				envir_vehicles[i].set_frenet_motion(velocity_per_timestep, 0.0, 0.0, 0.0);
				}
				
				// get last known car state
				vector<double> prev_car_s = ego.get_s();
				vector<double> prev_car_d = ego.get_d();            
				// collect best guess at current car state. S position in local segment space
				vector<double> car_state = {car_local_s, prev_car_s[1], prev_car_s[2], car_d, prev_car_d[1], prev_car_d[2]};

				// PLAN NEW PATH
	//            cout << "planning path" << endl;				
				vector<vector<double>> new_path = PTG.generate_trajectory(car_state, road.speed_limit, horizon, envir_vehicles);				

            
				out_log << timestep 
					<< "," << ego.car_x << "," << ego.car_y << "," << ego.car_yaw 
					<< "," << car_s << "," <<car_local_s << "," << prev_car_s[1]  << "," << prev_car_s[2] 
					<< "," << car_d	<< "," << prev_car_d[1]  << "," << prev_car_d[2] 
					<< endl;				
			}

          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
          // END - PATH PLANNING
          // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
			//End TODO

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;			

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			timestep++;        			  


          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        (*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> * ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> * ws, int code,
                         char *message, size_t length) {
    (*ws).close();
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
















































































