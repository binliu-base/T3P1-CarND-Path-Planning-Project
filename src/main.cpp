// #include "header.h"
#include "PathPlanner.h"

// trace_tag = true;
string Tools::tracelog =  "../data/logger.csv";
// ofstream Tools::traceStream = ofstream(Tools::tracelog.data(), ios::app);
ofstream Tools::traceStream = ofstream(Tools::tracelog.data());

// for convenience
using json = nlohmann::json;

static void readWPFile(PathPlanner::WP_MAP &Map);

static int timestep = 0;

int main() {
  uWS::Hub h;

  // Tools::traceStream << "t,x,y,yaw,car_s,car_local_s,prev_car_s[1],prev_car_s[2],car_d,prev_car_d[1],prev_car_d[2]" << endl;
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
	// /* Read in the waypoint file */
	PathPlanner::WP_MAP WpMap;
	readWPFile(WpMap);

	/* Initialize the path planner */
	PathPlanner planner = PathPlanner(WpMap);

  // h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> *ws, char *data, size_t length,
  //                    uWS::OpCode opCode) {
	h.onMessage([&planner](uWS::WebSocket<uWS::SERVER> * ws, char *data, size_t length, uWS::OpCode opCode) 
	{		
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

						/* Get the previous path */
						vvd_t vvPrevPath = 
						{
								j[1]["previous_path_x"],
								j[1]["previous_path_y"]
						};

						/* Sensor Fusion Data, a list of all other cars on the same side of the road. */
						vvd_t vvSenFus = j[1]["sensor_fusion"];          

        	// SDC car's localization Data
          	double car_yaw = j[1]["yaw"];

						/* Get the car state */
						Vehicle::GOCAR_STATE oCarState = 
						{
								j[1]["x"],
								j[1]["y"],
								j[1]["s"],
								j[1]["d"],
								car_yaw,
								deg2rad(car_yaw),
								j[1]["speed"]
						};

						/* Call the path planner */
						vvd_t vvPathPlan = planner.GeneratePathPlan(oCarState, vvPrevPath, vvSenFus);
						
						/* Create the JSON */
          	json msgJson;

          	// vector<double> next_x_vals;
          	// vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	// msgJson["next_x"] = next_x_vals;
          	// msgJson["next_y"] = next_y_vals;
						msgJson["next_x"] = vvPathPlan[0];
						msgJson["next_y"] = vvPathPlan[1];						

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	(*ws).send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            timestep++;
            Tools::traceStream << "timestep=" << timestep << endl;            
          
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

//   h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> *ws, uWS::HttpRequest req) {	
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> *ws, int code,
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

/*!
 * Reads in the waypoint file
 */
static void readWPFile(PathPlanner::WP_MAP &Map)
{
    string map_file_ = WP_FILE;
    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) 
    {
        istringstream iss(line);
        double x;
        double y;
        double s;
        double d_x;
        double d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        Map.x.pb(x);
        Map.y.pb(y);
        Map.s.pb(s);
        Map.dx.pb(d_x);
        Map.dy.pb(d_y);
    }
}
