#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>
#include <algorithm>

using namespace std;

class Vehicle {
public:

    /*! The car's current parameters */
    typedef struct car_state
    {
        double x;
        double y;
        double s;
        double d;
        double yaw_d;
        double yaw_r;
        double v;
    } GOCAR_STATE;

    /*! The car's current parameters */
    typedef struct sensor_fusion_state
    {
        double s;
        double s_dot;  
        double xxx1;      
        double d;
        double xxx2;              
        double xxx3;                
    } VEHICLE_STATE;    

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  // int s;
  double s;
  double d;

  int v;

  int a;

  int target_speed;

  int lanes_available;

  int max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  //added by binliu 170729
  double car_x ;
  double car_y ;
  double car_s ;
  double car_d ;
  double car_yaw ;
  double car_speed ;
  //end add

  /**
  * Constructor
  */
  
  Vehicle(int lane, double s, double v, double a);
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  // void update_state(map<int, vector <vector<int> > > predictions);
  void update_state(map<int, VEHICLE_STATE>  predictions) ;

  void configure(vector<double> road_data);

  // string display();

  void increment(int dt);

  // vector<int> state_at(int t);
  vector<double> state_at(double t) const;

  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector < vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector< vector<int> > > predictions);

  void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);

  void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  
  void realize_car_starting();

  vector<vector<double> > generate_predictions(int horizon);

  vector<string> get_successor_states();

  //added by binliu 170729
  vector<double> get_s() const;
  vector<double> get_d() const;  
  void set_frenet_pos(double pos_s, double pos_d); 
  void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);   
  //end add

  //added by binliu 170729
private:
  //double _pos_x;
  //double _pos_y;
  double _pos_s;
  double _pos_d;
  //double _vel_x;
  //double _vel_y;
  double _vel_s;
  double _vel_d;
  //double _acc_x;
  //double _acc_y;
  double _acc_s;
  double _acc_d;
  //end add  

};

#endif