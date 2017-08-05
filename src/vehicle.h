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
#include "header.h"

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

//    /*! Other vehicle's current parameters */
//     typedef struct sensor_fusion_state
//     {
//         int vehicle_id;
//         double x;
//         double y;  
//         double vx;      
//         double vy;
//         double s;              
//         double d;                
//     } VEHICLE_STATE;    

//     /*! Predicted vehicle's current parameters */
//     typedef struct predicted_state
//     {
//         double s;
//         double s_dot;  
//         double xxx1;      
//         double d;
//         double xxx2;              
//         double xxx3;                
//     } PREDICTED_STATE;     

  int lane;
  GOCAR_STATE goCarState;
  bool gbLaneChange;

  /**
  * Constructor
  */
  
  Vehicle(GOCAR_STATE  State);
//   Vehicle(VEHICLE_STATE State);
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

//   // void update_state(map<int, vector <vector<int> > > predictions);
//   void update_state(map<int, PREDICTED_STATE>  predictions) ;

//   void configure(vector<double> road_data);

//   void increment(int dt);

//   // vector<int> state_at(int t);
//   vector<double> state_at(double t) const;

//   bool collides_with(Vehicle other, int at_time);

//   collider will_collide_with(Vehicle other, int timesteps);

//   void realize_state(map<int, vvd_t> predictions);

//   void realize_constant_speed();

//   // int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
//   double _max_accel_for_lane(map<int,vvd_t > predictions, int lane, double s) ;

//   void realize_keep_lane(map<int, vvd_t> predictions);

//   void realize_lane_change(map<int,vvd_t > predictions, string direction);

//   void realize_prep_lane_change(map<int,vvd_t > predictions, string direction);
  
//   void realize_car_starting();

//   vector<vector<double> > generate_predictions(int horizon);

//   vector<string> get_successor_states();

//   //added by binliu 170729
//   vector<double> get_s() const;
//   vector<double> get_d() const;  
//   void set_frenet_pos(double pos_s, double pos_d); 
//   void set_frenet_motion(double vel_s, double acc_s, double vel_d, double acc_d);   
//   //end add

private:


};

#endif