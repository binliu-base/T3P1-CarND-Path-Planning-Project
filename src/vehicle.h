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

  int lane;
  string state = KL;
  GOCAR_STATE goCarState;
  bool gbLaneChange = false;
  /*! The value of distance increment per time step */
  double gnNextS = MAX_DIST_INC;
  /*! The next d value */
  double gnNextD = 6.0;  

  /*! Votes for lane change */
  int gnLaneChangeVotes = 0;  

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

  void update_state(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vLaneRanks);

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

  vector<string> get_successor_states();
  void FindClosestCars(const vvvd_t &vvvLanes, vvi_t &vvResult);
  // void RankLanes(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vResult);
  void RankLanes(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vResult, map<int,double> &vScoresMap);  
  void LaneChange(const vvvd_t &vvvLanes, const vvi_t &vvCars, const vi_t &vRanks);


private:


};

#endif