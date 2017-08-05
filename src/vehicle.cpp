#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(Vehicle::GOCAR_STATE  State) {

    // this->lane = (int)(round(round(State.d - 2.0) / 4.0));      
    memcpy(&goCarState, &State, sizeof(Vehicle::GOCAR_STATE));    
    this->gbLaneChange = false;
}

// Vehicle::Vehicle(VEHICLE_STATE  State){
//     this->lane = (int)(round(round(State.d - 2.0) / 4.0));;
//     this->s = State.s;
//     this->d = State.d;    
//     this->v =  sqrt(pow(State.vx, 2) + pow(State.vy, 2));
//     this->state = "KL";      
// }

Vehicle::Vehicle() {
}

Vehicle::~Vehicle() {
}

