# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

# Project Overview

Goal of this project is to implement Path Planning to plan and generate safe and smooth trajectories for a simulated self-driving car, driving along a 3 lane highway with traffic that is driving +-10 MPH of the 50 MPH speed limit. The simulator provides a feed of values containing the car's localization, sensor fusion data of around vehicles on right hand side of the road and a sparse map list of waypoints around the highway. 

Desigh Requirements are as follows: 

* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes.

# Final Result
[Video](https://youtu.be/xWD0j_8Z6gg) Using this path planner, our car successfully drives around the track for 4.32 Miles with 6 minutes 19 seconds. 

![pathplanner5](https://user-images.githubusercontent.com/24623272/29002135-5933af78-7ace-11e7-8e9a-8fee53692b5f.png)

### The Path Planner

#### 1. Prediction
Behaviour prediction is not implemented in the Path Planner, as an extensive behaviour prediction is not necessary for a simple highway driving scenario. But this is necessary in a more complex environment，eg. urban driving scenes.

#### 2. Behaviour Planning
Behaviour prediction, as can be seen in src/PathPlanner.cpp lines 311-354 is a simple finite state machine to decide when to change lanes. Which use the lane score algorithm from Mohan Karthik. The algorithm mainly do the following two things.

1. Estimating a score for each lane, as can be seen in src/vehicle.cpp lines 106-170
, to determine the best lane for us to be in (efficiency)

2. Evaluating the feasibility of moving to that lane in the immediate future (safety & comfort),as can be seen in src/vehicle.cpp lines 180-300

##### Ranking Lanes
The lane score algorithm rank lanes are using the following 3 factors
1. The lesser the number of lanes we need to change, the better. Because the lesser we change lanes, the more comfortable the drive is for the passengers (takes care of comfort).

2. The distance of the car ahead of us in that lane. The more the distance, the better the score.

3. The velocity of the car ahead of us in that lane. The greater the velocity, the faster we can travel in that lane, before being forced to change lane again.

#### 3. Trajectory Generation
Trajectory Generation, as can be seen in src/PathPLanner.cpp lines 194-306 where its path planning decision are put into practice. 

1. The Udaity simulator gives us the previous path after removing all consumed points in the last time cycle.

2. Inspired from John Chen, converting all coordinates from global coordinate system to local car’s coordinate system to make things easier.

3. Create a spline of the nearest surrounding waypoints (saying total 25 waypoints, 6 before , 1 nearest and 18 next waypoints) in the car’s reference frame. The waypoints must be converted to the car’s reference.

4. Create another spline with the previous path as the starting points, Then add points from the waypointSpline into this.

5. Create another spline to smoothen the velocity changes from the current velocity 
to the destination velocity (s/time). This ensures that we can follow our ideal trajectory without violating jerk / speed constraints.

6. Convert these points back to the world coordinates and feed them back to the simulator.

### Future Work
This is a simplified path planner solution. There is still a lot of things to improve it.

1. Add in behaviour prediction using multiple-model estimators that will allow the system to react much better & faster to the fine-grain dynamic changes.

2. Design and implement new cost functions for vehicle behavior and lane states. 

3. Polished these cost functions and make them cooperate efficiently.


### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
