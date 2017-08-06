#include "PathPlanner.h"


    /*!
    * @brief: Constructor to the PathPlanner class
    *
    * @param [in] oState: The state of the car
    * @param [in] prev_path: The previous path so far
    * @param [in] sensor_fusion: The sensor fusion data
    */
    PathPlanner::PathPlanner(const WP_MAP Map)
    {
        /* Save the waypoints */
        goMap = Map;

        /* Save the size of the waypoints */
        gnMapSz = goMap.x.size();   

        goCar = Vehicle(); 
    }

    /*!
    * @brief: Destructor
    */
    PathPlanner::~PathPlanner() {}

    /*!
    * @brief: Plans a path based on the current information
    *
    * @param [in] oState: The state of the car
    * @param [in] prev_path: The previous path so far
    * @param [in] sensor_fusion: The sensor fusion data
    *
    * @returns: A path of {{x's}, {y's}} for the car to drive
    */
    vvd_t PathPlanner::GeneratePathPlan(Vehicle::GOCAR_STATE &State, vvd_t &PrevPath, vvd_t &SensorFusion)
    {
        vvd_t vvResult;

        /* Save the current values */
        memcpy(&goCar.goCarState, &State, sizeof(Vehicle::GOCAR_STATE));                                
        gvvPrPath = PrevPath;
        gvvSenFus = SensorFusion;

        /* Save the previous path size */
        gnPrPathSz = gvvPrPath[0].size();
        REQUIRE(gnPrPathSz < NUM_POINTS)

        /* Save the sensor fusion size */
        gnSenFusSz = gvvSenFus.size();

        /* Get the current lane */
        goCar.lane = (int)(round(round(State.d - 2.0) / 4.0));

        /* Setup a lane spline */
        spline hLaneSpline;
        CreateLaneSpline(hLaneSpline);

        /* Setup a velocity spline */
        spline hVelocitySpline;
        CreateVelocitySplineFirstCycle(hVelocitySpline);     

        // Tools::traceStream << "timestep:" <<gnTimeStep << " " <<"previous path size: " << gnPrPathSz << endl;                                               

        /* If this is car starting ---  no paths */
        if (gnPrPathSz == 0)
        {
            HandleFirstCycle(hLaneSpline, hVelocitySpline, vvResult);
        }
        /* If this is car running --- keep lane */        
        else
        {
            HandleGenericCycle(hLaneSpline, vvResult);
        }

        /* Invoke the planner if there is no lane change in progress */
        if (goCar.gbLaneChange == false)
        {
            // Tools::traceStream << "Start BehaviorPlanner()" << endl;                        
            BehaviourPlanner();    

        }

        /* Increment the timestep */
        gnTimeStep++;

        return vvResult;
    }



    /*!
    * Computes a lane tracking spline in local car co-ordinates
    */
    // void PathPlanner::TrackLane(spline &hLaneSpline)
    void PathPlanner::CreateLaneSpline(spline &hLaneSpline)    
    {
        /* Get the surronding waypoints in local co-ordinates */
        vvd_t vvLocalWP = getLocalWPSeg();

        /* wrong way! */
        if (vvLocalWP[0][0] > 0.0) 
        {
            goCar.goCarState.yaw_d += 180.0;
            vvLocalWP = getLocalWPSeg();
        }
        
        hLaneSpline.set_points(vvLocalWP[0], vvLocalWP[1]);
    }

    /*!
    * Computes a velocity tracking spline for the first
    * cycle
    */
    // void PathPlanner::TrackVelocityFirst(spline &hVelocitySpline)
    void PathPlanner::CreateVelocitySplineFirstCycle(spline &hVelocitySpline)    
    {
        vd_t vTime, vDist;

        vTime.pb(-1.0);
        vTime.pb(double(NUM_POINTS * 0.3));
        vTime.pb(double(NUM_POINTS * 0.5));
        vTime.pb(double(NUM_POINTS * 1.0));
        vTime.pb(double(NUM_POINTS * 2.0));

        vDist.pb(goCar.gnNextS * 0.01);
        vDist.pb(goCar.gnNextS * 0.10);
        vDist.pb(goCar.gnNextS * 0.15);
        vDist.pb(goCar.gnNextS * 0.25);
        vDist.pb(goCar.gnNextS * 0.30);
        vDist.pb(goCar.gnNextS * 0.50);  

        /* Form the spline */
        hVelocitySpline.set_points(vTime, vDist);
              
    }

    /*!
    * Handles the first cycle
    */
    void PathPlanner::HandleFirstCycle(spline &hLaneSpline,spline &hVelocitySpline, vvd_t &vvResult)
    {
        vd_t vLocalX;
        vd_t vLocalY;
        double dLocalX = 0.0;
        double dLocalY = 0.0;

        /* Form a smooth localized lane using both velocity & lane splines */
        for (int i = 0; i < NUM_POINTS; i++)
        {
            dLocalX += hVelocitySpline(double(i));
            vLocalX.pb(dLocalX);
            vLocalY.pb(hLaneSpline(dLocalX));
        }

        /* Calculate the smoother path by smoothening the velocities further */
        dLocalX = 0.0;
        for(int i = 0; i < NUM_POINTS; i++)
        {
            /* Compute the distance & the intended speed */
            const double dDist = distance(dLocalX, dLocalY, vLocalX[i], vLocalY[i]);
            const double dSpeed = hVelocitySpline(double(i));

            /* If the actual distance is too different from the intended speed */
            if ((dDist < (dSpeed * 0.8) || (dDist > dSpeed)))
            {
                /* Smoothen the path using the heading */
                const double dHeading = atan2((vLocalY[i] - dLocalY), (vLocalX[i] - dLocalX));
                vLocalX[i] = dLocalX + hVelocitySpline(double(i)) * cos(dHeading);
                vLocalY[i] = hLaneSpline(vLocalX[i]);
            }

            /* Save the velocity */
            gvvVelHist.pb(distance(dLocalX, dLocalY, vLocalX[i], vLocalY[i]));

            /* Update the locals for the next round */    
            dLocalX = vLocalX[i];
            dLocalY = vLocalY[i];
        }

        /* Set the current velocity */
        goCar.gnNextS = hVelocitySpline(NUM_POINTS);

        /* Convert these points to world points */
        vvResult = getWorldPoints(vLocalX, vLocalY);

        /* Initialize the path history with these points */
        gvvPathHist = vvResult;
    }

    /*!
     * Handles the generic cycle case
     */
    void PathPlanner::HandleGenericCycle(spline &hLaneSpline, vvd_t &vvResult)
    {
        /* Get the localized previous path */
        vvd_t vvLPath = getLocalPoints(gvvPrPath[0], gvvPrPath[1]);
        REQUIRE(vvLPath[0].size() == gnPrPathSz)
        REQUIRE(vvLPath[1].size() == gnPrPathSz)

        /* Erase the consumed waypoints of the previous history */
        gvvVelHist.erase(gvvVelHist.begin(), gvvVelHist.begin() + (NUM_POINTS - gnPrPathSz));
        (gvvVelHist.begin(), gvvVelHist.begin() + (NUM_POINTS - gnPrPathSz));
        gvvPathHist[0].erase(gvvPathHist[0].begin(), gvvPathHist[0].begin() + (NUM_POINTS - gnPrPathSz));
        gvvPathHist[1].erase(gvvPathHist[1].begin(), gvvPathHist[1].begin() + (NUM_POINTS - gnPrPathSz));

        /* Check if we are done changing lanes */
        if ((goCar.gbLaneChange == true) && (goCar.goCarState.d >= (goCar.gnNextD - FLOAT_EPS)) && (goCar.goCarState.d <= (goCar.gnNextD + FLOAT_EPS)))
        {
            goCar.gnLaneChangeVotes = 0;
            goCar.gbLaneChange = false;
        }

        /*** PATH ***/
        /* Setup another lane tracker to include the previous path */
        spline hNewLane;
        CreateLaneSpline(hNewLane);

        /* Form a spline including the previous path */
        vd_t vLocalX;
        vd_t vLocalY;
        for (int i = 0; i < gnPrPathSz; i++) 
        {
          vLocalX.pb(vvLPath[0][i]);
          vLocalY.pb(vvLPath[1][i]);
        }

        /* Add the next set of points based on the distance increments
        set previously */
        double nextx = vvLPath[0][gnPrPathSz - 1] + 40;
        for (int i = 0; i < gnPrPathSz; i++)
        {
          vLocalX.pb(nextx);
          vLocalY.pb(hNewLane(nextx));
          nextx += goCar.gnNextS;
        }

        /* Reform the original lane spline */
        hLaneSpline.set_points(vLocalX, vLocalY);

        /*** VELOCITY ***/
        /* Setup a velocity tracker */
        vd_t vTime;
        vd_t vDist;

        for (int i = 0; i < gnPrPathSz; i++)
        {
            vTime.pb(double(i));
            vDist.pb(gvvVelHist[i]);
        }
        vTime.pb(double(NUM_POINTS * 5.0));
        vDist.pb(goCar.gnNextS);

        spline hVelocitySpline;
        hVelocitySpline.set_points(vTime, vDist);

        /* Fill up the local path by interpolating from the previous path,
        using the velocity & the path splines */
        for(int i = gnPrPathSz; i < NUM_POINTS; i++) 
        {
            vvLPath[0].pb(vvLPath[0][i - 1] + hVelocitySpline(double(i)));
            vvLPath[1].pb(hLaneSpline(vvLPath[0][i]));
        }
        REQUIRE(vvLPath[0].size() == NUM_POINTS)
        REQUIRE(vvLPath[1].size() == NUM_POINTS)

        /* Form a smoother path */
        double dLocalX = vvLPath[0][0];
        double dLocalY = vvLPath[1][0];
        for(int i = 0; i < NUM_POINTS; i++)
        {
            const double dist = distance(dLocalX, dLocalY, vvLPath[0][i], vvLPath[1][i]);
            if (dist > hVelocitySpline(double(i)))
            {
                const double dHeading = atan2((vvLPath[1][i] - dLocalY), (vvLPath[0][i] - dLocalX));
                vvLPath[0][i] = dLocalX + (hVelocitySpline(double(i)) * cos(dHeading));
                vvLPath[1][i] = hLaneSpline(vvLPath[0][i]);
            }
            if (i >= gnPrPathSz)
            {
                gvvVelHist.push_back(distance(dLocalX, dLocalY, vvLPath[0][i], vvLPath[1][i]));
            }
            
            dLocalX = vvLPath[0][i];
            dLocalY = vvLPath[1][i];
        }

        /* Convert these points to world points */
        vvd_t vvWorldPts = getWorldPoints(vvLPath[0], vvLPath[1]);
        REQUIRE(vvWorldPts[0].size() == NUM_POINTS)
        REQUIRE(vvWorldPts[1].size() == NUM_POINTS)

        /* Make the final structure, start with the previous path */
        vvResult = gvvPrPath;
        for (int i = gnPrPathSz; i < vvWorldPts[0].size(); i++)    
        {
            vvResult[0].pb(vvWorldPts[0][i]);
            vvResult[1].pb(vvWorldPts[1][i]);
        }
        REQUIRE(vvResult[0].size() == NUM_POINTS)
        REQUIRE(vvResult[1].size() == NUM_POINTS)

        /* Save the history */
        gvvPathHist.clear();
        gvvPathHist = vvResult;
    }

    /*!
     * Behaviour planner
     */
    void PathPlanner::BehaviourPlanner(void)
    {
        vvvd_t vvvLanes(SIM_NUM_LANES);

        for (int i = 0; i < gnSenFusSz; i++) 
        {
            vd_t vVehicle = gvvSenFus[i];

            /* Add the computed values into the sensor fusion structure */
            /* Dist increments (velocity) of the car */
            gvvSenFus[i].pb((distance(0.0, 0.0, vVehicle[3], vVehicle[4]) * SIM_TIME_SLICE));

            /* Displacement of other car from ours */
            gvvSenFus[i].pb(vVehicle[5] - goCar.goCarState.s);

            /* Add the cars into the corresponding lanes */
            for (int j = 0; j < SIM_NUM_LANES; j++)
            {
                if ((vVehicle[6] >= ((j * SIM_LANE_WD) - LANE_BUFFER)) && (vVehicle[6] <= (((j + 1) * SIM_LANE_WD) + LANE_BUFFER)))
                {
                    vvvLanes[j].pb(gvvSenFus[i]);
                }
            }
        }

        /* Sort the lanes */
        for (int i = 0; i < SIM_NUM_LANES; i++)
        {
            /* Sort based on the distance */
            sort(vvvLanes[i].begin(), vvvLanes[i].end(),[](const std::vector<double>& a, const std::vector<double>& b) 
            {
                return a[8] < b[8];
            });
        }

        /* Rank the lanes */
        vi_t vLaneRanks;
        vvi_t vvCloseCars;

        goCar.update_state(vvvLanes, vvCloseCars, vLaneRanks);

        /* Change lanes if feasible */
        goCar.LaneChange(vvvLanes, vvCloseCars, vLaneRanks);
    }



    /*!
     * @brief: Finds the closest waypoint index to the car, regardless of the direction
     *
     * @return: The index of the closest waypoint to the car
     */
    int PathPlanner::ClosestWaypoint(void)
    {
        double dLen = numeric_limits<double>::infinity();
        int nWP = 0;

        /* Loop through all the waypoints and find the closest one */
        for(int i = 0; i < gnMapSz; i++) 
        {
            const double dDist = distance(goCar.goCarState.x, goCar.goCarState.y, goMap.x[i], goMap.y[i]);
            if(dDist < dLen) 
            {
                dLen = dDist;
                nWP = i;
            }
        }
        return nWP;
    }

    /*!
     * @brief: Get's the next waypoint on the car's path
     *
     * @return: The index of the next waypoint
     */
    int PathPlanner::NextWaypoint(void) 
    {
        /* Get the closest waypoint */
        int nWP = ClosestWaypoint();

        /* Compute the heading of the car relative to the closest waypoint */
        const double dHeading = atan2((goMap.y[nWP] - goCar.goCarState.y), (goMap.x[nWP] - goCar.goCarState.x));

        /* If the car is not heading towards the next waypoint (i.e: it's behind us), then choose
        the next one instead */
        const double dAngleDiff = abs(goCar.goCarState.yaw_r - dHeading);
        if(dAngleDiff > (M_PI / 4.0))
        {
            nWP++;

            /* Loop around if required */
            if (nWP >= gnMapSz)
            {
                nWP = 0;
            }
        }

        return nWP;
    }

    /*! 
     * @brief: Transform from world cartesian x,y coordinates to Frenet s,d coordinates
     *
     * @return: The corresponding frenet co-ordinates as {s, d}
     */
    vd_t PathPlanner::getFrenet(void)
    {
        /* Get the next & previous way points */
        int nNextWP = NextWaypoint();
        int nPrevWP;
        if(nNextWP == 0) 
        {
            nPrevWP  = gnMapSz - 1;
        }
        else
        {
            nPrevWP = nNextWP - 1;
        }

        /* Compute the projection n */
        const double dNX = goMap.x[nNextWP] - goMap.x[nPrevWP];
        const double dNY = goMap.y[nNextWP] - goMap.y[nPrevWP];
        const double dXX = goCar.goCarState.x - goMap.x[nPrevWP];
        const double dXY = goCar.goCarState.y - goMap.y[nPrevWP];

        /* find the projection of x onto n */
        const double dProjNorm = (((dXX * dNX) + (dXY * dNY)) / ((dNX * dNX) + (dNY * dNY)));
        const double dProjX = dProjNorm * dNX;
        const double dProjY = dProjNorm * dNY;

        /* Compute the d */
        double dFrenetD = distance(dXX, dXY, dProjX, dProjY);

        /* See if d value is positive or negative by comparing it to a center point */
        const double dCenterX = 1000.0 - goMap.x[nPrevWP];
        const double dCenterY = 2000.0 - goMap.y[nPrevWP];
        const double dCenterToPos = distance(dCenterX, dCenterY, dXX, dXY);
        const double dCenterToRef = distance(dCenterX, dCenterY, dProjX, dProjY);

        /* If we are on the other side */
        if(dCenterToPos <= dCenterToRef) 
        {
            dFrenetD *= -1.0;
        }

        /* calculate s value */
        double dFrenetS = 0.0;
        for(int i = 0; i < nPrevWP; i++) 
        {
            dFrenetS += distance(goMap.x[i], goMap.y[i], goMap.x[i+1], goMap.y[i+1]);
        }
        dFrenetS += distance(0.0, 0.0, dProjX, dProjY);

        /* Return the values */
        return {dFrenetS, dFrenetD};
    }


    /*! 
     * @brief: Transform from global Cartesian x,y to local car coordinates x,y
     * where x is pointing to the positive x axis and y is deviation from the car's path
     * @param [in] dX, dY: The world point to be projected onto the car co-ordinate system
     *
     * @return: {x,y}, the point (dX, dY) in the car co-ordinate system.
     */
    vd_t PathPlanner::getLocalXY(const double dX, const double dY)
    {
        vd_t vResults;

        const float dDeltaX = (dX - goCar.goCarState.x);
        const float dDeltaY = (dY - goCar.goCarState.y);

        vResults.push_back((dDeltaX  * cos(goCar.goCarState.yaw_r)) + (dDeltaY * sin(goCar.goCarState.yaw_r)));
        vResults.push_back((-dDeltaX * sin(goCar.goCarState.yaw_r)) + (dDeltaY * cos(goCar.goCarState.yaw_r)));

        return vResults;
    }

    /*!
     * @brief: Transforms from the local car cordinates to world co-ordinate system
     *
     * @param [in] dX, dY: The local car point to be projected onto the world co-ordinate system
     *
     * @return: {x,y}, the point (dX, dY) in the world co-ordinate system.
     */
    vd_t PathPlanner::getWorldXY(const double dX, const double dY)
    {
        vd_t results;

        results.push_back((dX * cos(goCar.goCarState.yaw_r)) - (dY * sin(goCar.goCarState.yaw_r)) + goCar.goCarState.x);
        results.push_back((dX * sin(goCar.goCarState.yaw_r)) + (dY * cos(goCar.goCarState.yaw_r)) + goCar.goCarState.y);

        return results;
    }

    /*! 
     * @brief: Returns a set of waypoints around the car, and returns them in the
     * car co-ordinate system.
     *
     * @result: A 2d vector of {{x's}, {y's}} of waypoints localized to the car
     * co-ordinates
     */
    vvd_t PathPlanner::getLocalWPSeg(void)
    {
        vd_t vWpX;
        vd_t vWpY;
        vvd_t vvResults;

        /* Get the farthest past waypoint on the spline */
        int nWp = ClosestWaypoint();
        int nPrevWP = nWp - WP_SPLINE_PREV;
        if (nPrevWP < 0) 
        {
            nPrevWP += gnMapSz;
        }
        // Tools::traceStream << "getLocalWPSeg " << "line 746: " << "ClosestWp nWp=" << nWp << " WP_SPLINE_PREV =" << WP_SPLINE_PREV <<" nPrevWP=" << nPrevWP << " gnMapSz=" << gnMapSz << endl;                    
        /* Convert the waypoints into localaized points */
        for (int i = 0; i < WP_SPLINE_TOT; i++) 
        {
            const int nNextWP = (nPrevWP + i) % gnMapSz;
            const vd_t localxy = getLocalXY((goMap.x[nNextWP] + (goCar.gnNextD * goMap.dx[nNextWP])), (goMap.y[nNextWP] + (goCar.gnNextD * goMap.dy[nNextWP])));

            vWpX.push_back(localxy[0]);
            vWpY.push_back(localxy[1]);
            // Tools::traceStream << "getLocalWPSeg " << "line 753: " << "i=" << i << " nNextWP =" << nNextWP  << endl;                    

        }

        vvResults.push_back(vWpX);
        vvResults.push_back(vWpY);

        return vvResults;
    }

    /*!
     * @brief: Convert a set of world x,y vector coordinates to local x y vectors
     *
     * @param [in] vdX, vdY: A set of world points to be projected onto the car co-ordinate system
     *
     * @return: {{x's},{y's}}, the points {{vdX's}, {vdY's}} in the car co-ordinate system.
     */
    vvd_t PathPlanner::getLocalPoints(const vd_t vdX, const vd_t vdY) 
    {
        vd_t vLocalX;
        vd_t vLocalY;
        vvd_t vvResults;

        const int sz = vdX.size();

        /* Loop around and push the points in */
        for (int i = 0; i < sz; i++) 
        {
            const vd_t localxy = getLocalXY(vdX[i], vdY[i]);
            vLocalX.push_back(localxy[0]);
            vLocalY.push_back(localxy[1]);
        }
        vvResults.push_back(vLocalX);
        vvResults.push_back(vLocalY);

        return vvResults;
    }

    /*!
     * @brief: Convert a set of local x,y vector coordinates to world x y vectors
     *
     * @param [in] car_x, car_y: The car's (x,y) in world co-ordinates
     * @param [in] car_yaw_d: The car's heading
     * @param [in] lx, ly: A set of car points to be projected onto the world co-ordinate system
     *
     * @return: {{x's},{y's}}, the points {{lx's}, {ly's}} in the world co-ordinate system.
     */
    vvd_t PathPlanner::getWorldPoints(const vd_t vdX, const vd_t vdY) 
    {
        vd_t vWorldX;
        vd_t vWorldY;
        vvd_t vvResults;

        /* Store the size */
        const int sz = vdX.size();

        /* Loop around and push the points in */
        for (int i = 0; i < sz; i++) 
        {
            const vd_t worldxy = getWorldXY(vdX[i], vdY[i]);
            vWorldX.push_back(worldxy[0]);
            vWorldY.push_back(worldxy[1]);
        }
        vvResults.push_back(vWorldX);
        vvResults.push_back(vWorldY);

        return vvResults;
    }
