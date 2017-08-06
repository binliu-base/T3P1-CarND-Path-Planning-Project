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
}

Vehicle::Vehicle() {
}

Vehicle::~Vehicle() {
}

vector<string> Vehicle::get_successor_states() {

    vector<string> successor_states;
    
    if (state.compare(KL) == 0) {
        successor_states.push_back(KL);
        successor_states.push_back(LCL);
        successor_states.push_back(LCR);
    }
    else if (state.compare(LCR) == 0) {
        successor_states.push_back(KL);
        successor_states.push_back(LCL);
        successor_states.push_back(LCR);
    }
    else if (state.compare(LCL) == 0) {
        successor_states.push_back(KL);
        successor_states.push_back(LCL);
        successor_states.push_back(LCR);
    }
    if (this->lane == 0) {
        successor_states.erase(std::remove(successor_states.begin(), successor_states.end(), LCL), successor_states.end());
    }
    
    if (this->lane == 2) {
        successor_states.erase(std::remove(successor_states.begin(), successor_states.end(), LCR), successor_states.end());
    }

    return successor_states;    
}

void Vehicle::update_state(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vLaneRanks) {

    vector<string> successor_states = get_successor_states(); 
    map<string,double> costs;
    map<int,double> vScoresMap;          

    RankLanes(vvvLanes, vvCars, vLaneRanks, vScoresMap);

    costs[KL] = 0.0;
    costs[LCL]  = 0.0;
    costs[LCR]  = 0.0;

    costs[KL] = vScoresMap[lane];

    if (lane == 0 ){
        costs[LCR]  = vScoresMap[lane+1];
    }
    else if(lane == 1){
        costs[LCL]  = vScoresMap[lane-1];
        costs[LCR]  = vScoresMap[lane+1];
    }
    else{
        costs[LCL]  = vScoresMap[lane-1];
    }

    double max_score = 0;

    string next_state ;

    for (auto successor_state: successor_states) {
          
         double successor_cost = costs[successor_state];
         cout << "successor_state: " << successor_state << " " << "successor_score:" << successor_cost << endl;
         if (successor_cost > max_score){
            next_state = successor_state;
            max_score = successor_cost;
         }
    }
    if (gnLaneChangeVotes > MIN_LC_VOTES)
    {
        // printf ("%s %s \n","Next State of Our Car:", state.c_str());                      
        state = next_state;
    }     

    cout << endl;    
    printf ("%s %s \n", "next state of our car: ", next_state.c_str());                          

}

    /*!
     * Ranks the lanes based on the car ahead
     */
    void Vehicle::RankLanes(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vResult, map<int,double> &vScoresMap)
    {
        vector<pair<double,int>> vScores;           

        /* Find the cars closest to us in all the lanes */
        FindClosestCars(vvvLanes, vvCars);

        /* Compute the lane scores */
        vResult.resize(SIM_NUM_LANES);
        int clean = system("clear");

        // printf ("%s %s  %s  %s  %s\n","Lane", "Scores","LCS","DS","VS" );        
        for (int i = 0; i < SIM_NUM_LANES; i++)
        {
            double nLCS, nDS, nVS;

            /* Lane change */
            nLCS = BEH_LANE_SCR * (1.0 - (fabs(i - this->lane) / (SIM_NUM_LANES - 1)));

            /* Distance to ahead car */
            if (vvCars[i][1] == -1)
            {
                nDS = BEH_DIST_SCR;
            }
            else
            {
                nDS = BEH_DIST_SCR * (1.0 - ((MAX_VEH_GAP - vvvLanes[i][vvCars[i][1]][8]) / MAX_VEH_GAP));
            }

            /* Velocity cost */
            if (vvCars[i][1] == -1)
            {
                nVS = BEH_VEL_SCR;
            }
            else
            {
                nVS = BEH_VEL_SCR * (1.0 - (((MAX_DIST_INC * 2.0) - vvvLanes[i][vvCars[i][1]][7]) / (MAX_DIST_INC * 2.0)));
            }
            // printf("%d:\t", i);
            // printf("%.2f \t", (nLCS + nDS + nVS));
            // printf("%.2f \t", nLCS);
            // if (vvCars[i][1] == -1)
            // {
            //     printf("NA \t %.2f \t", nDS);
            //     printf("NA \t %.2f \n", nVS);
            // }
            // else
            // {
            //     printf("%.2f \t %.2f \t", vvvLanes[i][vvCars[i][1]][8], nDS);
            //     printf("%.2f \t %.2f \n", vvvLanes[i][vvCars[i][1]][7], nVS);
            // }

            /* Add it in */
            vScores.pb(make_pair((nLCS + nDS + nVS), i));
            vScoresMap[i] = nLCS + nDS + nVS;          
        }

        /* Sort the scores */
        sort(vScores.begin(), vScores.end());

        /* Get the ranks */
        for (int i = 0; i < SIM_NUM_LANES; i++)
        {
            vResult[i] = vScores[SIM_NUM_LANES - i - 1].second;
        }
    }




    /*! Checks which of the lanes (including the current one)
     * is most feasible in the order of their rankings, and if
     * it's feasible, initiates the change
     */
    void Vehicle::LaneChange(const vvvd_t &vvvLanes, const vvi_t &vvCars, const vi_t &vRanks)
    {
        int nDestLane = lane;

        for (int i = 0; i < SIM_NUM_LANES; i++)
        {
            /* Get the lane number */
            int nLane = vRanks[i];

            /* If the best lane is the current lane, then nothing to do*/ 

            if (nLane == lane)
            {
                gnLaneChangeVotes = 0;

                /* Nothing to do */
                break;
            }

            /* Find out how many lane shifts are we talking about to the
            best lane */
            int nChanges = nLane - lane;
            int nDir = nChanges / abs(nChanges);

            /* Check feasibility */
            bool bFeasible = true;

            /* If we are travelling too fast, then a multiple lane change might
            cause too much jerk */
            if ((goCarState.v >= 40.0) && (abs(nChanges) > 1))
            {
                bFeasible = false;
            }

            /* Check if in the series of intermediate & destination lanes,
            if there are no cars immediately behind us */
            for (int i = 1; i <= abs(nChanges); i++)
            {
                /* Can we change the lane */
                const int nTempLane = lane + (i * nDir);
                const int nCarIdxBk = vvCars[nTempLane][0];
                const int nCarIdxFr = vvCars[nTempLane][1];
                if (nCarIdxBk != -1)
                {
                    const double nDist = abs(vvvLanes[nTempLane][nCarIdxBk][8]);
                    const double nVel = vvvLanes[nTempLane][nCarIdxBk][7];
                    if (((nVel < gnNextS) && (nDist > (MIN_VEH_GAP * 0.5))) ||
                        ((nVel > gnNextS) && (nDist > (MIN_VEH_GAP * 3.0))))
                    {
                        /* So this lane is fine to change, nothing to do */
                    }
                    else
                    {
                        bFeasible = false;
                        break;
                    } 
                }
                if (nCarIdxFr != -1)
                {
                    const double nDist = abs(vvvLanes[nTempLane][nCarIdxFr][8]);
                    const double nVel = vvvLanes[nTempLane][nCarIdxFr][7];
                    if (nDist > (MIN_VEH_GAP * 2.0))
                    {
                        /* So this lane is fine to change, nothing to do */
                    }
                    else
                    {
                        bFeasible = false;
                        break;
                    } 
                }
            }

            /* Check if all the lanes were fine */
            if (bFeasible == true)
            {
                gnLaneChangeVotes++;

                if (gnLaneChangeVotes > MIN_LC_VOTES)
                {
                    gbLaneChange = true;
                    nDestLane = nLane;
                    gnLaneChangeVotes = 0;
                }                
                break;
            }
        }

        /* Update the "d" */
        gnNextD = (nDestLane * SIM_LANE_WD) + (SIM_LANE_WD * 0.5); 

        /* Update the "s" */
        const int nCarIdx = vvCars[nDestLane][1];
        if (nCarIdx == -1)
        {
            /* Full speed ahead, since no car ahead of us */
            gnNextS = MAX_DIST_INC;
            // Tools::traceStream << "line 460, no car ahead of us,  MAX Speed: " << gnNextS << endl;                                                                           
        }
        else
        {
            const double nDist = vvvLanes[nDestLane][nCarIdx][8];
            const double nVel = vvvLanes[nDestLane][nCarIdx][7];
            if (nDist > (MIN_VEH_GAP * 4.0))
            {
                gnNextS = MAX_DIST_INC; 
                // Tools::traceStream << "line 468 MAX Speed: " << gnNextS << endl;                                                               
            }
            else if (nDist < (MIN_VEH_GAP * 1.0))
            {
                /* Emergency breaks */
                gnNextS = 0;
                // Tools::traceStream << "line 474 Emergency breaks: " << gnNextS << endl;
            }
            else
            {
                gnNextS = ((gnNextS * VELOCITY_COEF) < nVel) ? nVel : (gnNextS * VELOCITY_COEF);
                // Tools::traceStream << "line 479 Limit Speed (Max Speed * 0.8 ):  " << gnNextS << endl;                                                                                               
            }
        }
    }


    /*!
     * Finds the closest car behind and ahead of our car in 
     * each of the lanes
     */
    void Vehicle::FindClosestCars(const vvvd_t &vvvLanes, vvi_t &vvResult)
    {
        vvResult.resize(SIM_NUM_LANES);

        for (int i = 0; i < SIM_NUM_LANES; i++)
        {
            const int sz = vvvLanes[i].size();
            vvResult[i].pb(-1);
            vvResult[i].pb(-1);

            /* Find the closest car behind */
            for (int j = (sz - 1); j >= 0; j--)
            {
                /* Find the maximum negative value */
                if (vvvLanes[i][j][8] < 0)
                {
                    vvResult[i][0] = j;
                    break;
                }
            }

            /* Find the closest car ahead */
            for (int j = 0; j < sz; j++)
            {
                /* Find the minimum positive value */
                if (vvvLanes[i][j][8] > 0)
                {
                    vvResult[i][1] = j; 
                    break;
                }
            }
        }
    }