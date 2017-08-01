#include "header.h"

#ifndef PATHPLANNER_H
#define PATHPLANNER_H

class PathPlanner
{
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
    } CAR_STATE;

    typedef struct wp_map
    {
        vd_t x;
        vd_t y;
        vd_t s;
        vd_t dx;
        vd_t dy;
    } WP_MAP;

    /*!
    * @brief: Constructor to the PathPlanner class
    *
    * @param [in] oState: The state of the car
    * @param [in] prev_path: The previous path so far
    * @param [in] sensor_fusion: The sensor fusion data
    */
    PathPlanner(const WP_MAP Map);
    /*!
    * @brief: Destructor
    */
    ~PathPlanner();

    vvd_t Plan(CAR_STATE &State, vvd_t &PrevPath, vvd_t &SensorFusion);
    void TrackLane(spline &hLaneSpline);
    void TrackVelocityFirst(spline &hVelocitySpline);
    void HandleFirstCycle(spline &hLaneSpline, vvd_t &vvResult);
    void HandleGenericCycle(spline &hLaneSpline, vvd_t &vvResult);
    void BehaviourPlanner(void);
    void LaneChange(const vvvd_t &vvvLanes, const vvi_t &vvCars, const vi_t &vRanks);
    void RankLanes(const vvvd_t &vvvLanes, vvi_t &vvCars, vi_t &vResult);
    void FindClosestCars(const vvvd_t &vvvLanes, vvi_t &vvResult);
    int ClosestWaypoint(void);
    int NextWaypoint(void) ;
    vd_t getFrenet(void);
    vvd_t getLocalWPSeg(void);
    vd_t getLocalXY(const double dX, const double dY);
    vd_t getWorldXY(const double dX, const double dY);
    vvd_t getLocalPoints(const vd_t vdX, const vd_t vdY) ;
    vvd_t getWorldPoints(const vd_t vdX, const vd_t vdY) ;

private:
    /*! The waypoint map information */
    WP_MAP goMap;

    /*! The current state of the car */
    CAR_STATE goCar;

    /*! The current lane of the car */
    int gnCurLane;

    /*! The previous path */
    vvd_t gvvPrPath;

    /*! The size of the previous path */
    int gnPrPathSz;

    /*! Sensor Fusion */
    vvd_t gvvSenFus;

    /*! Size of the sensor fusion vector */
    int gnSenFusSz;

    /*! Stores the velocity of the path */
    vd_t gvvVelHist;

    /*! Stores the path history */
    vvd_t gvvPathHist;

    /*! Tracks if we are in a lane change */
    bool gbLaneChange = false;

    /*! Size of the waypoints */
    int gnMapSz;

    /*! The next d value */
    double gnNextD = 6.0;

    /*! The value of distance increment per time step */
    double gnNextS = MAX_DIST_INC;

    /*! The current time step */
    long long gnTimeStep = 0;

    /*! Votes for lane change */
    int gnLaneChangeVotes = 0;
    

};

#endif /* PATHPLANNER_H */