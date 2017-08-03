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
#include "spline.h"

/* #################### NAMESPACES #################### */
using namespace std;
using json = nlohmann::json;
using spline = tk::spline;

/* #################### DEFINES #################### */
/** INFRASTRUCTURE **/
#define WP_FILE             ("../data/highway_map.csv") /*!< The path to the waypoint file */
#define SIM_PORT            (4567)      /*!< The port, the simulator uses to connect */
#define SF_NUM_ELEMENTS     (7)         /*!< Number of elements in each car's sensor fusion */
#define SIM_TIME_SLICE      (0.02)      /*!< The time slice in the simulator */
#define SIM_NUM_LANES       (3)         /*!< Number of lanes in the simulator */
#define SIM_LANE_WD         (4)         /*!< The lane width in the simulator */
#define FLOAT_INF           (numeric_limits<double>::infinity())

/** ALGORITHM **/
#define NUM_POINTS          (40)        /*!< Number of points predicted in each cycle */
#define MAX_DIST_INC        (0.4425)    /*!< The maximum dist inc per time step, corresponds to max velocity */
#define WP_SPLINE_PREV      (6)         /*!< Number of waypoints to look behind when constructing a spline */
#define WP_SPLINE_TOT       (25)        /*!< Total number of waypoints to look when constructing a spline */
#define LANE_BUFFER         (0.3)       /*!< The buffer between lanes, where cars are considered to be changing lanes */
#define FLOAT_EPS           (0.1)       /*!< A small epsilon used in the algorithm */
#define BEH_LANE_SCR        (0.75)      /*!< The lane scoring weight for the behaviour planner */
#define BEH_DIST_SCR        (3.0)       /*!< The velocity scoring weight for the behaviour planner */
#define BEH_VEL_SCR         (3.0)       /*!< The distance scoring weight for the behaviour planner */
#define MIN_VEH_GAP         (10.0)      /*!< The minimum vehicle gap to be maintained at all costs */
#define MAX_VEH_GAP         (200.0)     /*!< The maximum observable vehicle gap */
#define MIN_LC_VOTES        (20)        /*!< The minimum number of votes needed to make a decision of lane change */

//added by binliu 170802
#define SPEED_LIMIT        (49.9) 
//end add

/* #################### SIMPLIFICATIONS #################### */
typedef vector<int> vi_t;
typedef vector<double> vd_t;
typedef vector<vector<int>> vvi_t;
typedef vector<vector<double>> vvd_t;
typedef vector<vector<vector<double>>> vvvd_t;
#define pb push_back
#define REQUIRE(x) { \
    if (!(x)) \
    { \
        cout << "ERROR AT " << __LINE__ << " IN FILE " << __FILE__ << endl; \
        exit(1); \
    } \
}


/* #################### STATIC FUNCTIONS #################### */


/*!
 * @brief: Converts degrees to radians
 *
 * @param [in] x: The value in degrees to be converted to radians
 *
 * @return: The corresponding value in radians
 */
static double deg2rad(const double x)
{ 
    return ((x * M_PI) / 180.0); 
}

/*! @brief: Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 *
 * @param [in] s: The string to be tested
 *
 * @return: The payload extracted as a string
 */
static string hasData(const string s) 
{
    const auto found_null = s.find("null");
    const auto b1 = s.find_first_of("[");
    const auto b2 = s.find_first_of("}");
    if (found_null != string::npos) 
    {
        return "";
    } 
    else if (b1 != string::npos && b2 != string::npos) 
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

// For converting back and forth between radians and degrees.
static double rad2deg(double x) { return x * 180.0 / M_PI; }

/*!
    * @brief: Computes the distance between 2 points on catesian co-ordinate system
    *
    * @param [in] x1, x2, y1, y2: The co-ordinates of the two points (x1, y1), (x2, y2)
    *
    * @return: The euclidean distance between them
    */
static double distance(const double x1, const double y1, const double x2, const double y2)
{
    const double dXDiff = x2 - x1;
    const double dYDiff = y2 - y1;
    return sqrt((dXDiff * dXDiff)  + (dYDiff * dYDiff));
}


 /**
  *  Setup debuging
  */ 

using namespace std;

class Tools {
public:
   /**
  * Debug stuff
  */ 
  static string tracelog;
  static bool trace_tag;
  static ofstream  traceStream;  
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();
};