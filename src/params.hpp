#ifndef __PARAMS_HPP__
#define __PARAMS_HPP__

#include <vector>


using namespace std;

#define MPH2mps(x) (x*0.44704)
namespace params {

const double SIMULATION_STEP = 0.02; // Simulator timestep in seconds.
const int NB_ANCHOR_PTS = 5;         // Total number of anchor points for trajectory generation.   
const double ANCHOR_S_INCR = 30.0;   // Frenet s distance between consecutive anchor points.
const double X_HORIZON = 30.0;	     // Trajectory planning horizon.

const double REF_VELOCITY = MPH2mps(49.0); // 49 MPH converted to m/s.
const double COLD_START_VELOCITY = 9.0*.02;     // 9.0 m/s
const int TRAJECTORY_SIZE = 50;             // Number of points in the trajectory.

const int EGO_CAR_ID = -1;          // Own car ID.

const double MAX_THROTTLE = 25.0 * SIMULATION_STEP; // Default acceleration/deceleration.
const int DEFAULT_LANE = 1;
const int LEFT_MOST_LANE = 0 ;
const int RIGHT_MOST_LANE = 2;
const double MIN_SAFE_DISTANCE = 30.0;
const double SAFE_FOLLOW_DISTANCE = 1.5 * MIN_SAFE_DISTANCE;

const double MIN_FRENET_D = 0.0;
const double MAX_FRENET_D = 12.0;

enum ACTION_STATES{
    HALT = 0,
    KEEP_LANE ,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT,
};


typedef struct {
    int id;
    double x;
    double y;
    double yaw;
    double s;
    double d;
    double speed;

} CAR_STATE;



typedef struct{
    map<int, vector<CAR_STATE>> ahead;
    map<int, vector<CAR_STATE>> behind;
} TRAFFIC_MAP;



typedef struct {
    vector<CAR_STATE> cars_info;
} WORLD_STATE;



typedef enum {
    AHEAD,
    BEHIND
} DIRECTION;


}
#endif //__PARAMS_HPP__