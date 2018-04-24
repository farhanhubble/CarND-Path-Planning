#ifndef __PARAMS_HPP__
#define __PARAMS_HPP__

#include <vector>

using namespace std;

namespace params {

const double SIMULATION_STEP = 0.02; // Simulator timestep in seconds.
const int NB_ANCHOR_PTS = 5;         // Total number of anchor points for trajectory generation.   
const double ANCHOR_S_INCR = 30.0;   // Frenet s distance between consecutive anchor points.
const double X_HORIZON = 30.0;	     // Trajectory planning horizon.

const double REF_VELOCITY = 49.0 * 0.44704; // 49 MPH converted to m/s.
const int TRAJECTORY_SIZE = 50;      // Number of points in the trajectory.

const int EGO_CAR_ID = -1;          // Own car ID.

const double DEFAULT_THROTTLE = 5.0 * SIMULATION_STEP; // Default acceleration/deceleration.
const int DEFAULT_LANE = 1;
const double MIN_SAFE_DISTANCE = 30.0;

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

typedef struct {
    vector<CAR_STATE> cars_info;
} WORLD_STATE;


}
#endif //__PARAMS_HPP__