#ifndef __PARAMS_HPP__
#define __PARAMS_HPP__
namespace params {
const double SIMULATION_STEP = 0.02; // Simulator timestep in seconds.
const int NB_ANCHOR_PTS = 5;         // Total number of anchor points for trajectory generation.   
const double ANCHOR_S_INCR = 30.0;   // Frenet s distance between consecutive anchor points.
const double X_HORIZON = 30.0;	     // Trajectory planning horizon.

const double REF_VELOCITY = 49.0 * 0.44704; // 49 MPH converted to m/s.
const int TRAJECTORY_SIZE = 50;      // Number of points in the trajectory.
}
#endif //__PARAMS_HPP__