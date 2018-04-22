#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <vector>

#include "params.hpp"
#include "spline.h"
#include "utils.hpp"

using namespace std;

class Controller {

private:
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

public:
    Controller(vector<double> map_waypoints_x,
                vector<double> map_waypoints_y,
                vector<double> map_waypoints_s);
 
    std::tuple<vector<double>, vector<double>> 
    generate_trajectory(const params::CAR_STATE &car_state,
						const params::WORLD_STATE &world_state,
						vector<double>previous_path_x, 
						vector<double>previous_path_y);

};


Controller :: Controller(vector<double> map_waypoints_x,
                        vector<double> map_waypoints_y,
                        vector<double> map_waypoints_s) {

    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}
                        
                        
std::tuple<vector<double>, vector<double>> 
Controller :: generate_trajectory(const params::CAR_STATE &car_state,
								  const params::WORLD_STATE &world_state,
								  vector<double>previous_path_x, 
								  vector<double>previous_path_y) {

	int remaining_trajectory_len = previous_path_x.size();

	vector<double> anchor_xs;
	vector<double> anchor_ys;

	/* Calculate first 2 anchor points. To generate a smooth trajectory,
	* we use the last 2 points from the previous trajectory or, if there 
	* aren't enough points in the previous trajectory, use the car's
	* current coordinates as one of the points and a point tangential to 
	* car's current trajectory, slightly behind this point as the other
	* anchor point.
	*/
	double ref_x = 0;
	double ref_y = 0;
	double ref_x_prev = 0;
	double ref_y_prev = 0;
	double ref_yaw = 0;

	if(remaining_trajectory_len < 2){
		ref_x = car_state.x;
		ref_y = car_state.y;

		ref_yaw = deg2rad(car_state.yaw);

		ref_x_prev = ref_x - cos(ref_yaw);
		ref_y_prev = ref_y - sin(ref_yaw);
	} 
	else{
		ref_x = previous_path_x[remaining_trajectory_len-1];
		ref_y = previous_path_y[remaining_trajectory_len-1];

		ref_x_prev = previous_path_x[remaining_trajectory_len-2];
		ref_y_prev = previous_path_y[remaining_trajectory_len-2];

		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
	}

	anchor_xs.push_back(ref_x_prev);
	anchor_xs.push_back(ref_x);
	anchor_ys.push_back(ref_y_prev);
	anchor_ys.push_back(ref_y);

	/* Add more anchor points after the first two points.
		* These points are equi-spaced in Frenet s coordinates.
		*/
	int nb_additional_anchors = params::NB_ANCHOR_PTS-anchor_xs.size(); 
	for(int i=0; i<nb_additional_anchors; i++){
		vector<double> x_y = getXY(car_state.s + (i+1)*params::ANCHOR_S_INCR,
															car_state.d,
															map_waypoints_s,
															map_waypoints_x,
															map_waypoints_y);
		anchor_xs.push_back(x_y[0]);
		anchor_ys.push_back(x_y[1]);
	}


	/* Convert anchor point coordinates to an origin at (x_ref,y_ref) and rotate
		* by yaw_ref.
		*/
	for(int i=0; i<anchor_xs.size(); i++){
		double delta_x = anchor_xs[i] - ref_x;
		double delta_y = anchor_ys[i] - ref_y;

		anchor_xs[i] = delta_x * cos(ref_yaw) + delta_y * sin(ref_yaw);
		anchor_ys[i] = -delta_x * sin(ref_yaw) + delta_y * cos(ref_yaw);
	}


	/* Fit a spline to the anchor points. */
	tk::spline sp;
	sp.set_points(anchor_xs, anchor_ys);


	/* New trajectory. */
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	/* Copy over untraversed points from previous trajectory to new 
		* trajectory.
		*/
	for(int i=0; i<remaining_trajectory_len; i++){
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}

	/* Populate new trajectory points. */
	double y_horizon = sp(params::X_HORIZON);
	double horizon_distance = sqrt(params::X_HORIZON*params::X_HORIZON + y_horizon*y_horizon);

	double horizon_steps = horizon_distance / (params::SIMULATION_STEP * params::REF_VELOCITY);

	double x_start = 0; 	// In reference point's coordinate frame.
	for(int i=0; i<params::TRAJECTORY_SIZE-remaining_trajectory_len; i++){
		double x = x_start + (i+1)*params::X_HORIZON / horizon_steps;
		double y = sp(x);

	/* Convert back to map coordinate frame.
		* rotate by yaw_ref, then translate by (-x_ref, -y_ref)
		*/

		double x_map = x * cos(ref_yaw) - y * sin(ref_yaw) + ref_x; 
		double y_map = x * sin(ref_yaw) + y * cos(ref_yaw) + ref_y;

		next_x_vals.push_back(x_map);
		next_y_vals.push_back(y_map);

		cout  <<	x_map << " " << y_map << endl;

	}

	return std::tuple<vector<double>,vector<double>>(next_x_vals, next_y_vals);

}

#endif //__CONTROLLER_HPP__
