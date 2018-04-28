#ifndef __CONTROLLER_HPP__
#define __CONTROLLER_HPP__

#include <vector>

#include "params.hpp"
#include "spline.h"
#include "utils.hpp"

using namespace std;

class Controller {

private:
	double acceleration;
	double speed;
	double lane;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;

	
	inline void brake(){
		this->speed -= this->acceleration;
		if(this->speed < 0){
			this->speed = 0;
		}
	}

    std::tuple<double, double, double> 
	get_reference_and_anchors_points(const params::CAR_STATE &car_state,
									vector<double> &previous_path_x,
									vector<double> &previous_path_y, 
									vector<double> &anchor_xs,
									vector<double> &anchor_ys);
	
	params::TRAFFIC_MAP
	get_traffic_map(const params::CAR_STATE& ego_car_state, const params::WORLD_STATE& world_state);


	void set_target_params(const params::CAR_STATE &car_state,
						   const params::WORLD_STATE &world_state);


	inline void throttle(){
		this->speed += this->acceleration;
		if(this->speed > params::REF_VELOCITY){
			this->speed = params::REF_VELOCITY;
		}
	}



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

	this->acceleration = params::DEFAULT_THROTTLE;
	this->lane = params::DEFAULT_LANE;
	this->speed = params::REF_VELOCITY;
    this->map_waypoints_x = map_waypoints_x;
    this->map_waypoints_y = map_waypoints_y;
    this->map_waypoints_s = map_waypoints_s;
}



std::tuple<double,double, double> 
Controller::get_reference_and_anchors_points(const params::CAR_STATE &car_state,
								vector<double> &previous_path_x,
								vector<double> &previous_path_y, 
								vector<double> &anchor_xs,
								vector<double> &anchor_ys){

	
	double ref_x = 0;
	double ref_y = 0;
	double ref_x_prev = 0;
	double ref_y_prev = 0;
	double ref_yaw = 0;

	int remaining_trajectory_len = previous_path_x.size();

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
															center_of(this->lane),
															this->map_waypoints_s,
															this->map_waypoints_x,
															this->map_waypoints_y);
		anchor_xs.push_back(x_y[0]);
		anchor_ys.push_back(x_y[1]);
	}

	return std::tuple<double, double, double>(ref_x, ref_y, ref_yaw);
}
                        
                        

std::tuple<vector<double>, vector<double>> 
Controller :: generate_trajectory(const params::CAR_STATE &car_state,
								  const params::WORLD_STATE &world_state,
								  vector<double>previous_path_x, 
								  vector<double>previous_path_y) {

	int remaining_trajectory_len = previous_path_x.size();

	set_target_params(car_state, world_state);

	vector<double> anchor_xs;
	vector<double> anchor_ys;

	std::tuple<double,double,double> ref_x_y_yaw = 
		get_reference_and_anchors_points(car_state, 
										previous_path_x, 
										previous_path_y,
										anchor_xs,
										anchor_ys);

	double ref_x = std::get<0>(ref_x_y_yaw);
	double ref_y = std::get<1>(ref_x_y_yaw);
	double ref_yaw = std::get<2>(ref_x_y_yaw);
	
	
	/* Transform anchor point coordinates to  (x_ref,y_ref,@yaw_ref). */
	std::pair<vector<double>, vector<double>> transformed_anchors =
		transform_coordinates(std::pair<vector<double>, vector<double>>(anchor_xs,anchor_ys), 
							  std::pair<double, double>(ref_x,ref_y), 
							  ref_yaw);
	
	anchor_xs = std::get<0>(transformed_anchors);
	anchor_ys = std::get<1>(transformed_anchors);


	/* Fit a spline to the anchor points. */
	tk::spline sp;
	//print_coordinates(anchor_xs, anchor_ys);
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

	double horizon_steps = horizon_distance / (params::SIMULATION_STEP * this->speed);

	double x_start = 0; 	// In reference point's coordinate frame.
	vector<double> trajectory_xs;
	vector<double> trajectory_ys;

	for(int i=0; i<params::TRAJECTORY_SIZE-remaining_trajectory_len; i++){
		double x = x_start + (i+1)*params::X_HORIZON / horizon_steps;
		double y = sp(x);

		trajectory_xs.push_back(x);
		trajectory_ys.push_back(y);
	}

	/* Transform new trajectory points to map's coordinates. */
	std::pair<vector<double>, vector<double>> transformed_trajectory =
		inverse_transform_coordinates(std::pair<vector<double>,vector<double>>(trajectory_xs, trajectory_ys),
												  std::pair<double, double>(ref_x,ref_y),
												  ref_yaw);

	vector<double> transformed_trajectory_xs = std::get<0>(transformed_trajectory);
	vector<double> transformed_trajectory_ys = std::get<1>(transformed_trajectory);

	/* Append new trajectory points. */	
	//print_coordinates(transformed_trajectory_xs, transformed_trajectory_xs);
	next_x_vals.insert(next_x_vals.end(), transformed_trajectory_xs.begin(), transformed_trajectory_xs.end());
	next_y_vals.insert(next_y_vals.end(), transformed_trajectory_ys.begin(), transformed_trajectory_ys.end());

	return std::tuple<vector<double>,vector<double>>(next_x_vals, next_y_vals);

}



params::TRAFFIC_MAP
Controller::get_traffic_map(params::CAR_STATE& ego_car_state, params::WORLD_STATE& world_state){
	params::TRAFFIC_MAP traffic_map;

	/* Group traffic by lane, then by whether its ahead of us or behind us. */
	for(int i=0; i<world_state.cars_info.size(); i++){
		params::CAR_STATE other_car_state = world_state.cars_info[i];
		int other_lane =  to_lane(other_car_state.d);

		double separation = delta_s(ego_car_state.s, other_car_state.s);
	
		map<int, vector<params::CAR_STATE>> traffic_by_lane = (separation > 0 ? traffic_map.ahead : traffic_map.behind);
	
		auto search = traffic_by_lane.find(other_lane);
		if(search != traffic_by_lane.end()){
			vector<params::CAR_STATE> s = search->second;
			s.push_back(other_car_state);
		}
		else{
			vector<params::CAR_STATE> lane_traffic;
			lane_traffic.push_back(other_car_state);
			traffic_by_lane.insert(pair< int,vector<params::CAR_STATE> >(other_lane, lane_traffic));
		}
	}


	/* Sort traffic in every lane, both behind us and ahead of us, by absolute distance. */
	auto search = traffic_map.ahead.begin();
	while( search != traffic_map.ahead.end() ){
		vector<params::CAR_STATE> lane_traffic = search->second;
		sort_by_abs_distance(ego_car_state, lane_traffic);
	}

	auto search = traffic_map.behind.begin();
	while( search != traffic_map.behind.end() ){
		vector<params::CAR_STATE> lane_traffic = search->second;
		sort_by_abs_distance(ego_car_state, lane_traffic);
	}
}



params::CAR_STATE get_nearest_car(const params::TRAFFIC_MAP &traffic_map, const int lane_id, params::DIRECTION dir){
	map<int, vector<params::CAR_STATE>> traffic_by_lane = (dir == params::AHEAD ? traffic_map.ahead : traffic_map.behind);
	auto search = traffic_by_lane.find(lane_id);
	if(search != traffic_by_lane.end()) {
		vector<params::CAR_STATE> other_cars_states = search->second;
		return other_cars_states[0];
	}
}



void 
Controller::set_target_params(const params::CAR_STATE &ego_car_state,
							  const params::WORLD_STATE &world_state) {

	params::TRAFFIC_MAP traffic_map = get_traffic_map(ego_car_state, world_state);
	int my_lane = to_lane(ego_car_state.d) ;
	
	bool car_ahead = false;
	
	for(int i=0; i<world_state.cars_info.size(); i++){
		params::CAR_STATE other_car_state = world_state.cars_info[i];
		int other_lane =  to_lane(other_car_state.d);

		//cout << my_lane << other_lane << delta_s(car_state.s, other_car_state.s) << endl;
		
		if(other_lane == my_lane){
			double separation = delta_s(ego_car_state.s, other_car_state.s);
			if(separation > 0 && separation  < params::MIN_SAFE_DISTANCE ) {
				cout << "My d: " << ego_car_state.d << " my lane: " << my_lane << ", their d: " << other_car_state.d << " their lane: " << other_lane << endl;
				cout << "My s: " << ego_car_state.s << " their s: " << other_car_state.s <<endl;
				cout << "Separation: " << separation <<endl<<endl;
				car_ahead = true;
				brake();
			} 
		}
	}

	if(car_ahead == false){
		throttle();
	}
}

#endif //__CONTROLLER_HPP__
