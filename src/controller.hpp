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
	params::ACTION_STATES action;
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;


	vector<tuple<int,double,double>>
	get_prospective_lanes(const params::TRAFFIC_MAP& traffic_map, 
					   const params::CAR_STATE &ego_car_state, 
					   vector<int> target_lanes);


	params::CAR_STATE* 
	get_nearest_car(const params::TRAFFIC_MAP &traffic_map, const int lane_id, params::DIRECTION dir);

	vector<tuple<int,double,double>>  
	get_new_lane(const params::TRAFFIC_MAP& traffic_map, const params::CAR_STATE& ego_car_state);


    std::tuple<double, double, double> 
	get_reference_and_anchors_points(const params::CAR_STATE &car_state,
									vector<double> &previous_path_x,
									vector<double> &previous_path_y, 
									vector<double> &anchor_xs,
									vector<double> &anchor_ys);
	

	params::TRAFFIC_MAP
	get_traffic_map(const params::CAR_STATE& ego_car_state, const params::WORLD_STATE& world_state);


	void
	print_traffic_map(const params::TRAFFIC_MAP& traffic_map);


	void set_target_params(const params::CAR_STATE &car_state,
						   const params::WORLD_STATE &world_state);


	inline void throttle(double val){
		if(abs(val) > params::MAX_THROTTLE) { 
			cout << "ERROR: Throttle value " << val << " exceeds maximum safe throttle.";
			val = val < -params::MAX_THROTTLE ? -params::MAX_THROTTLE : val;
			val = val >  params::MAX_THROTTLE ?  params::MAX_THROTTLE : val; 
			cout << " Adjusting throttle to " << val << endl;
		}
		this->acceleration  = val;
		
		cout << "Changing speed from " << this->speed;
		this->speed += this->acceleration;

		if(this->speed < 0){
			this->speed = 0;
		}
		else if(this->speed > params::REF_VELOCITY){
			this->speed = params::REF_VELOCITY;
		}
		cout << " to " << this->speed << endl;

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

	this->acceleration = params::MAX_THROTTLE;
	this->action = params::KEEP_LANE;
	this->lane = params::DEFAULT_LANE;
	this->speed = params::COLD_START_VELOCITY;
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
	

	double s_incr = params::ANCHOR_S_INCR;
	if(this->action > params::KEEP_LANE){
		s_incr *= 1.5;
	}
	for(int i=0; i<nb_additional_anchors; i++){
		vector<double> x_y = getXY(car_state.s + (i+1)*s_incr,
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
Controller::get_traffic_map(const params::CAR_STATE& ego_car_state, const params::WORLD_STATE& world_state){
	cout << "Total vehicles being tracked:" << world_state.cars_info.size() << endl;
	int a=0, b =0;
	params::TRAFFIC_MAP traffic_map;

	/* Group traffic by lane, then by whether its ahead of us or behind us. */
	for(int i=0; i<world_state.cars_info.size(); i++){
		params::CAR_STATE other_car_state = world_state.cars_info[i];
		int other_lane =  to_lane(other_car_state.d);

		double separation = delta_s(ego_car_state.s, other_car_state.s);
	
		map<int, vector<params::CAR_STATE>> *p_traffic_by_lane = (separation > 0 ? &traffic_map.ahead : &traffic_map.behind);
		if(p_traffic_by_lane == &traffic_map.ahead) {a++;}
		else if(p_traffic_by_lane == &traffic_map.behind) {b++;}	
		auto search = p_traffic_by_lane->find(other_lane);
		if(search != p_traffic_by_lane->end()){
			vector<params::CAR_STATE>* ptr_s = &search->second;
			ptr_s->push_back(other_car_state);
		}
		else{
			vector<params::CAR_STATE> lane_traffic;
			lane_traffic.push_back(other_car_state);
			p_traffic_by_lane->insert(pair< int,vector<params::CAR_STATE> >(other_lane, lane_traffic));
		}
	}

	/* Sort traffic in every lane, both behind us and ahead of us, by absolute distance from our vehicle */
	auto __sort__ = [&ego_car_state](map<int, vector<params::CAR_STATE>> &traffic_map) -> void {
		for(auto search = traffic_map.begin(); search != traffic_map.end(); ++search){
		vector<params::CAR_STATE>* ptr_lane_traffic = &search->second;
		sort_by_abs_distance(ego_car_state, ptr_lane_traffic); 
		}
	};
	__sort__(traffic_map.ahead);
	__sort__(traffic_map.behind);

	return traffic_map;
}



params::CAR_STATE* 
Controller::get_nearest_car(const params::TRAFFIC_MAP &traffic_map, const int lane_id, params::DIRECTION dir){
	map<int, vector<params::CAR_STATE>> traffic_by_lane = (dir == params::AHEAD ? traffic_map.ahead : traffic_map.behind);
	auto search = traffic_by_lane.find(lane_id);
	if(search != traffic_by_lane.end()) {
		vector<params::CAR_STATE> other_cars_states = search->second;
		return &other_cars_states[0];
	}

	return (params::CAR_STATE*)NULL;
}



void 
Controller::print_traffic_map(const params::TRAFFIC_MAP& traffic_map){
	map<int, vector<params::CAR_STATE>> traffic_ahead = traffic_map.ahead;
	map<int, vector<params::CAR_STATE>> traffic_behind = traffic_map.behind;

	auto __print__ = [](const char *header, const map<int, vector<params::CAR_STATE>> &traffic) -> void {
		cout << "###" << header << "###" <<endl;
		for(auto it_ahead = traffic.begin(); it_ahead != traffic.end(); ++it_ahead){
			int lane_id = it_ahead->first;
			vector<params::CAR_STATE> lane_traffic = it_ahead->second;

			cout << lane_id << ": ";
			for(int i=0; i<lane_traffic.size(); i++){
				cout << "[ " << "id:" << lane_traffic[i].id << " s:" << lane_traffic[i].s << " d:" <<lane_traffic[i].d << " ]"; 
			}
			cout << endl;
		}
	};

	__print__("AHEAD",traffic_ahead);
	__print__("BEHIND",traffic_behind);
	
}


vector<tuple<int,double,double>> 
Controller :: get_prospective_lanes(const params::TRAFFIC_MAP& traffic_map, const params::CAR_STATE &ego_car_state, vector<int> target_lanes){
	vector<tuple<int, double, double>> feasible_lanes;
	char switch_map[3] = {'X','X','X'};

	for(auto it=target_lanes.begin(); it!= target_lanes.end(); ++it){
		int target_lane = *it;
		const params::CAR_STATE *p_car_ahead  = get_nearest_car(traffic_map, target_lane, params::AHEAD);
		const params::CAR_STATE *p_car_behind = get_nearest_car(traffic_map, target_lane, params::BEHIND);

		double delta_ahead  = p_car_ahead  == (params::CAR_STATE*)NULL ? 65535 : abs(delta_s(ego_car_state.s, p_car_ahead->s));
		double delta_behind = p_car_behind == (params::CAR_STATE*)NULL ? 65535 : abs(delta_s(ego_car_state.s, p_car_behind->s));
		
		if( delta_ahead  > params::SAFE_FOLLOW_DISTANCE && 
			delta_behind > 0.5*params::MIN_SAFE_DISTANCE) {	
			feasible_lanes.push_back(tuple<int,double,double>(*it, delta_ahead, delta_behind));
			switch_map[target_lane] = 'I';

		}
	}
	switch_map[to_lane(ego_car_state.d)] = 'O';
	cout << switch_map[0] << " " << switch_map[1] << " " << switch_map[2] << endl <<endl;
	return feasible_lanes;
}



vector<tuple<int,double,double>>  
Controller::get_new_lane(const params::TRAFFIC_MAP& traffic_map, const params::CAR_STATE& ego_car_state){
	int my_lane = to_lane(ego_car_state.d);

	vector<int> targets;
	if(my_lane == 0){
		targets.push_back(1);
		targets.push_back(2);
	}
	else if(my_lane == 1){
		targets.push_back(0);
		targets.push_back(2);
	}

	else if(my_lane == 2){
		targets.push_back(0);
		targets.push_back(1);
	}

	return get_prospective_lanes(traffic_map, ego_car_state, targets);

}



void 
Controller::set_target_params(const params::CAR_STATE &ego_car_state,
							  const params::WORLD_STATE &world_state) {
	int my_lane = to_lane(ego_car_state.d) ;
	if(this->action == params::CHANGE_LANE_LEFT || this->action == params::CHANGE_LANE_RIGHT){
		/* If lane change in progress, parameters remain unchanged. */
		if(my_lane != this->lane){
			cout << "Lane change ongoing, blocking parameter update." << endl;
			return;
		}
	}
	
	params::TRAFFIC_MAP traffic_map = get_traffic_map(ego_car_state, world_state);
	
	
	//cout << "[ " << "id:" << ego_car_state.id << " s:" << ego_car_state.s << " d:" <<ego_car_state.d << " ]" << endl; 
	//print_traffic_map(traffic_map);

	params::ACTION_STATES action = params::KEEP_LANE;
	double desired_throttle = 0;
	
	const params::CAR_STATE *p_car_ahead = get_nearest_car(traffic_map, my_lane, params::AHEAD);
	if(p_car_ahead == (params::CAR_STATE*)NULL) {
		desired_throttle = params::MAX_THROTTLE;
		action = params::KEEP_LANE;
		cout << "NO CAR AHEAD" << endl;
	}
	else{
		double separation = delta_s(ego_car_state.s, p_car_ahead->s);
		cout << "CAR AHEAD AT " << separation << "m." <<endl;
		//cout << "Self:" << "[ " << "s: " << ego_car_state.s << " d: " << ego_car_state.d << " speed: " << MPH2mps(ego_car_state.speed)   << "(REPORTED) " << this->speed <<"(CONTROLLER)" << " ]" << endl;
		//cout << "Othr:" << "[ " << "s: " << p_car_ahead->s  << " d: " << p_car_ahead->d  << " speed: " << p_car_ahead->speed   << "(REPORTED) "  << " ]"  << endl;
		
		if(separation > params::SAFE_FOLLOW_DISTANCE){
			desired_throttle = params::MAX_THROTTLE;
			action = params::KEEP_LANE;
			cout << "Car ahead beyond follow zone. Throttling at " << desired_throttle << endl;
		}

		else if(separation <= params::SAFE_FOLLOW_DISTANCE && separation > params::MIN_SAFE_DISTANCE) {
			auto target_lanes = get_new_lane(traffic_map, ego_car_state);
			
			/* Change lane if possible, not already doing so. */
			if(target_lanes.size() != 0 && this->action == params::KEEP_LANE){
				desired_throttle = params::MAX_THROTTLE;
				for(auto it=target_lanes.begin(); it != target_lanes.end(); ++it){
					int tgt_lane = get<0>(*it);
					if(tgt_lane ==  my_lane-1 || tgt_lane == my_lane+1){
						action = tgt_lane == my_lane-1 ? params::CHANGE_LANE_LEFT : params::CHANGE_LANE_RIGHT;
						break;		
					}
				}
			}
			else{
				double delta_speed = p_car_ahead->speed - this->speed;
				double t = (separation - params::MIN_SAFE_DISTANCE) / delta_speed;
				desired_throttle = (delta_speed / t)*params::SIMULATION_STEP;
				cout << "Car ahead in follow zone. Throttling at " << desired_throttle << endl;
			}
		}

		else if(separation  < params::MIN_SAFE_DISTANCE ) {			
			if(this->speed >  0.95 * p_car_ahead->speed){
				desired_throttle = -params::MAX_THROTTLE;
				action = params::KEEP_LANE;
				cout << "Car ahead in collision zone moving slower. Throttling at " << desired_throttle << endl;
			}		
			cout << "Car ahead in collision zone moving faster. Not throttling" << endl;
		}	
	}

	cout << "action= " << action << endl;
	if(action != params::KEEP_LANE){
		cout << "Change lanes from " << this->lane;
		this->lane = this->lane + (action == params::CHANGE_LANE_LEFT? -1 : +1 );
		cout << " to " << this->lane  << endl;
	}
	throttle(desired_throttle);
	this->action = action;
	cout << endl;
}

#endif //__CONTROLLER_HPP__
