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
#include "params.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

						int remaining_trajectory_len = previous_path_x.size();
						//cout << "remaining_trajectory_len " << remaining_trajectory_len << endl;

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
							ref_x = car_x;
							ref_y = car_y;

							ref_yaw = deg2rad(car_yaw);
	
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
							vector<double> x_y = getXY(car_s + (i+1)*params::ANCHOR_S_INCR,
																				6.0,
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

							cout << remaining_trajectory_len << " " <<	x_map << " " << y_map << endl;

						}

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";
          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
