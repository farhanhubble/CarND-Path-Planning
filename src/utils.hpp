#ifndef __UTILS_HPP__
#define __UTILS_HPP__

#include <vector>
#include <math.h>
#include <string>
#include "json.hpp"
#include "params.hpp"

using namespace std;

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

params::WORLD_STATE to_world_state(vector<vector<double>> sensor_info) {
	params::WORLD_STATE world_state;
	for(int i=0; i<sensor_info.size(); i++){
		vector<double> car_info = sensor_info[i];
		
		int id = (int)car_info[0];
		double x = car_info[1];
		double y = car_info[2];
		double v_x = car_info[3];
		double v_y = car_info[4];
		double s = car_info[5];
		double d = car_info[6];

		double yaw = rad2deg(atan2(v_y,v_x));
		double speed = v_x * v_x + v_y * v_y;
		params::CAR_STATE car_state = {id, x, y, yaw, s, d, speed};

		world_state.cars_info.push_back(car_state);
	}

	return world_state;
}


std::pair<vector<double>, vector<double>> 
translate_coordinates(std::pair<vector<double>, vector<double>> points, 
					 std::pair<double,double> pivot) {
	vector<double> xs = std::get<0>(points);
	vector<double> ys = std::get<1>(points);

	double pivot_x = std::get<0>(pivot);
	double pivot_y = std::get<1>(pivot);

	int nb_points = xs.size();
	
	vector<double> out_xs(nb_points);
	vector<double> out_ys(nb_points);
	
	for(int i=0; i<nb_points; i++){
		out_xs[i] = xs[i] - pivot_x;
		out_ys[i] = ys[i] - pivot_y;
	}

	return std::pair<vector<double>, vector<double>> (out_xs, out_ys); 
}



std::pair<vector<double>, vector<double>> 
rotate_coordinates(std::pair<vector<double>, vector<double>> points, double theta) {
	vector<double> xs = std::get<0>(points);
	vector<double> ys = std::get<1>(points);

	int nb_points = xs.size();
	
	vector<double> out_xs(nb_points);
	vector<double> out_ys(nb_points);
	
	for(int i=0; i<nb_points; i++){
		out_xs[i] =  xs[i] * cos(theta) + ys[i] * sin(theta);
		out_ys[i] = -xs[i] * sin(theta) + ys[i] * cos(theta);
	}

	return std::pair<vector<double>, vector<double>> (out_xs, out_ys); 
}



std::pair<vector<double>, vector<double>> 
transform_coordinates(std::pair<vector<double>, vector<double>> points,
					  std::pair<double,double> pivot,
					  double theta) {

	auto result = translate_coordinates(points, pivot);
	return rotate_coordinates(result, theta);
}



std::pair<vector<double>, vector<double>> 
inverse_transform_coordinates(std::pair<vector<double>, vector<double>> points,
					  		  std::pair<double,double> pivot,
					  		  double theta) {

	auto result = rotate_coordinates(points, -theta);
	auto negated_pivot = std::pair<double, double>(-1*std::get<0>(pivot), -1*std::get<1>(pivot));
	return translate_coordinates(result, negated_pivot);
}



void print_coordinates(vector<double> xs, vector<double> ys, char separator='\n'){
	assert(xs.size() == ys.size());
	int nb_pts = xs.size();

	for(int i=0; i<nb_pts; i++){
		cout << xs[i] << " " << ys[i] << separator;
	}
}
#endif //__UTILS_HPP__