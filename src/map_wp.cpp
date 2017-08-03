#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "map_wp.h"

using namespace std;

map_wp::map_wp(){

	string map_file_ = "/home/highway_map.csv";
	//string map_file_ = "../data/highway_map.csv";

	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;

	int i=0;
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
		i++;
	}
	cout << "highway_map length = " << i << endl;

	max_s = 6945.554;
	calc_splines();

	string dbg_file_ = "/home/dbg_next_path_xy.csv";
	//string dbg_file_ = "/host/dbg_next_path_xy.csv";
	dbg_path_xy.open(dbg_file_.c_str());
}

map_wp::~map_wp(){
	dbg_path_xy.close();
}

void map_wp::calc_splines(){
	spline_x.set_points(map_waypoints_s, map_waypoints_x);
	spline_y.set_points(map_waypoints_s, map_waypoints_y);
	spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
	spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

vector<double> map_wp::getXY_spline(double s, double d){
 	s = fmod(s,max_s);
	
	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);
	return {x,y};
}

void map_wp::print_path(vector<double>next_x_vals, vector<double>next_y_vals){
	for (int i=0; i<next_x_vals.size(); i++){
		//cout << next_x_vals[i] << " , " << next_y_vals[i] << endl;
		dbg_path_xy << next_x_vals[i] << "," << next_y_vals[i] << endl;
	}
}
