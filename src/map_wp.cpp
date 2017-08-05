#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "map_wp.h"

using namespace std;

//
// read the highway_map.csv file and calc the spline from it.
// we wrap this file in order to maintain continuity on "s" at the end of the track. 
map_wp::map_wp(){

	string map_file_ = "/home/highway_map.csv";
	//string map_file_ = "../data/highway_map.csv";

	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;
	
	int i=0, N_pad=10;
	double x, y, s, d_x, d_y;
	vector<double> x_pad(N_pad), y_pad(N_pad), s_pad(N_pad), dx_pad(N_pad), dy_pad(N_pad);
	while (getline(in_map_, line)) {
		istringstream iss(line);
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
		
		if (i<N_pad){
			x_pad[i]=x;
			y_pad[i]=y;
			s_pad[i]=(s+MAX_S);
			dx_pad[i]=d_x;
			dy_pad[i]=d_y;
		}
		i++;
	}
	cout << "Original highway_map length = " << i;
	cout << ". After padding = " << (N_pad+i) << endl;

	for (int i=0; i<N_pad; i++){
		map_waypoints_x.push_back(x_pad[i]);
		map_waypoints_y.push_back(y_pad[i]);
		map_waypoints_s.push_back(s_pad[i]);
		map_waypoints_dx.push_back(dx_pad[i]);
		map_waypoints_dy.push_back(dy_pad[i]);
	}
	
	calc_splines();

	//string dbg_file_ = "/home/dbg_next_path_xy.csv";
	//dbg_path_xy.open(dbg_file_.c_str());
}

map_wp::~map_wp(){
	//dbg_path_xy.close();
}

//
// calc the splines
void map_wp::calc_splines(){
	spline_x.set_points(map_waypoints_s, map_waypoints_x);
	spline_y.set_points(map_waypoints_s, map_waypoints_y);
	spline_dx.set_points(map_waypoints_s, map_waypoints_dx);
	spline_dy.set_points(map_waypoints_s, map_waypoints_dy);
}

// 
// get {x,y} location as a function of {s,d}
vector<double> map_wp::getXY_spline(double s, double d){
	s = fmod(s,MAX_S);

	double x = spline_x(s) + d * spline_dx(s);
	double y = spline_y(s) + d * spline_dy(s);
	
	return {x,y};
}

//
// debug prints
void map_wp::print_path(vector<double>next_x_vals, vector<double>next_y_vals){
	for (int i=0; i<next_x_vals.size(); i++){
		cout << next_x_vals[i] << " , " << next_y_vals[i] << endl;
		//dbg_path_xy << next_x_vals[i] << "," << next_y_vals[i] << endl;
	}
}
