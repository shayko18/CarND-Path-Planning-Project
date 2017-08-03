#include <fstream>
#include <sstream>
#include <cmath>
#include "vehicle_info.h"

using namespace std;

vehicle_info::vehicle_info(){
	s=0.0;
	s_dot=0.0;
	s_dot_dot=0.0;
	d=0.0;
	d_dot=0.0;
	d_dot_dot=0.0;
}

void vehicle_info::set(double s_, double s_dot_, double s_dot_dot_, double d_, double d_dot_, double d_dot_dot_){
	s=s_;
	s_dot=s_dot_;
	s_dot_dot=s_dot_dot_;
	d=d_;
	d_dot=d_dot_;
	d_dot_dot=d_dot_dot_;
}

void vehicle_info::set_vec(vector<double> vec){
	set(vec[0], vec[1], vec[2], vec[3], vec[4], vec[5]);
}

vector<double> vehicle_info::get_vec(){
	vector<double> vec;
	vec = {s, s_dot, s_dot_dot, d, d_dot, d_dot_dot};
	return vec; 
}