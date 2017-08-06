#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "poly_path.h"
#include "object_info.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

poly_path::poly_path(){
	cnt_change_lane=0;
	change_lane_trg=eNan;
}

poly_path::~poly_path(){

}

//
// calc the "next" path
void poly_path::calc_path_sd(std::vector<double> vehicle_info, std::vector<object_info> objects_info, int N){

	path_s.clear();
	path_d.clear();
		
	// init states 
	vector<double> s_start = {vehicle_info[0], vehicle_info[1], vehicle_info[2]};
	vector<double> d_start = {vehicle_info[3], vehicle_info[4], vehicle_info[5]};
	
	eLane my_curr_lane = get_lane(vehicle_info[3]);
	eLane my_next_lane = my_curr_lane;
	
	// distance and velocities ahead {right,center,left}
	vector<vector<double>> dis_vel_ahead = get_dis_val_ahead(s_start[0], objects_info);
	vector<double> dis_ahead = dis_vel_ahead[0];
	vector<double> vel_ahead = dis_vel_ahead[1];
	bool lane_change=false;
	
	//
	// algorithm to select the next lane
	if (cnt_change_lane){                        // we are in the middle of changing lanes. give it 2 seconds (2 updates)
		my_next_lane=change_lane_trg;
		cnt_change_lane = (get_is_in_lane(change_lane_trg, d_start[0]))?0:(cnt_change_lane+1);
	}
	if (cnt_change_lane==0){                     // we check if we need to change lanes
		if (dis_ahead[my_curr_lane] < MIN_SAFE_DIS_AHEAD){
			if (my_curr_lane!=eCenter){
				if (dis_ahead[eCenter] > MIN_SAFE_DIS_AHEAD){
					my_next_lane = eCenter;
				}
			}
			else{
				if (MAX(dis_ahead[eRight], dis_ahead[eLeft])>MIN_SAFE_DIS_AHEAD){
					my_next_lane = (dis_ahead[eRight]>dis_ahead[eLeft])?eRight:eLeft;
				}
			}
		}
		else if (my_curr_lane!=eCenter && (dis_ahead[eCenter] > 2*MIN_SAFE_DIS_AHEAD)){ // we prefer to be on the center lane
			my_next_lane = eCenter;
		}	
		lane_change = (my_next_lane!=my_curr_lane);
		if (lane_change){
			cnt_change_lane=1;
			change_lane_trg=my_next_lane;		
		}
	}
	
	//
	// finding the target speed at the end of the path.
	bool follow_car = false;
	double s_dot_end = MAX_SPEED_MPS;
	if ((dis_ahead[my_next_lane] < MIN_SAFE_DIS_AHEAD) && (vel_ahead[my_next_lane] < MAX_SPEED_MPS)){
		follow_car=true;
		s_dot_end = vel_ahead[my_next_lane];
	}
	double ds_dot = s_dot_end - s_start[1];
	ds_dot = MIN(ds_dot,(MAX_SPEED_MPS*0.25));
	ds_dot = MAX(ds_dot,-(MAX_SPEED_MPS*0.25));
	
	s_dot_end = s_start[1]+ds_dot;
	
	if (s_dot_end >= (MAX_SPEED_MPS-0.001)){
		s_dot_end-=0.5*ds_dot;
	}
	
	double ds = s_dot_end*DT;
	double d_next = get_next_lane_d(my_next_lane, d_start[0]);
	 
	// log prints (debug)
	bool log_enable = true;
	if (log_enable){
		if (!lane_change) {cout << "STAY_" << cnt_change_lane;}
		else {cout << "TURN_" << cnt_change_lane;}
		if (follow_car) {cout << " RE";}
		else {cout << " MY";}
		
		cout << ", Ss=" << print_fmt(s_start[0]) << ", Ds=" << print_fmt(d_start[0]);
		cout << ", De=" << print_fmt(d_next) << ", Ve=" << print_fmt(MS_2_MPH(s_dot_end)) << " ; ";
		cout << "{" << print_fmt(dis_ahead[0]) << ", " << print_fmt(MS_2_MPH(vel_ahead[0])) << "}  ";
		cout << "{" << print_fmt(dis_ahead[1]) << ", " << print_fmt(MS_2_MPH(vel_ahead[1])) << "}  ";
		cout << "{" << print_fmt(dis_ahead[2]) << ", " << print_fmt(MS_2_MPH(vel_ahead[2])) << "}  ";
		cout << endl;
	}
	
	// target state
	vector<double> s_end = {s_start[0]+ds*N, s_dot_end, 0.0};
	vector<double> d_end = {d_next, 0.0, 0.0};
		
	// calc the 6th order poly coeffs (s, d)
	vector<double> s_poly_coeff = jmt(s_start, s_end, N*DT);
	vector<double> d_poly_coeff = jmt(d_start, d_end, N*DT);

	// calc the new path (s, d)
	double t=0.0;
	for (int i=0; i<N; i++){
		path_s.push_back(poly_val(s_poly_coeff, t));
		path_d.push_back(poly_val(d_poly_coeff, t));
		t+=DT;
	}
}

//
// jmt
vector<double> poly_path::jmt(vector<double> start, vector <double> end, double T){
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT 
    an array of length 6, each value corresponding to a coefficent in the polynomial 
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5
    */
    
	
	
    MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;
			
	MatrixXd B = MatrixXd(3,1);	    
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];
		
	
	MatrixXd Ai = A.inverse();
	
	MatrixXd C = Ai*B;
	
	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++){
	    result.push_back(C.data()[i]);
	}
	
    return result;
}

//
// eval the poly val at the given point
double poly_path::poly_val(vector<double> coeff, double t){
	double result = 0.0;
    for (int i = 0; i < coeff.size(); i++) {
       result += coeff[i] * pow(t, i);
    }
    return result;
}

//
// setting the car info according to the path
vector<double> poly_path::est_vehicle_info(int n){
	int n0 = MAX(n,1);
	n0 = MIN (n0, path_s.size()-2);
	
	double s0 = path_s[n0];
	double s_m1 = path_s[n0-1];
	double s_p1 = path_s[n0+1];
	double d0 = path_d[n0];
	double d_m1 = path_d[n0-1];
	double d_p1 = path_d[n0+1];
	
	double s_dot = (s_p1-s_m1)/(2*DT);
	double d_dot = (d_p1-d_m1)/(2*DT);
	
	double s_dot_dot = (s_p1+s_m1-2*s0)/(DT*DT);
	double d_dot_dot = (d_p1+d_m1-2*d0)/(DT*DT);
	
	return {s0, s_dot, s_dot_dot, d0, d_dot, d_dot_dot};
}

//
// getting the distance and velocity of the other cars ahead of us
vector<vector<double>> poly_path::get_dis_val_ahead(double s, vector<object_info> objects_info){
	vector<double> dis_ahead = {1234.0, 1234.0, 1234.0};
	vector<double> vel_ahead = {MAX_SPEED_MPS+5.0, MAX_SPEED_MPS+5.0, MAX_SPEED_MPS+5.0};

	double ds;
	eLane object_lane; 
	object_info tmp_object_info;
	for (int i=0; i<objects_info.size(); i++){
		tmp_object_info = objects_info[i];
		object_lane = get_lane(tmp_object_info.get_d());
		if (object_lane==eNan) continue;

		ds = tmp_object_info.get_s() - s;
		if (ds<=0 && ds>(-MIN_SAFE_DIS_BEHIND)) ds+=MIN_SAFE_DIS_BEHIND;
		if (ds>0 && ds<dis_ahead[object_lane]){
			dis_ahead[object_lane] = ds;
			vel_ahead[object_lane] = tmp_object_info.get_v();
		}
	}
	return {dis_ahead,vel_ahead};
}

//
// calculating the final "d" location we want
double poly_path::get_next_lane_d(eLane target_lane, double d_start){
	double d_end=(double)(2+4*target_lane);
	double dd = (d_end-d_start);
	
	if (target_lane!=eCenter){ // in order not to exit the road borders - smooth the path.
		dd = MIN(dd, 3.3);
		dd = MAX(dd, -3.3);	
	}
	return d_start+dd;
}

//
// in order to check if we "finished" our lane changing
bool poly_path::get_is_in_lane(eLane target_lane, double d){
	bool ret=true;
	double d0 = (double)(2+4*target_lane);
	double d_limmit = 1.0;
	if (d>(d0+d_limmit)) ret=false;
	if (d<(d0-d_limmit)) ret=false;
	return ret;
}

//
// for debug prints
double poly_path::print_fmt(double x){ 
	int p=100;
	int xq = (int)(x*p);
	return (double)xq/p;
}