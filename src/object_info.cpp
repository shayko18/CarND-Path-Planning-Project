#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include "object_info.h"

using namespace std;

object_info::object_info(){
	id=-1;
	s=0.0;
	d=0.0;
	vx=0.0;
	vy=0.0;
}

void object_info::set(int id_, double s_, double d_, double vx_, double vy_, double car_s){
	id=id_;
	s=s_;
	d=d_;
	vx=vx_*0.44704;
	vy=vy_*0.44704;
	
	int k = (int)((car_s-s)/(MAX_S*0.5));
	k = (k+1)>>1;
	
	s+=(k*MAX_S);
	
	bool log_enable = false;
	if (log_enable){
		cout << "id=" << id;
		cout << ", s=" << s;
		cout << ", d=" << d;
		cout << ", vx=" << vx;
		cout << ", vy=" << vy;
		cout << endl;
	}
}

double object_info::get_v(){
	double v = sqrt(vx*vx + vy*vy);;
	return v;
}

