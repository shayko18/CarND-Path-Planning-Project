#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "spline.h"
#include "defs.h"


class vehicle_info{
public:
	vehicle_info();
	
	void set(double s_, double s_dot_, double s_dot_dot_, double d_, double d_dot_, double d_dot_dot_);
	
	void set_vec(std::vector<double> vec);
	void set_s(double x){s=x;}
	void set_s_dot(double x){s_dot=x;}
	void set_s_dot_dot(double x){s_dot_dot=x;}
	void set_d(double x){d=x;}
	void set_d_dot(double x){d_dot=x;}
	void set_d_dot_dot(double x){d_dot_dot=x;}
	
	std::vector<double> get_vec();
	double get_s(){return s;}
	double get_s_dot(){return s_dot;}
	double get_s_dot_dot(){return s_dot_dot;}
	double get_d(){return d;}
	double get_d_dot(){return d_dot;}
	double get_d_dot_dot(){return d_dot_dot;}
	
	
private:	
	double s;
	double s_dot;
	double s_dot_dot;
	
	double d;
	double d_dot;
	double d_dot_dot;

};