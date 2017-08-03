#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "spline.h"
#include "defs.h"


class object_info{
public:
	object_info();
	
	void set(int id_, double s_, double d_, double vx_, double vy_);
	double get_s(){return s;}
	double get_d(){return d;}
	double get_vx(){return vx;}
	double get_vy(){return vy;}
	double get_v();
		
private:
	int id;
	double s;
	double d;
	double vx;
	double vy;
};