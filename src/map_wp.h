#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "spline.h"
#include "defs.h"


class map_wp{
public:
	map_wp();
	~map_wp();

	std::vector<double> getXY_spline(double s, double d);
	void print_path(std::vector<double>next_x_vals, std::vector<double>next_y_vals);
	

private:	
	std::vector<double> map_waypoints_x;
	std::vector<double> map_waypoints_y;
	std::vector<double> map_waypoints_s;
	std::vector<double> map_waypoints_dx;
	std::vector<double> map_waypoints_dy;
	
	tk::spline spline_x;
	tk::spline spline_y;
	tk::spline spline_dx;
	tk::spline spline_dy;
	
	double	max_s;
	std::ofstream dbg_path_xy;
	
	void calc_splines();
};
