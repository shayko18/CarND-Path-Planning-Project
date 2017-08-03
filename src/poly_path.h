#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/LU"
#include "spline.h"
#include "defs.h"


class poly_path{
public:
	enum eLane {eLeft, eCenter, eRight, eNan};
	poly_path();
	~poly_path();
		
	void calc_path_sd(std::vector<double> vehicle_info, std::vector<class object_info> objects_info, int N); 
	std::vector<double> est_vehicle_info(int n);
	
	int get_path_size(){return path_s.size();}
	double get_path_s(int i){return path_s[i];}
	double get_path_d(int i){return path_d[i];}
	
private:	
	std::vector<double> jmt(std::vector<double> start, std::vector<double> end, double T);
	double poly_val(std::vector<double> coeff, double t);
	std::vector<std::vector<double>> get_dis_val_ahead(double s, std::vector<class object_info> objects_info);
	
	
	eLane get_lane(double d){return (d<=12.0 && d>=0.0)?(eLane)(int)(d/4):eNan;}
	double get_next_lane_d(eLane target_lane, double d_start, bool lane_change);
	double print_fmt(double x);
	
	std::vector<double> path_s;
	std::vector<double> path_d;
};