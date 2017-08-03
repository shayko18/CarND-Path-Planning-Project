#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"

#include "defs.h"
#include "spline.h"
#include "map_wp.h"
#include "poly_path.h"
#include "vehicle_info.h"
#include "object_info.h"


using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }
double mph2mps(double x) { return x * 0.44704; }

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
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
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

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
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
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
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

static int global_cnt=0;
static int loop_cnt=0;
static double prev_car_s=-1.0;
static double car_s_init=-1.0;

int main() {  
  uWS::Hub h;
  map_wp map_waypoints;
  poly_path poly_path_alg;
  vector<vehicle_info> my_vehicle_info_pm(16);

  h.onMessage([&map_waypoints, &my_vehicle_info_pm, &poly_path_alg](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
		
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
		  // Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
			car_speed = mph2mps(car_speed);
			
			if (prev_car_s>car_s){
				car_s+=MAX_S;
			}

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
			
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

			vector<double> next_x_vals;
			vector<double> next_y_vals;
			
			const int next_path_sz = 200;
			const int tolerance_sz = my_vehicle_info_pm.size();
			const int update_rate_sz = (UPDATE_RATE/DT)-(tolerance_sz>>1);
			const int prev_path_sz = previous_path_x.size();

			if (prev_path_sz < (next_path_sz-update_rate_sz)){
				int fine_location = MIN(next_path_sz-update_rate_sz-prev_path_sz, tolerance_sz-1);
				if (prev_path_sz==0){
					fine_location = 0;
				}

				vector<object_info> objects_info(sensor_fusion.size());
				for (int i = 0; i < sensor_fusion.size(); i++) {
					objects_info[i].set(sensor_fusion[i][0],sensor_fusion[i][5],sensor_fusion[i][6],sensor_fusion[i][3],sensor_fusion[i][4], car_s);
				}

				vehicle_info my_vehicle_info = my_vehicle_info_pm[fine_location];
				if (prev_path_sz==0){
					my_vehicle_info.set_s(car_s);
					my_vehicle_info.set_d(car_d);
					car_s_init = car_s;
				}
				else{
					if ((int)((car_s-car_s_init)/MAX_S) == loop_cnt+1){
						cout << endl;
						cout << " ^^^^^ FINISH " << ++loop_cnt << " LOOP ^^^^" << endl;
						cout << endl;
					}
				}

				poly_path_alg.calc_path_sd(my_vehicle_info.get_vec(), objects_info, next_path_sz);
				for (int i=0; i<tolerance_sz; i++){
					my_vehicle_info_pm[i].set_vec(poly_path_alg.est_vehicle_info(i+update_rate_sz)); 
				}
				
				const int marge_path_low = 15;
				const int marge_path_high = marge_path_low+10;
				vector<double> next_xy_val;
				double next_x, next_y;
				for (int i=0; i<next_path_sz; i++){
					next_xy_val = map_waypoints.getXY_spline(poly_path_alg.get_path_s(i), poly_path_alg.get_path_d(i));
					if (prev_path_sz){
						double alpha = 1.0;
						if (i > marge_path_high) {
							alpha = 0.0;
						}
						else if (i>marge_path_low){
							alpha = ((double)(marge_path_high-i)) / (double)(marge_path_high-marge_path_low);
						}
						
						if (i>=prev_path_sz){
							next_x = next_xy_val[0];
							next_y = next_xy_val[1];	
						}
						else{
							next_x = alpha*(double)previous_path_x[i] + (1.0-alpha)*next_xy_val[0];
							next_y = alpha*(double)previous_path_y[i] + (1.0-alpha)*next_xy_val[1];						
						}

					
						next_x_vals.push_back(next_x);
						next_y_vals.push_back(next_y);
					}
					else{
						next_x_vals.push_back(next_xy_val[0]);
						next_y_vals.push_back(next_xy_val[1]);
					}				
				}
			}
			else{
				for(int i=0; i<prev_path_sz; i++) {
					next_x_vals.push_back(previous_path_x[i]);
					next_y_vals.push_back(previous_path_y[i]);
				}
			}

			prev_car_s = car_s;
			//map_waypoints.print_path(next_x_vals,next_y_vals);
			//cout << "xx cnt=" << ++global_cnt << " xx" << endl;
			
	
			
			// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
			msgJson["next_x"] = next_x_vals;
			msgJson["next_y"] = next_y_vals;
			
          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}