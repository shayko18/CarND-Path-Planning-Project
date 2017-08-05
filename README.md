# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
 

### Files in this project

- main.cpp
- defs.h: macros and constants
- poly_path.cpp, poly_path.h: we calculate and hold the "next" path (for s and d)
- map_wp.cpp, map_wp.h: we calculate and hold the spline of the given highway map
- vehicle_info.cpp, vehicle_info.h: vehicle info for "jmt" (s,s_dot,s_dot_dot, d,d_dot,d_dot_dot)
- object_info.cpp, object_info.h: "other" objects info from the sensor fusion.
- spline.h.


### Highlevel algorithm

In order to drive the car we have at each "cycle" (telemetry event):
- the "prev" path. (empty the the start of the world)

if this is an "update cycle", which is every 1 sec (lines 235-240 in main.cpp):
 
	- we calculate the "next" path. (lines 271 in main.cpp)
	- we merge the two in order to keep thing smooth. (lines 280-314 in main.cpp)
otherwise:

	- we continue with the prev path (lines 315-319 in main.cpp)



### Calculating the next path:
1. the initial state is saved in 'my_vehicle_info' and it is taken form:
2. 
		- from the telemetry, if this is the first time. (lines 256-260 in main.cpp):
		- from the prev path saved. We saved the estimated info (~1 sec into the future), and now we see exactly where we are which is 'fine_location'. (lines 241-244, 255) in main.cpp)
2. we also read the other cars info and save them in 'objects_info' (lines 248-251 in main.cpp)
3. we will use 'jmt' (lines 104-105, 118-161 in poly_path.cpp) to get a 6th order poly to describe the path for s and d (lines 108-113 in poly_path.cpp):
4. 
		- we have the initial state (lines 30-31 in poly_path.cpp).
		- we look at the other cars ahead of us. for each lane we find the closest car and calculate the distance from it and the velocity of that car (lines 37-39, 197-217 in poly_path.cpp)
		- now we have a simple FSM that will set our target location (s,d) and speed (s): (lines 45-81 in poly_path.cpp)
			- if the road ahead is clear:
				- if we are at the center lane we continue and set the target speed toward the maximal speed (the two ifs are false)
				- if we are not at the center lane and the center lane is open we change to the center lane and set the target speed toward the maximal speed (the first if is false and the second is true). This is done because the center lane is the best place to be: we have two options to change lanes from it.
			- if the road ahead is not clear and there is a nearby lane that is clear we change to it. If there are two clear lanes (only when we are at the center lane) we take the clearer lane (the first if is true)
			- if there is no clear lane we set the velocity the the velocity of the car that is ahead of us (lines 65-68 in poly_path.cpp)
4. we predict the car info for about 1 second into the future (lines 275-277 in main.cpp)
5. we calculate the target speed ("s speed") (lines 63-77 in poly_path.cpp). We limit the velocity change is order to to have huge acceleration (lines 70-71 in poly_path.cpp)
6. we calculate the target location:
7. 
		- s: according to "s speed" (lines 79,100 in poly_path.cpp)
		- d: according to the target lane. Here we don't want to get overshoot and find the car outside to road so we limit the difference in the change (lines 81, 221-230 in poly_path.cpp)
7. the rest of the target values are set to zero (lines 100-101 in poly_path.cpp)
8. we use the "jmt" function (lines 104-105 in poly_path.cpp), and find the path (lines 108-113 in poly_path.cpp)


### Merging the next path with the prev path:
1. at the start of the world (prev_path_sz=0), we just use the next path (lines 309-312 in main.cpp)
for the other case:
2. the first 'marge_path_low' (15) samples are taken from the pre path (lines 288, 301-302 in main.cpp)
3. the up to 'marge_path_high' (25) we average the two paths according to linear weight (lines 293, 301-302 in main.cpp)
4. all the rest of the samples are from the next path (lines 290, 301-302, 296-297 in main.cpp). This is why we could already save the samples for the next update only from the next path.

	
### Remarks:
1. we make sure to unwrap the "s" variable both for our car (lines 215-217 in main.cpp) and for the other cars (lines 28-31 in object_info.cpp)
2. we pad (with 10 samples) the highway map in order to maintain continuity on "s" at the end of the track (lines 49-55 in map_wp.cpp)
3. because there is random in the simulator there could be new cases each time we run the simulator with our code.











  
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
