# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
 
[//]: # (Image References)

[image1]: ./finish_line_pic.png "finish_line_pic"
[image2]: ./finish_line_log.png "finish_line_log"

### Files in this project

- main.cpp
- defs.h: macros and constants
- poly_path.cpp, poly_path.h: we calculate and hold the "next" path (for s and d)
- map_wp.cpp, map_wp.h: we calculate and hold the spline of the given highway map
- vehicle_info.cpp, vehicle_info.h: vehicle info for "jmt" (s,s_dot,s_dot_dot, d,d_dot,d_dot_dot)
- object_info.cpp, object_info.h: "other" objects info from the sensor fusion.
- spline.h.


### Finish Line snapshoot and log
![alt text][image1]

![alt text][image2]


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

		- from the telemetry, if this is the first time. (lines 256-260 in main.cpp):
		- from the prev path saved. We saved the estimated info (~1 sec into the future), and now we see exactly where we are which is 'fine_location'. (lines 241-244, 255) in main.cpp)
2. we also read the other cars info and save them in 'objects_info' (lines 248-251 in main.cpp)
3. we will use 'jmt' (lines 114-115, 128-171 in poly_path.cpp) to get a 6th order poly to describe the path for s and d (lines 118-123 in poly_path.cpp):

		- we have the initial state (lines 30-32 in poly_path.cpp).
		- we look at the other cars ahead of us or close to us and behind us (MIN_SAFE_DIS_BEHIND). for each lane we find the closest car and calculate the distance from it and the velocity of that car (lines 38-40, 207-227 in poly_path.cpp)
		- now we have a simple FSM that will set our target location (s,d) and speed (s): (lines 45-70 in poly_path.cpp)
			- if we are in the middle of the lane change we wait for it to finish, before we change to another lane. This is done because we don't wont to stay in between lanes for too much time (47, 244-248 in poly_path.cpp) . 
			- if the road ahead is clear:
				- if we are at the center lane we continue and set the target speed toward the maximal speed (the two ifs are false)
				- if we are not at the center lane and the center lane is open we change to the center lane and set the target speed toward the maximal speed (the first if is false and the second is true). This is done because the center lane is the best place to be: we have two options to change lanes from it.
			- if the road ahead is not clear (MIN_SAFE_DIS_AHEAD) and there is a nearby lane that is clear we change to it. If there are two clear lanes (only when we are at the center lane) we take the clearer lane (the first if is true)
			- if there is no clear lane we set the velocity the the velocity of the car that is ahead of us (lines 75-79 in poly_path.cpp)
4. we predict the car info for about 1 second into the future (lines 275-277 in main.cpp)
5. we calculate the target speed ("s speed") (lines 73-87 in poly_path.cpp). We limit the velocity change is order to to have huge acceleration (lines 80-81 in poly_path.cpp)
6. we calculate the target location:
7. 
		- s: according to "s speed" (lines 80,110 in poly_path.cpp)
		- d: according to the target lane. Here we don't want to get overshoot and find the car outside to road so we limit the difference in the change (lines 81, 221-230 in poly_path.cpp)
8. the rest of the target values are set to zero (lines 110-111 in poly_path.cpp)
9. we use the "jmt" function (lines 114-115 in poly_path.cpp), and find the path (lines 118-123 in poly_path.cpp)


### Merging the next path with the prev path:
1. at the start of the world (prev_path_sz=0), we just use the next path (lines 309-312 in main.cpp)
for the other case:
2. the first 'marge_path_low' (15) samples are taken from the pre path (lines 288, 301-302 in main.cpp)
3. the up to 'marge_path_high' (25) we average the two paths according to linear weight (lines 293, 301-302 in main.cpp)
4. all the rest of the samples are from the next path (lines 290, 301-302, 296-297 in main.cpp). This is why we could already save the samples for the next update only from the next path.

	
### Remarks:
1. we make sure to unwrap the "s" variable both for our car (lines 215-217 in main.cpp) and for the other cars (lines 28-31 in object_info.cpp)
2. we pad (with 10 samples) the highway map in order to maintain continuity on "s" at the end of the track (lines 49-55 in map_wp.cpp)
3. because there is randomness in the simulator there could be new cases each time we run the simulator with our code. Also there could be cases when the other cars run into us. for example we are driving parallel to each other, and the other car start to change lanes to our lane.


