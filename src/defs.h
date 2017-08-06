#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)>(b))?(b):(a))
#define MS_2_MPH(a) (2.23693629*(a))

#define MAX_S        (6945.554) 
#define DT           (0.02) // every sample is about 0.02 sec
#define UPDATE_RATE  (1.0) // every sample is about 1 sec
#define MAX_SPEED_MPH (47.0) // ~50 mph
#define MAX_SPEED_MPS (MAX_SPEED_MPH*0.44704) // in meter/sec
#define MIN_SAFE_DIS_AHEAD (MAX_SPEED_MPS*UPDATE_RATE*2.5) // safe distance ahead before we change lanes
#define MIN_SAFE_DIS_BEHIND (MIN_SAFE_DIS_AHEAD*0.20) // safe distance behind before we change lanes