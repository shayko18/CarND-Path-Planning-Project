#define MAX(a,b) (((a)>(b))?(a):(b))
#define MIN(a,b) (((a)>(b))?(b):(a))
#define ABS(a) (((a)>(0))?(a):(-a))
#define MS_2_MPH(a) (2.23693629*(a))

#define MAX_S        (6945.554) 
#define DT           (0.02) // every sample is about 0.02 sec
#define UPDATE_RATE  (1.0) // every sample is about 1 sec
#define MAX_SPEED_MPH (47.0) // ~50 mph
#define MAX_SPEED_MPS (MAX_SPEED_MPH*0.44704) // in meter/sec
#define MIN_SAFE_DIS (MAX_SPEED_MPS*UPDATE_RATE*2.0) // safe distance before we change lanes