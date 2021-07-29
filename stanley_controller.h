#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H
#include <vector>
#include <math.h>
#include "lon_controller.h"
#include <fstream>
#define size_t unsigned long int
#define pi 3.1415926

class StanleyParams{
public:
    void set_k_gain(float k_gain);
    float k_gain();
    void set_ts(float ts);
    float ts();

private:
    float k_gain_;
    float ts_;
};

class VehicleParams{
public:
    void set_wheel_base(float wheel_base);
    float wheel_base();
private:
    float wheel_base_;
};

class VehicleState{
public:
    VehicleState(float x,float y,float yaw,float line_v);
    float x ;
    float y ;
    float yaw ;
    float line_v ;
};

typedef struct Point_{
    float x;
    float y;
    float theta = 0;
    /// derivative direction on the x-y plane
}Point;

typedef struct Trajectory_ {
 Point point;
/// curvature on the x-y planning
 float kappa = 0.0;
}Trajectory;

class StanleyController{
public:
   bool stanley_init(StanleyParams &StyPar,VehicleParams &VehPar);
   void Update(VehicleState &state,float acc,float delta,StanleyParams StyPar,VehicleParams VehPar);
   size_t find_closest_point(const VehicleState state,std::vector<Trajectory> trajectory_);
   float stanley_controller(VehicleState state,std::vector<Trajectory> trajectory_,
                            StanleyParams StyPar);
};
#endif // STANLEY_CONTROLLER_H
