#include "stanley_controller.h"

VehicleState::VehicleState(float x,float y,float yaw,float line_v):x(x),y(y),yaw(yaw),line_v(line_v){}

void StanleyParams::set_k_gain(float k_gain){
    k_gain_ = k_gain;
}
float StanleyParams::k_gain(){
    return k_gain_;
}
void StanleyParams::set_ts(float ts){
    ts_ = ts;
}
float StanleyParams::ts(){
    return ts_;
}
void VehicleParams::set_wheel_base(float wheel_base){
    wheel_base_ = wheel_base;
}
float VehicleParams::wheel_base(){
    return wheel_base_;
}

bool StanleyController::stanley_init(StanleyParams &StyPar,VehicleParams &VehPar){
    float k = 0.05;// 增益参数
    float ts = 0.8;// 时间间隔，单位：s
    float L = 3.0;// 车辆轴距，单位：m
    StyPar.set_k_gain(k);
    StyPar.set_ts(ts);
    VehPar.set_wheel_base(L);
    return true;
}

size_t StanleyController::find_closest_point(VehicleState state,std::vector<Trajectory> trajectory_){
   size_t tra_size = trajectory_.size();
   size_t index_min = 0;
   auto fun_distance_squre = [](Point point,VehicleState state){
       float dx = point.x - state.x;
       float dy = point.y - state.y;
       return dx * dx + dy + dy;
   };
   float distance = fun_distance_squre(trajectory_.front().point,state);
   for (size_t i = 1 ; i < tra_size ; ++i) {
       float d_temp = fun_distance_squre(trajectory_[i].point,state);
       if(d_temp < distance){
           distance = d_temp;
           index_min = i;
       }
   }
   return index_min;
}
void StanleyController::Update(VehicleState &state,float acc,float delta,
                               StanleyParams StyPar,VehicleParams VehPar){
    state.x = state.x + state.line_v * cos(state.yaw) * StyPar.ts();
    state.y = state.y + state.line_v * sin(state.yaw) * StyPar.ts();
    state.yaw = state.yaw + state.line_v / VehPar.wheel_base() * tan(delta) * StyPar.ts();
    state.line_v = state.line_v + acc *StyPar.ts();
}
float StanleyController::stanley_controller(VehicleState state,std::vector<Trajectory> trajectory_,
                                            StanleyParams StyPar){
    size_t index = find_closest_point(state,trajectory_);
    auto dis_error = 0.0;
    Point clo_point;
    if(index >= trajectory_.size()){
        size_t ind = trajectory_.size();
        clo_point.x = trajectory_[ind].point.x;
        clo_point.y = trajectory_[ind].point.y;
        clo_point.theta = trajectory_[ind].point.theta;
    }
    if(index < trajectory_.size()) {
        clo_point.x = trajectory_[index].point.x;
        clo_point.y = trajectory_[index].point.y;
        clo_point.theta = trajectory_[index].point.theta;
    }
    //Calculate lateral error
    if((state.x - clo_point.x) * clo_point.theta - (state.y - clo_point.y) > 0){
        dis_error =sqrt(pow((state.x - clo_point.x),2) + pow((state.y - clo_point.y),2));
    }else {
    dis_error = - sqrt(pow((state.x - clo_point.x),2) + pow((state.y - clo_point.y),2));
    }
    float theta_fai = clo_point.theta - state.yaw;
    //float theta_fai = atan((clo_point.y - state.y)/(clo_point.x - state.x)) - state.yaw;
    float theta_y = atan2(StyPar.k_gain() * dis_error,state.line_v);
    float delta = theta_fai + theta_y;
    //steerlimit 5
    if(delta > pi/6.0){
       delta = pi/6.0;
    }else if(delta < -pi/6.0){
        delta = -pi/6.0;
    }
    return delta;
}

