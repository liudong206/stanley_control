#include "lon_controller.h"
PidControl::PidControl(float p,float i,float d){
    Kp = p;
    Ki = i;
    Kd = d;
}
float PidControl::LocationPid(float SetValue,float ActualValue){
    error_k = SetValue - ActualValue;//calculate current error
    error_sum += error_k;
    float LocPidResult = Kp * error_k + Ki * error_sum + Kd * (error_k1 - error_k);
    error_k1 = error_k;
    return LocPidResult;
}
float PidControl::AddPid(float SetValue,float ActualValue){
    error_k = SetValue - ActualValue;
    float AddPidResult = Kp * (error_k - error_k1) + Ki * error_k + Kd * (error_k - 2*error_k1 + error_k2);
    error_k2 = error_k1;
    error_k1 = error_k2;
    return AddPidResult;
}

