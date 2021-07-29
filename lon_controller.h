#ifndef LON_CONTROLLER_H
#define LON_CONTROLLER_H

class PidControl{
public:
    PidControl(float Kp,float Ki,float Kd);
    float Kp;
    float Ki;
    float Kd;
    float error_k = 0.0;//current error
    float error_k1 = 0.0;//Last time error
    float error_k2 = 0.0;//last last time error
    float error_sum = 0.0 ;//error sum
    float LocationPid(float SetValue,float ActualValue);
    float AddPid(float SetValue,float ActualValue);
};
#endif // LON_CONTROLLER_H
