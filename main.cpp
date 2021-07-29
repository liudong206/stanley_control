#include "mainwindow.h"
#include <QApplication>
#include "lon_controller.h"
#include "stanley_controller.h"
#include <unistd.h>
#include <iostream>
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    //w.show();

    std::vector<Trajectory> trajectory_;
    StanleyController stanley_controller;
    StanleyParams StyPar;
    VehicleParams VehPar;
    VehicleState veh_state(0.0,-3.0,0.0,0.0);
    Trajectory tra;
    stanley_controller.stanley_init(StyPar,VehPar);
     std::cout<<StyPar.k_gain()<<std::endl
             <<StyPar.ts()<<std::endl
            <<VehPar.wheel_base()<<std::endl;

    for (size_t i = 0;i <= 500; ++i) {
        tra.point.x = i;
        tra.point.y = 0;
        tra.point.theta = 0;
        trajectory_.push_back(tra);
    }
    float target_speed = 10.0/3.6; //[m/s]
    float T = 200.0; //最大模拟时间
    float time = 0.0;
    PidControl pid(0.5,0.0,0.0);
    size_t target_index = stanley_controller.find_closest_point(veh_state,trajectory_);
    std::ofstream af("log/veh_state.xls",std::ios::app);
    std::ofstream tf("log/delta.xls",std::ios::app);
    std::ofstream accf("log/acc.xls",std::ios::app);
    std::ofstream vf("log/v.xls",std::ios::app);
    while (time < T && target_index < trajectory_.size() - 5) {
        float delta = stanley_controller.stanley_controller(veh_state,trajectory_,StyPar);
        if(!tf.fail()){
           tf<<delta
             <<std::endl;
        }else {
            std::cout<<"打开文件失败"<<std::endl;
        }
        float acc = pid.AddPid(target_speed,veh_state.line_v);
        if(!accf.fail()){
           accf<<acc
             <<std::endl;
        }else {
            std::cout<<"打开文件失败"<<std::endl;
        }
        stanley_controller.Update(veh_state,acc,delta,StyPar,VehPar);
        if(!vf.fail()){
           vf<<veh_state.line_v
             <<std::endl;
        }else {
            std::cout<<"打开文件失败"<<std::endl;
        }
        time = time + StyPar.ts();
        target_index = stanley_controller.find_closest_point(veh_state,trajectory_);
        if(!af.fail()){
           af<<veh_state.y
             <<std::endl;
        }else {
            std::cout<<"打开文件失败"<<std::endl;
        }
        usleep(2);
    }

     std::ofstream of("log/tra.xls",std::ios::app);
     if(!of.fail()){
         for (size_t j = 0;j < 500;j++) {
            of<<trajectory_[j].point.y<<std::endl;
         }
     }else {
         std::cout<<"打开文件失败"<<std::endl;
     }
     af.close();
     of.close();
     accf.close();
     vf.close();
     std::cout<<"over"<<std::endl;
    return a.exec();
}
