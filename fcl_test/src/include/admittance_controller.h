#ifndef _Admittance_H_
#define _Admittance_H_

#include "ros/ros.h"
#include <ros/package.h>




#include <Eigen/Dense>


	

class admittance_controller{

private:


public:

admittance_controller();
~admittance_controller();

const double Maxx_x = 0.0;
const double Maxx_y = 0.0;
const double Maxx_z = 0.0;

const double Damping_x = 0.180;
const double Damping_y = 0.180;
const double Damping_z = 0.180;

const double Spring_x = 100;
const double Spring_y = 100;
const double Spring_z = 100;

Eigen::Matrix3d Mass_;
Eigen::Matrix3d Damping_;
Eigen::Matrix3d Spring_;
   
Eigen::Vector3d Real_pos;
Eigen::Vector3d Real_vel;
Eigen::Vector3d Real_acc;
Eigen::Vector3d Real_For;



void admittance_feedback_init();
void admittance_feedback_init_current_state(Eigen::VectorXd Real_Force_, Eigen::VectorXd Real_acceleartion_, Eigen::VectorXd Real_velocity_, Eigen::VectorXd Real_position_);

Eigen::Vector3d admittance_feedback_Position(Eigen::Vector3d Desire_Force, Eigen::Vector3d Desire_acc, Eigen::Vector3d Desire_vel, Eigen::Vector3d Desire_pos);


};
#endif