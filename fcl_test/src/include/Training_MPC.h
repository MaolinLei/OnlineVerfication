#ifndef _TRAININGmpc_H_
#define _TRAININGmpc_H_

#include "ros/ros.h"
#include <ros/package.h>
#include <MPC_Optimal.h>

#include <Eigen/Dense>


	

class Training_MPC{

private:

dt = 0.01

public:





Training_MPC();
~Training_MPC();

KDL::Tree my_tree;
traj traj;
KDL::Chain chain;
int number_desired_point;


Eigen::Vector3d admittance_feedback_Position(Eigen::Vector3d Desire_Force, Eigen::Vector3d Desire_acc, Eigen::Vector3d Desire_vel, Eigen::Vector3d Desire_pos);


};
#endif