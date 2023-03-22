#include "admittance_controller.h"



admittance_controller::admittance_controller()
{

}

admittance_controller::~admittance_controller()
{

}

void admittance_controller::admittance_feedback_init()
{
   Mass_(0,0) = Maxx_x;
   Mass_(1,1) = Maxx_y;
   Mass_(2,2) = Maxx_z;

   Damping_(0,0) = Damping_x;
   Damping_(1,1) = Damping_y;
   Damping_(2,2) = Damping_z;

   Spring_(0,0) = Spring_x;
   Spring_(1,1) = Spring_y;
   Spring_(2,2) = Spring_z;

   Real_pos.setZero();
   Real_vel.setZero();
   Real_acc.setZero();


}

Eigen::Vector3d admittance_controller::admittance_feedback_Position(Eigen::Vector3d Desire_Force, Eigen::Vector3d Desire_acc, Eigen::Vector3d Desire_vel, Eigen::Vector3d Desire_pos)
{
   Eigen::Vector3d Real_pos;
   Real_pos = Spring_.inverse() * (Real_For - Desire_Force) - Spring_.inverse() * (Mass_ * Real_acc + Damping_ * Real_vel) + Desire_pos;

   return Real_pos;

}


void admittance_controller::admittance_feedback_init_current_state(Eigen::VectorXd Real_Force_, Eigen::VectorXd Real_acceleartion_, Eigen::VectorXd Real_velocity_, Eigen::VectorXd Real_position_)
{
  Real_For = Real_Force_.topRows(3);
  Real_For(0) = 0;
  Real_For(1) = 0;
  Real_For(2) = 10;
  Real_acc = Real_acceleartion_.topRows(3);
  Real_vel = Real_velocity_.topRows(3);
  Real_pos = Real_position_.topRows(3);

}




