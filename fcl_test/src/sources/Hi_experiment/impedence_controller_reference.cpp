#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float32.h>
#include "FCL_TEST/state.h"

//using namespace std;
//using namespace Eigen;
using namespace KDL;
using namespace std;


void control_robot_postion_test(auto robot ,auto& model, Eigen::VectorXd desire_position, Eigen::VectorXd desire_velocity, Eigen::VectorXd tau)
{

        //model.update();
        robot->setPositionReference(desire_position);
        robot->setVelocityReference(desire_velocity);       
        robot->setEffortReference(tau);
        robot->move();

}





int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_modular"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle nh_("xbotcore");

    ros::NodeHandle n;
    ros::NodeHandle n_;
    

    Tree my_tree;

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

   double Maxx_x, Maxx_y, Maxx_z;
   double Damping_x, Damping_y, Damping_z;
   double Spring_x, Spring_y, Spring_z;

   Maxx_x = 0.1;
   Maxx_y = 0.1;
   Maxx_z = 0.1;

   Damping_x = 180;
   Damping_y = 180;
   Damping_z = 180;

   Spring_x = 1;
   Spring_y = 1;
   Spring_z = 1;

   Eigen::Vector3d Desire_pos;
   Eigen::Vector3d Desire_vel;
   Eigen::Vector3d Desire_acc;
  
   Desire_pos.setZero();
   Desire_vel.setZero();
   Desire_acc.setZero();

   Eigen::Vector3d Desire_Force; 
   Desire_Force.setZero();
   Eigen::Vector3d Real_Force; 
   Real_Force.setZero();
   
   Eigen::Matrix3d Mass_;
   Eigen::Matrix3d Damping_;
   Eigen::Matrix3d Spring_;
   
   Mass_.setZero();
   Damping_.setZero();  
   Spring_.setZero();  

   Mass_(0,0) = Maxx_x;
   Mass_(1,1) = Maxx_y;
   Mass_(2,2) = Maxx_z;

   Damping_(0,0) = Damping_x;
   Damping_(1,1) = Damping_y;
   Damping_(2,2) = Damping_z;


   Spring_(0,0) = Spring_x;
   Spring_(1,1) = Spring_y;
   Spring_(2,2) = Spring_z;

   pp = 0;

   Eigen::Vector3d Real_pos;
   Eigen::Vector3d Real_vel;
   Eigen::Vector3d Real_acc;
  
   Real_pos.setZero();
   Real_vel.setZero();
   Real_acc.setZero();

   Real_pos = Spring_.inverse() * (Real_Force - Desire_Force) - Spring_.inverse() * (Mass_ * Real_acc + Damping_ * Real_vel) + Desire_pos;

      while(pp<=1000)
    {
     

   
    

    pp++;
 
    }    
return 0; 


}
