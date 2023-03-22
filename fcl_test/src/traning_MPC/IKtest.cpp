
#include <IK_qp.h>



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


int main(int argc, char **argv)
{

    ros::init(argc, argv, "Training_MPC"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle nh_("xbotcore");

    ros::NodeHandle n;

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

    // ros::NodeHandle n_;
    Eigen::VectorXd Position_disire_(3);
    Position_disire_ (0) = 0.2;
    Position_disire_ (1) = 1.0;
    Position_disire_ (2) = 0.2;
    
    inverser_kinematical MPC_OPTIMAL_;
    Eigen::VectorXd First_Result;
    int pp = 0;
    string path; 
    path = "/home/mlei/catkin_ws/src/modularbot/urdf/ModularBot.urdf";
while(pp<=1)
{

     MPC_OPTIMAL_.solvIK(Position_disire_, path ,First_Result);

     // std::cout << First_Result << std::endl;
     pp++;  

    ros::spinOnce();

     rate.sleep();
    }


 

return 0; 


}
