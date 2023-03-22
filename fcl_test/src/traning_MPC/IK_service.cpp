
#include <IK_qp.h>

#include <iostream>
#include <fstream>


#include "FCL_TEST/state.h"
#include "FCL_TEST/state_service.h"



//using namespace std;
//using namespace Eigen;
using namespace KDL;
using namespace std;




bool solveIK(FCL_TEST::state_service::Request  &req,
             FCL_TEST::state_service::Response &res)
{
  Eigen::VectorXd Position_disire_(3);
  string path = "/home/mlei/concert_ws/ros_src/modularbot/urdf/ModularBot.urdf";   

  Position_disire_ (0) = double(req.x);
  Position_disire_ (1) = double(req.y);
  Position_disire_ (2) = double(req.z);  
   
  Eigen::VectorXd First_Result;
  Eigen::VectorXd current_position_endeffector;
  
  inverser_kinematical MPC_OPTIMAL_;
  current_position_endeffector = MPC_OPTIMAL_.solvIK(Position_disire_, path ,First_Result);
  
  current_position_endeffector.cast <float> ();
  res.x_ = current_position_endeffector(0);
  res.y_ = current_position_endeffector(1);
  res.z_ = current_position_endeffector(2);
  


    
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.x, (long int)req.y);
  ROS_INFO("sending back response: [%ld]", (long int)res.x_);
  return true;
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "IK"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);


    ros::ServiceServer service = n.advertiseService("IK", solveIK);
    // ros::NodeHandle n_;
  
    // Eigen::VectorXd Position_disire_(3);
    // Position_disire_ (0) = 0.2;
    // Position_disire_ (1) = 1.0;
    // Position_disire_ (2) = 0.2;


    ros::spin();
    return 0;


}
