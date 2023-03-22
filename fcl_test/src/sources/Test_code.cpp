
#include "MPC_Optimal.h"
#include <nav_msgs/Path.h>
#include <xbot_msgs/JointCommand.h>


#include <xbot_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>

using namespace fcl; 
using namespace KDL; 
double dt = 0.01;

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

    ros::init(argc, argv, "Test_code"); //Say to ROS the name of the node and the parameters

    
    ros::NodeHandle nh_("xbotcore");

    Tree my_tree;

    kdl_parser::treeFromFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

    auto cfg = XBot::ConfigOptionsFromParamServer(nh_);
    cfg.set_parameter("is_model_floating_base", false);

    // create robot (note: xbot2 should be up and running)
    auto robot = XBot::RobotInterface::getRobot(cfg);

    // this model is automatically kept in sync with robot
    auto& model = robot->model();

    // set a default control mode (applied to all joints)
    auto default_ctrl_mode = XBot::ControlMode::Position();
    robot->setControlMode(default_ctrl_mode);

    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base_link","TCP_gripper_A",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    //Turn camera to get better FOV
    //_Pub_Desire_(head_camera_joint_cmd_pub, 1.5);
    
    Eigen::VectorXd q0(5), q(5);
    Eigen::VectorXd tau;
    Eigen::VectorXd acceleration(5);
    
    q0.setZero();
    acceleration.setZero();

    int PP=0;
    int p=0;
    ros::Rate loop_rate(100);

    robot->sense();
    robot->getJointPosition(q0);


Eigen::VectorXd pos(5);

while(PP<=500)
{
    robot->sense();
    //  robot->getJointPosition(q);
     Eigen::VectorXd velocity_increment(5);
      for(int j=0;j<5;j++)
      {
        velocity_increment(j) = 0.1;    
           q0(j) = q0(j) - 0.1*dt; 
      }


      //Eigen::VectorXd jointpositions_vector_(5);
       Eigen::VectorXd joint_velcity_(5);
       acceleration.setZero();
       for(int j=0;j<5;j++)
       {
          joint_velcity_(j) = 0.000;      
       }

        model.setJointPosition(q0);
        model.setJointVelocity(joint_velcity_);
        model.setJointAcceleration(acceleration);
        model.update();
        model.computeInverseDynamics(tau);

        std::cout << tau << std::endl; 


        control_robot_postion_test(robot ,model, q0, joint_velcity_, tau);

    ros::spinOnce();
    loop_rate.sleep();
    PP++;
}

// ros::spin();
 return 0;

}
