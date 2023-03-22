#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>

using namespace fcl; 
double dt = 0.01;

Eigen::VectorXd Position_Joint(7);
void poseCallback(const xbot_msgs::JointState& msg)
{
Eigen::VectorXf Position_Joint_float(7);
for(int j =25;j<32; j++)
{   
    //std::cout << msg.link_position[j] << std::endl;
    Position_Joint_float(j-25) = msg.link_position[j];

}
Position_Joint = Position_Joint_float.cast<double>();


}

void init_Sub(ros::NodeHandle nh_)
{
  
  ros::Subscriber sub = nh_.subscribe("/xbotcore/joint_states", 100, &poseCallback);
}

auto init_Pub_Control_Node(ros::NodeHandle nh_)
{
  
  auto head_camera_joint_cmd_pub = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command",10);
  return head_camera_joint_cmd_pub;
}

void _Pub_Desire_(auto head_camera_joint_cmd_pub, Eigen::VectorXd position)
{
    xbot_msgs::JointCommand j_arm1_1;
    xbot_msgs::JointCommand j_arm1_2;
    xbot_msgs::JointCommand j_arm1_3;
    xbot_msgs::JointCommand j_arm1_4;
    xbot_msgs::JointCommand j_arm1_5;
    xbot_msgs::JointCommand j_arm1_6;
    xbot_msgs::JointCommand j_arm1_7;

    j_arm1_1.name.push_back("j_arm1_1");
    j_arm1_1.ctrl_mode.push_back(9);
    //head_camera_cmd.stiffness.push_back(1000);
    //head_camera_cmd.position.push_back(0.62);
    j_arm1_2.name.push_back("j_arm1_2");
    j_arm1_2.ctrl_mode.push_back(9);

    j_arm1_3.name.push_back("j_arm1_3");
    j_arm1_3.ctrl_mode.push_back(9);

    j_arm1_4.name.push_back("j_arm1_4");
    j_arm1_4.ctrl_mode.push_back(9);

    j_arm1_5.name.push_back("j_arm1_5");
    j_arm1_5.ctrl_mode.push_back(9);

    j_arm1_6.name.push_back("j_arm1_6");
    j_arm1_6.ctrl_mode.push_back(9);

    j_arm1_7.name.push_back("j_arm1_7");
    j_arm1_7.ctrl_mode.push_back(9);

    j_arm1_1.position.push_back(position(0));
    j_arm1_2.position.push_back(position(1));
    j_arm1_3.position.push_back(position(2));
    j_arm1_4.position.push_back(position(3));
    j_arm1_5.position.push_back(position(4));
    j_arm1_6.position.push_back(position(5));
    j_arm1_7.position.push_back(position(6));

    head_camera_joint_cmd_pub.publish(j_arm1_1);
    head_camera_joint_cmd_pub.publish(j_arm1_2);
    head_camera_joint_cmd_pub.publish(j_arm1_3);
    head_camera_joint_cmd_pub.publish(j_arm1_4);
    head_camera_joint_cmd_pub.publish(j_arm1_5);
    head_camera_joint_cmd_pub.publish(j_arm1_6);
    head_camera_joint_cmd_pub.publish(j_arm1_7);
}






int main(int argc, char **argv)
{

    ros::init(argc, argv, "Test_code"); //Say to ROS the name of the node and the parameters

    
    ros::NodeHandle nh_;

    //init_Sub(nh_);
    ros::Subscriber sub = nh_.subscribe("/xbotcore/joint_states", 1000, &poseCallback);
    auto head_camera_joint_cmd_pub = init_Pub_Control_Node(nh_);

    //Turn camera to get better FOV
    //_Pub_Desire_(head_camera_joint_cmd_pub, 1.5);
   

int PP=0;
int p=0;
ros::Rate loop_rate(100);
std::cout << "Position_Joint=" << Position_Joint << std::endl << std::endl;

Eigen::VectorXd pos(7);

while(PP<=500)
{

for (int i=0;i<7;i++)
{
pos(i) = 0;
pos(1) = 1.5;
}
     _Pub_Desire_(head_camera_joint_cmd_pub, pos);

std::cout <<Position_Joint - pos  << std::endl<<std::endl;

    p=0.01*PP+ p ;
    PP++;
    ros::spinOnce();
    loop_rate.sleep();
}

// ros::spin();
 return 0;

}
