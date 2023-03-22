#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>

//using namespace std;
//using namespace Eigen;
using namespace KDL;




Eigen::VectorXd expection_4Obj_4Current_Read(3);
Eigen::VectorXd expection_4Obj_4Current_Oreintation_Read(4);

Eigen::VectorXd expection_4Obj_4Next_Read(6);
Eigen::VectorXd expection_4Obj_4Next_Oreintation_Read(8);

void Topic_Subscrib(const nav_msgs::Path& msg)
{  


    int n = floor(msg.poses.size()/2);
    expection_4Obj_4Next_Read(0) = msg.poses[n].pose.position.x - 0.5;
    expection_4Obj_4Next_Read(1) = msg.poses[n].pose.position.y + 0.0 ;
    expection_4Obj_4Next_Read(2) = msg.poses[n].pose.position.z - 0.5;

    expection_4Obj_4Next_Read(3) = msg.poses[msg.poses.size()-1].pose.position.x - 0.5 ;
    expection_4Obj_4Next_Read(4) = msg.poses[msg.poses.size()-1].pose.position.y + 0.0 ;
    expection_4Obj_4Next_Read(5) = msg.poses[msg.poses.size()-1].pose.position.z -0.5;

    expection_4Obj_4Next_Oreintation_Read(0) = msg.poses[n].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(1) = msg.poses[n].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(2) = msg.poses[n].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(3) = msg.poses[n].pose.orientation.w;

    expection_4Obj_4Next_Oreintation_Read(4) = msg.poses[msg.poses.size()-1].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(5) = msg.poses[msg.poses.size()-1].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(6) = msg.poses[msg.poses.size()-1].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(7) = msg.poses[msg.poses.size()-1].pose.orientation.w;


   // std::cout << "expection_4Obj_4Next_Oreintation_Read" << expection_4Obj_4Next_Oreintation_Read << std::endl;


}




void GTCallback(const geometry_msgs::PoseStamped& msg)
{
        Eigen::VectorXf expection_4Obj_4Current_Read_float(3);
        Eigen::VectorXf expection_4Obj_4Current_Oreintation_Read_float(4);
   //  for (int i=0; i<msg->name.size(); i++)
    
       //  if (msg->name[i].compare("actor") == 0)
        

            // expection_4Obj_4Current_Read(0) = msg->pose[i].position.x-0.5;
            // expection_4Obj_4Current_Read(1) = msg->pose[i].position.y;
            // expection_4Obj_4Current_Read(2) = expection_4Obj_4Next_Read(2);


            expection_4Obj_4Current_Read_float(0)= msg.pose.position.x;
            expection_4Obj_4Current_Read_float(1)= msg.pose.position.y;
            expection_4Obj_4Current_Read_float(2)= msg.pose.position.z;

            //->pose[i].orientation.x;
            expection_4Obj_4Current_Oreintation_Read_float(0) = msg.pose.orientation.x;
            expection_4Obj_4Current_Oreintation_Read_float(1) = msg.pose.orientation.y;
            expection_4Obj_4Current_Oreintation_Read_float(2) = msg.pose.orientation.z;
            expection_4Obj_4Current_Oreintation_Read_float(3) = msg.pose.orientation.w;
   
            //std::cout << "gt_.position.y =="<<std::endl;

     //       break;
        
    
            expection_4Obj_4Current_Read = expection_4Obj_4Current_Read_float.cast<double>();
            expection_4Obj_4Current_Oreintation_Read = expection_4Obj_4Current_Oreintation_Read_float.cast<double>();
             std::cout << "gt_.position.x =="<< expection_4Obj_4Current_Read <<std::endl;
}







Eigen::VectorXd Position_Joint(7);
void poseCallback(const xbot_msgs::JointState& msg)
{
Eigen::VectorXf Position_Joint_float(7);
for(int j =8;j<15; j++)
{   
    //std::cout << msg.link_position[j] << std::endl;
    Position_Joint_float(j-8) = msg.link_position[j];

}
Position_Joint = Position_Joint_float.cast<double>();

//std::cout << Position_Joint_float << std::endl <<std::endl;

}

int show = 1;

void Callshadow(const std_msgs::Int8& msg)
{
    
    show = msg.data;
    
            //std::cout << "gt_.position.x =="<<std::endl;
            //std::cout << "gt_.position.y =="<<std::endl;

}

    using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_Center"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;
    ros::NodeHandle n_;
    ros::NodeHandle nh;

    //ros::Subscriber topic_sub = n.subscribe("/obstacle/lq_path", 1000, Topic_Subscrib);
    //ros::Subscriber ground_truth_sub_ = n_.subscribe("/human_pose", 1000, &GTCallback);
    ros::Subscriber sub = nh.subscribe("/xbotcore/joint_states", 1000, &poseCallback);

    MPC_OPTIMAL MPC_OPTIMAL_;

    ros::Publisher Cen_joint_cmd_pub = MPC_OPTIMAL_.init_Pub_Control_Node(nh);


    Tree my_tree;

    kdl_parser::treeFromFile("/home/oem/hhcm_ws/src/iit-centauro-ros-pkg/centauro_urdf/urdf/centauro.urdf",my_tree);

    YAML::Node arm_param_config = YAML::LoadFile("/home/oem/hhcm_ws/config/Cen_Robot.yaml");

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

    Eigen::VectorXd Desire_position(7);






   int pp = 1;
while(pp<=200)
    {
    
     Desire_position(0) = 0.52;
     Desire_position(1) = 0.32;
     Desire_position(2) = 0.27;
     Desire_position(3) = -2.24;
     Desire_position(4) = 0.05;
     Desire_position(5) = -0.78;
     Desire_position(6) = 0.2;

    MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub,Desire_position);

    ros::spinOnce();

     rate.sleep();
     pp++;

}


return 0; 

}