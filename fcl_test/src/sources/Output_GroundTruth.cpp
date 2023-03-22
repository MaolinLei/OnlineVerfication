#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>

#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int8.h>

Eigen::VectorXd expection_4Obj_4Current_Read(3);
Eigen::VectorXd expection_4Obj_4Current_Oreintation_Read(4);

Eigen::VectorXd expection_4Obj_4Next_Read(6);
Eigen::VectorXd expection_4Obj_4Next_Oreintation_Read(8);


    using namespace std;





void Topic_Subscrib(const nav_msgs::Path& msg)
{  


    int n = floor(msg.poses.size()/2);
    expection_4Obj_4Next_Read(0) = msg.poses[n].pose.position.x - 0.5;
    expection_4Obj_4Next_Read(1) = msg.poses[n].pose.position.y+  0.0 ;
    expection_4Obj_4Next_Read(2) = msg.poses[n].pose.position.z;

    expection_4Obj_4Next_Read(3) = msg.poses[msg.poses.size()-1].pose.position.x - 0.5 ;
    expection_4Obj_4Next_Read(4) = msg.poses[msg.poses.size()-1].pose.position.y+ 0.0 ;
    expection_4Obj_4Next_Read(5) = msg.poses[msg.poses.size()-1].pose.position.z;

    expection_4Obj_4Next_Oreintation_Read(0) = msg.poses[n].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(1) = msg.poses[n].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(2) = msg.poses[n].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(3) = msg.poses[n].pose.orientation.w;

    expection_4Obj_4Next_Oreintation_Read(4) = msg.poses[msg.poses.size()-1].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(5) = msg.poses[msg.poses.size()-1].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(6) = msg.poses[msg.poses.size()-1].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(7) = msg.poses[msg.poses.size()-1].pose.orientation.w;

            std::cout << expection_4Obj_4Next_Read<<std::endl;

}




void GTCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("actor") == 0)
        {

            expection_4Obj_4Current_Read(0) = msg->pose[i].position.x - 0.5;
            expection_4Obj_4Current_Read(1) = msg->pose[i].position.y;
            expection_4Obj_4Current_Read(2) = expection_4Obj_4Next_Read(2);

            expection_4Obj_4Current_Oreintation_Read(0)= msg->pose[i].orientation.x;
            expection_4Obj_4Current_Oreintation_Read(1) = msg->pose[i].orientation.y;
            expection_4Obj_4Current_Oreintation_Read(2) = msg->pose[i].orientation.z;
            expection_4Obj_4Current_Oreintation_Read(3) = msg->pose[i].orientation.w;

            //std::cout << "gt_.position.x =="<<std::endl;
            std::cout << expection_4Obj_4Current_Oreintation_Read<<std::endl;

            break;
        }
    }
}




int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_Center"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;
    ros::NodeHandle n_;
    ros::NodeHandle nh;

     ros::Subscriber topic_sub = n.subscribe("/obstacle/lq_new_path", 1000, Topic_Subscrib);
     // ros::Subscriber ground_truth_sub_ = n_.subscribe("gazebo/model_states", 1000, &GTCallback);

    int pp = 0;
    
    double dt;
    dt = 0.02;
    ros::Rate rate(1./dt);

    ofstream Ori_Reali;   
    ofstream Ori_Esti;    

    ofstream Pos_Reali;   
    ofstream Pos_Esti; 

    Ori_Esti.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/OriandPosTxt/OriEst.txt");
    Ori_Reali.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/OriandPosTxt/OriReal.txt");

    Pos_Esti.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/OriandPosTxt/PosEst.txt");
    Pos_Reali.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/OriandPosTxt/PosReal.txt");

  while(pp<=3000)
  {
     
    pp++;



    
   //std::cout << "ROTTOPIC_INIT" << std::endl;
    if (Pos_Esti.is_open()) {
        Pos_Esti << expection_4Obj_4Next_Read(0) <<"\t"<<  expection_4Obj_4Next_Read(1) <<"\t" << expection_4Obj_4Next_Read(2) <<std::endl;
          }
    if (Ori_Esti.is_open()) {
        Ori_Esti << expection_4Obj_4Next_Oreintation_Read(4) <<"\t"<<  expection_4Obj_4Next_Oreintation_Read(5) <<"\t" << expection_4Obj_4Next_Oreintation_Read(6) <<"\t" << expection_4Obj_4Next_Oreintation_Read(7) <<"\t" <<std::endl;
          } 

  

   //std::cout << "ROTTOPIC_INIT" << std::endl;
    //   if (Pos_Reali.is_open()) {
    //    Pos_Reali << expection_4Obj_4Current_Read(0) <<"\t"<<  expection_4Obj_4Current_Read(1) <<"\t" << expection_4Obj_4Current_Read(2) <<std::endl;
     //     }
    //   if (Ori_Reali.is_open()) {
     //   Ori_Reali << expection_4Obj_4Current_Oreintation_Read(0) <<"\t"<<  expection_4Obj_4Current_Oreintation_Read(1) <<"\t" << expection_4Obj_4Next_Oreintation_Read(2) <<"\t"<< expection_4Obj_4Current_Oreintation_Read(3) <<"\t" <<std::endl;
      //    } 


    ros::spinOnce();
    rate.sleep();
    
  }


}