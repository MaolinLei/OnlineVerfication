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



void control_robot_postion_test(auto robot ,auto& model, Eigen::VectorXd desire_position, Eigen::VectorXd desire_velocity, Eigen::VectorXd tau)
{

        //model.update();
        robot->setPositionReference(desire_position);
        robot->setVelocityReference(desire_velocity);       
        robot->setEffortReference(tau);
        robot->move();

}

Eigen::VectorXd expection_4Obj_4Current_Read(3);
Eigen::VectorXd expection_4Obj_4Current_Oreintation_Read(4);

Eigen::VectorXd expection_4Obj_4Next_Read(6);
Eigen::VectorXd expection_4Obj_4Next_Oreintation_Read(8);

void Topic_Subscrib(const nav_msgs::Path& msg)
{  


    int n = floor(msg.poses.size()/2);
    expection_4Obj_4Next_Read(0) = msg.poses[n].pose.position.x + 0.5;
    expection_4Obj_4Next_Read(1) = msg.poses[n].pose.position.y+ 0.2 ;
    expection_4Obj_4Next_Read(2) = msg.poses[n].pose.position.z-0.5;

    expection_4Obj_4Next_Read(3) = msg.poses[msg.poses.size()-1].pose.position.x + 0.5;
    expection_4Obj_4Next_Read(4) = msg.poses[msg.poses.size()-1].pose.position.y+ 0.2 ;
    expection_4Obj_4Next_Read(5) = msg.poses[msg.poses.size()-1].pose.position.z - 0.5;

    expection_4Obj_4Next_Oreintation_Read(0) = msg.poses[n].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(1) = msg.poses[n].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(2) = msg.poses[n].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(3) = msg.poses[n].pose.orientation.w;

    expection_4Obj_4Next_Oreintation_Read(4) = msg.poses[msg.poses.size()-1].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(5) = msg.poses[msg.poses.size()-1].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(6) = msg.poses[msg.poses.size()-1].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(7) = msg.poses[msg.poses.size()-1].pose.orientation.w;


   //std::cout << "ROTTOPIC_INIT" << std::endl;


}


/* void GTCallback(const geometry_msgs::PoseStamped& msg)
{
   //  for (int i=0; i<msg->name.size(); i++)
    {
       //  if (msg->name[i].compare("actor") == 0)
        {

            // expection_4Obj_4Current_Read(0) = msg->pose[i].position.x-0.5;
            // expection_4Obj_4Current_Read(1) = msg->pose[i].position.y;
            // expection_4Obj_4Current_Read(2) = expection_4Obj_4Next_Read(2);
            expection_4Obj_4Current_Oreintation_Read(0)= msg.pose.point.x;
            expection_4Obj_4Current_Oreintation_Read(1)= msg.pose.point.y;
            expection_4Obj_4Current_Oreintation_Read(2)= msg.pose.point.z;

            //->pose[i].orientation.x;
            expection_4Obj_4Current_Oreintation_Read(1) = msg.pose.Quaternion.x;
            expection_4Obj_4Current_Oreintation_Read(1) = msg.pose.Quaternion.y;
            expection_4Obj_4Current_Oreintation_Read(2) = msg.pose.Quaternion.z;
            expection_4Obj_4Current_Oreintation_Read(3) = msg.pose.Quaternion.w;
   
            //std::cout << "gt_.position.y =="<<std::endl;

     //       break;
        }
    }

             std::cout << "gt_.position.x =="<<std::endl;
} */



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
             std::cout << "msg.pose.orientation.x =="<< msg.pose.orientation.x <<std::endl;
}




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


  /* void poseCallback(const xbot_msgs::JointState& msg)
{
Eigen::VectorXf Position_Joint_float(7);
for(int j =8;j<15; j++)
{   
    //std::cout << msg.link_position[j] << std::endl;
    Position_Joint_float(j-8) = msg.link_position[j];
}
Position_Joint = Position_Joint_float.cast<double>();
 // std::cout << Position_Joint << std::endl <<std::endl;

}  
*/

int show = 1;

void Callshadow(const std_msgs::Int8& msg)
{
    
    show = msg.data;
    
            //std::cout << "gt_.position.x =="<<std::endl;
            std::cout << "gt_.position.y ==" << show <<std::endl;

}

    using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_Center"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;
    ros::NodeHandle n_;
    ros::NodeHandle nh;
    ros::NodeHandle nfind;

    ros::Subscriber topic_sub = n.subscribe("/obstacle/lq_new_path", 1000, &Topic_Subscrib);
    ros::Subscriber ground_truth_sub_ = n_.subscribe("/human_pose", 1000, &GTCallback);
    ros::Subscriber sub = nh.subscribe("/xbotcore/joint_states", 1000, &poseCallback);
    ros::Subscriber sub_inshadow = nfind.subscribe("/darknet_ros/found_object", 1000, &Callshadow);

    MPC_OPTIMAL MPC_OPTIMAL_;

    ros::Publisher Cen_joint_cmd_pub = MPC_OPTIMAL_.init_Pub_Control_Node(nh);


    Tree my_tree;

    kdl_parser::treeFromFile("/home/oem/hhcm_ws/src/iit-centauro-ros-pkg/centauro_urdf/urdf/centauro.urdf",my_tree);

    YAML::Node arm_param_config = YAML::LoadFile("/home/oem/hhcm_ws/config/Cen_Robot.yaml");

    double dt;
    dt = 0.02;
    ros::Rate rate(1./dt);

///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> arm_link_names_;
    for (auto iter : arm_param_config["arm_link_names"])
    {
        arm_link_names_.push_back(iter.as<std::string>());
    }
    // get starting position
    Eigen::VectorXd q0(7);
    q0 =Position_Joint;
     

for(int i=0;i<3;i++)
{
expection_4Obj_4Current_Read(i) = 1000;
expection_4Obj_4Next_Read(i) = 1000;
}
for(int i=3;i<6;i++)
{
// expection_4Obj_4Current_Read(i) = 1000;
expection_4Obj_4Next_Read(i) = 1000;
}

expection_4Obj_4Current_Oreintation_Read(0) = 0;
expection_4Obj_4Current_Oreintation_Read(1) = 0;
expection_4Obj_4Current_Oreintation_Read(2) = 0;
expection_4Obj_4Current_Oreintation_Read(3) = 1;


expection_4Obj_4Next_Oreintation_Read(0) = 0;
expection_4Obj_4Next_Oreintation_Read(1) = 0;
expection_4Obj_4Next_Oreintation_Read(2) = 0;
expection_4Obj_4Next_Oreintation_Read(3) = 1;

expection_4Obj_4Next_Oreintation_Read(4) = 0;
expection_4Obj_4Next_Oreintation_Read(5) = 0;
expection_4Obj_4Next_Oreintation_Read(6) = 0;
expection_4Obj_4Next_Oreintation_Read(7) = 1;
/////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<std::string> arm_rotation_joints;
    ArmCollision collision_detection(my_tree, arm_param_config, arm_rotation_joints);



////////////////////////////////////////////////////////////////////

  /*traj Collision;
    Collision.dt = dt;
    Collision._Acc = 0.05;
    Collision._Vel = 0.8;
    Collision._dev_order = 4.0;
    Collision.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    Collision.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    Collision.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    Collision.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true);*/ 

    traj traj;
    traj.dt = dt;
    traj._Acc = 1.0;
    traj._Vel = 2.0;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    traj.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    traj.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true);
    //traj.sphere_visual_Random = n.advertise<visualization_msgs::Marker>("ProperVolume_random",1,true);

    int horizon = 10;

    Chain chain;
    my_tree.getChain("torso_2","ball1_tip",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();

    string name_4Link[nj+1];

    cout<<"nj="<<nj<<endl;
	
    JntArray jointpositions(nj);
    JntArrayVel jointVelocity(nj);
    Eigen::VectorXd jointpositions_vector(nj);
   
    Eigen::VectorXd jointpositions_vector_(nj);
    Eigen::VectorXd jointpositions_velcity_(nj);

    int Wait=0;
    while(Wait <= 5)
    {
        Wait ++ ;
        
       // MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub, q0);
        ros::spinOnce();
        rate.sleep();

    }

    
     // q0 = Position_Joint;

    Wait=0;
    while(Wait <= 100)
    {
        Wait ++ ;
        
        // std::cout << Wait <<std::endl;

        // MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub, q0);
        ros::spinOnce();
        rate.sleep();

    }
       

     //return 0;
    jointpositions_vector_ = Position_Joint;
    jointpositions_vector = Position_Joint;
    //cout << chain.getSegment(0).getName() << endl <<endl;

    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) = jointpositions_vector_(i);
     //jointpositions.data(i) = 3.14/3;

     jointpositions_vector(i)=jointpositions.data(i) ;
    }

    std::cout << jointpositions_vector << std::endl;
    for(unsigned int i=0;i<nj+1;i++)
    {
    name_4Link[i] = arm_link_names_[i];
    //std::cout << "arm_link_names_[]=" << arm_link_names_[i] << std::endl; 
    }

    Frame cartpos;
    ChainJntToJacSolver kdl_solver(chain);
    ChainJntToJacDotSolver kdl_solver_JacDot(chain);
    fksolver.JntToCart(jointpositions,cartpos);
///////////////////////////////////////////////////////////////////////////////////////////////////////////    
    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);

	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}

    
    
    
    
    int pp=0;
    int Number_Joint = nj ;
    Eigen::VectorXd velocity_inLastTIme(Number_Joint );
    velocity_inLastTIme.setZero(); 

    Eigen:: VectorXd Joint_velocity_(nj);
    Joint_velocity_.setZero();

    for(unsigned int i=0;i<nj;i++)
    {
     jointVelocity.q.data(i) = jointpositions.data(i) ;
     jointVelocity.qdot.data(i) = Joint_velocity_(i);
    }

    
    Eigen::VectorXd Position_Error_;
    Eigen::MatrixXd A_4distance_Constraint;
    Eigen::MatrixXd B_4distance_Constraint; 

    double distance;
    Eigen::Vector3d nearest_points[2];

    int number_4Link_total = nj+1;
    int number_4Link = 0;
    int Nearest_Link_ = 0 ;  
    double t;
    Eigen ::MatrixXd Jacobian_Nearest;
    Eigen::Vector3d Position_NearPoint(0.00,0.00,0.00);
    //Jacobian_Nearest = Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], jointpositions_vector.topRows(Nearest_Link_), Position_NearPoint);
     Eigen::VectorXd Desire_position(nj);
     Desire_position.setZero();

//////////////////////////////////////////
    std::vector<Eigen::Vector3d> p_list;
   /* double pos_des[3];
    Eigen::Vector3d p;
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);
    p_list.push_back(p);
    pos_des[0] = cartpos.p(0) - 0.0;
    pos_des[1] = cartpos.p(1) + 0.20 ;
    pos_des[2] = cartpos.p(2) + 0.15 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); */




    double pos_des[3];
    Eigen::Vector3d p;
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p); 
    







    /*pos_des[0] = cartpos.p(0) + 0.1;
    pos_des[0] = cartpos.p(0) + 0.20;
    pos_des[1] = cartpos.p(1) + 0.2 ;
       pos_des[1] = cartpos.p(1) + 0.06 ;
    pos_des[2] = cartpos.p(2) + 0.30 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); */

   /*  pos_des[0] = cartpos.p(0) + 0.1;
    pos_des[0] = cartpos.p(0) + 0.15;
    pos_des[1] = cartpos.p(1) + 0.2 ;
    pos_des[1] = cartpos.p(1) + 0.1 ;
    pos_des[2] = cartpos.p(2) + 0.2 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); */


    /*pos_des[0] = cartpos.p(0) + 0.;
    pos_des[0] = cartpos.p(0) + 0.;
    pos_des[1] = cartpos.p(1) + 0.25 ;
    pos_des[1] = cartpos.p(1) + 0.0 ;
    pos_des[2] = cartpos.p(2) + 0.2 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); */







    pos_des[0] = cartpos.p(0) + 0.;
     pos_des[0] = cartpos.p(0) + 0.0;
    // pos_des[0] = cartpos.p(0) + 0.0;
    pos_des[1] = cartpos.p(1) + 0.0;
    pos_des[2] = cartpos.p(2) + 0.25;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); 


    pos_des[0] = cartpos.p(0) + 0.;
    pos_des[0] = cartpos.p(0) + 0.2;
    // pos_des[0] = cartpos.p(0) + 0.0;
    pos_des[1] = cartpos.p(1) + 0.15 ;
    pos_des[2] = cartpos.p(2) + 0.20;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); 

   
    pos_des[0] = cartpos.p(0) - 0.0;
    pos_des[0] = cartpos.p(0) + 0.01;
    pos_des[1] = cartpos.p(1) + 0.02 ;
    pos_des[1] = cartpos.p(1) + 0.00 ;
    pos_des[2] = cartpos.p(2) - 0.0 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p);

    /* pos_des[0] = cartpos.p(0) + 0.;
    pos_des[0] = cartpos.p(0) + 0.;
    pos_des[1] = cartpos.p(1) + 0.25 ;
    pos_des[1] = cartpos.p(1) + 0.0 ;
    pos_des[2] = cartpos.p(2)  ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p);*/ 

   /* p[0] = cartpos.p(0) + 0.1;
    p[1] = cartpos.p(1)+ 0.20;
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2 )+ 0.25;
    p_list.push_back(p); 
    p[0] = cartpos.p(0) + 0.05;
    p[1] = cartpos.p(1)+ 0.18;
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2 )+ 0.2; */

    /* p_list.push_back(p); 
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);
    p_list.push_back(p);*/ 

    /* pos_des[0] = cartpos.p(0) + 0.1;
    pos_des[1] = cartpos.p(1) + 0.0;
    pos_des[2] = cartpos.p(2) + 0.35 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); */


    /* pos_des[0] = cartpos.p(0) + 0.05;
    pos_des[1] = cartpos.p(1) + 0.25 ;
    pos_des[2] = cartpos.p(2) + 0.2 ;

    

    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];

    p_list.push_back(p); */



    /*pos_des[0] = cartpos.p(0) - 0.2;
    pos_des[1] = cartpos.p(1) + 0.0 ;
    pos_des[2] = cartpos.p(2) + 0.2 ;

    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];

    p_list.push_back(p);*/ 



    /* pos_des[0] = cartpos.p(0) + 0.25;
    pos_des[1] = cartpos.p(1) + 0.25 ;
    pos_des[2] = cartpos.p(2) + 0.25 ;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p);*/ 


    /*pos_des[0] = cartpos.p(0) + 0.2 ;
    pos_des[1] = cartpos.p(1) + 0.0 ;
    pos_des[2] = cartpos.p(2) + 0.15 ;

    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];*/

    //p_list.push_back(p);

    Eigen::Vector3d v;
    v.setZero();

    bool success = traj.trajGeneration(p_list,v);

    /*std::vector<Eigen::Vector3d> p_list_collision;
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1) -0.8;
    p[2] = cartpos.p(2) + 0;
    //p[2] = cartpos.p(2) + 10000;
    p_list_collision.push_back(p);

    p[0] = cartpos.p(0) - 0.3 ;
    p[1] = cartpos.p(1) + 0.85  ;
    p[2] = cartpos.p(2) + 0.500 ;

    //p[2] = cartpos.p(2) + 500 ;
    p_list_collision.push_back(p);


    p[0] = cartpos.p(0) - 0.3 ;
    p[1] = cartpos.p(1) - 1.0  ;
    p[2] = cartpos.p(2) + 0.6000 ;
    //p[2] = cartpos.p(2) + 6000 ;
    p_list_collision.push_back(p);

    p[0] = cartpos.p(0) - 0.3 ;
    p[1] = cartpos.p(1) + 1.2  ;
    p[2] = cartpos.p(2) + 0.800 ;
    //p[2] = cartpos.p(2) + 800 ;
    p_list_collision.push_back(p);

    //Collision.trajGeneration(p_list_collision,v); */


    //Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.707 * M_PI, Eigen::Vector3d::UnitX());   
    Eigen::AngleAxisd rollAngle(-0.05* M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.05 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.05* M_PI, Eigen::Vector3d::UnitX());  

    // Eigen::AngleAxisd rollAngle(0.0* M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.01 * M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.010* M_PI, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen :: MatrixXd Rotation_desire = q.matrix();
  

    Rotation_desire = Rotation_op  * Rotation_desire;
    Eigen::MatrixXd _Rotation_Error_ = (Rotation_op.transpose() * Rotation_desire).log();
    Eigen::MatrixXd Rotation_desire_record = Rotation_desire;
    Eigen::MatrixXd _Rotation_op_init = Rotation_op;


  

    ofstream examplefile_Pos;   
    ofstream examplefile_Posdes;   
    ofstream examplefile_des_EndVeclocity;   

    ofstream End_Orentation;   
    ofstream End_Orentation_desire;   
    examplefile_Pos.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos.txt");
    examplefile_Posdes.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos_des.txt");
    End_Orentation_desire.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Orentation_desire.txt");
    End_Orentation.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Orentation.txt");
    //examplefile_des_EndVeclocity.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_des_EndVeclocity.txt");

    Jacobian jacobian_kdl(nj);
    Jacobian jacobian_Dot_kdl(nj);
    
    Eigen::VectorXd Error_(3);
    Error_.setZero();
   




        Eigen::VectorXd nearest_points_Robot(3);
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "torso_2";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 125245;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
    double t_collision;
    double t_orientation = 0;


     std::cout << "Position_Joint_initiatl = " <<Position_Joint <<std::endl;
     q0 = Position_Joint;
     Desire_position= Position_Joint;

     std::cout << "Desire_position----?" << Desire_position <<std::endl;
     std::cout << "q0----?" << q0 <<std::endl;

    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) =  q0(i);
     
    //  /Desire_position(i) = Position_Joint(i) + dt *velocity_inLastTIme(i);
     
     // jointpositions.data(i) ;
     //Desire_position(i) = q0(i) + dt* velocity_inLastTIme(i);
         // jointpositions.data(i)  = q0(i) ;

    } 
    int Q = 0;
    //while(Q <= 0)
     
    {  
    t = 0;
    pp=0;

    while(pp<=800)
    {


         // std::cout <<"q0 =" << q0 << std::endl << std::endl;

    for(unsigned int i=0;i<nj;i++)
    {
    // jointpositions.data(i) = q0(i) ;

     jointVelocity.q.data(i) = jointpositions.data(i) ;
     jointVelocity.qdot.data(i) = velocity_inLastTIme(i);
    
    }
     //std::cout << "q0 - Desire_position =" <<q0 - Desire_position << std::endl;
    fksolver.JntToCart(jointpositions,cartpos);
    kdl_solver.JntToJac(jointpositions,jacobian_kdl) ;
    kdl_solver_JacDot.JntToJacDot(jointVelocity,jacobian_Dot_kdl);
    

    Eigen:: MatrixXd Jacobian_4Robot_End;
    Eigen:: MatrixXd Jacobian_4Robot_End_4velocity;
    Eigen:: MatrixXd Jacobian_4Robot_End_4angvelocity;
    Eigen:: MatrixXd velocity_Jacobian_end_;
    Eigen:: VectorXd angvelocity_Jacobian_end_;

    Jacobian_4Robot_End = jacobian_kdl.data;
    Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);
    velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;

    Eigen:: MatrixXd JacobianDot_4Robot_End_;
    JacobianDot_4Robot_End_ = jacobian_Dot_kdl.data;
    //std::cout << "JacobianDot_4Robot_End_ = " << JacobianDot_4Robot_End_ << std::endl;

    /*p[0] = 0.1;
    p[1] = 0.1;
    p[2] = 0.4;
    p_list.push_back(p);*/
    
    
    v = velocity_Jacobian_end_;

    if (t_orientation <=1)
    {
    t_orientation = (pp+1)*dt/100;
    }
    
    Rotation_desire =  _Rotation_op_init * (t_orientation * _Rotation_Error_).exp() ;

    Eigen::VectorXd Position_disire_(3*horizon); 
    Eigen::VectorXd Velocity_disire_(3*horizon); 
    Eigen::VectorXd Pos_Vel_disire_(6*horizon); 
    for(int i=0;i<horizon;i++)
    {
    
    //if( == )
    {
    t = pp*dt + dt*i  + 0.0;
    }
    //loop_ = loop_ + 1;
    //tt = loop_ * dt ;
    //Eigen::Vector3d Position_disire = traj.getTrajPos(t);
    //Eigen::Vector3d Velocity_disire = traj.getVel(t);

    //Eigen::Vector3d Position_disire(pos_des[0],pos_des[1],pos_des[2]);

    //Eigen::Vector3d Velocity_disire(0.2,0.2,0.1);
    //cout << "t=" << t <<endl;
    //cout << traj.getTrajPos(t) <<endl << endl;costFunction_VelocityandPos

    Position_disire_.middleRows(i*3,3) = traj.getTrajPos(t);
    Velocity_disire_.middleRows(i*3,3) = traj.getVel(t);
    
    Pos_Vel_disire_.middleRows(i*3,3) = traj.getTrajPos(t) ;
    Pos_Vel_disire_.middleRows(3+i*3,3) = traj.getVel(t) ;   
    //Position_disire_.middleRows(i*3,3) = Position_disire;
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;


	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}


   //std::cout << "<<<<<<<<<<<<<<" <<std::endl;
   //MPC_OPTIMAL_.costFunction_Velocity(nj, horizon, dt ,cartpos, 
   //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 

   /*MPC_OPTIMAL_.ConstructCost_Orientation(nj, horizon, dt , 
                                          Rotation_op , angvelocity_Jacobian_end_, 
                                          Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                                          H_, g_); */ 


    MPC_OPTIMAL_.costFunction_VelocityandPos_Orientation( nj, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_op, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_, t);  
    
    
    
    /*MPC_OPTIMAL_.Quaternion_Cost_Function(nj, horizon, dt , 
                                          Rotation_op , angvelocity_Jacobian_end_, 
                                          Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                                          H_, g_); */
    //std::cout << H_ << std::endl;
    //std::cout << g_ << std::endl;
     
       /* MPC_OPTIMAL_.costFunction_VelocityandPos(nj, horizon, dt ,cartpos,
                           velocity_Jacobian_end_ , Joint_velocity_, 
                           Pos_Vel_disire_, 
                           Jacobian_4Robot_End_4velocity , JacobianDot_4Robot_End_,  
                           H_, g_, t); */






    //std::cout << H_ << std::endl;
    //std::cout << g_ << std::endl;
    //H_ = H_ + 0.0 * H_Orientation;
    //g_ = g_ + 0.0 * g_Orientation;
////////////////////////////////////////////////////////////////////////////////////////////
                    //std::cout << "Jacobian_Nearest =" << Jacobian_Nearest << std::endl;
    Eigen::VectorXd position_joint_current(Number_Joint*horizon);
    //Eigen::VectorXd upper_position_joint(Number_Joint*horizon); 
    //Eigen::VectorXd upper_velocity_joint(Number_Joint*horizon); 
    //Eigen::VectorXd upper_acceletion_joint(Number_Joint*horizon); 

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = jointpositions.data(i);

    }
    }
/////////////////////////////////////////////////////////////////////////////////////////////////////////

    //detect with dynamic obstacle  
    Eigen::VectorXd axis_distance_on_(3);
    Eigen::VectorXd expection_4Obj_Pos_4Next(3);
    Eigen::VectorXd expection_4Obj_Pos(3);
    Eigen::VectorXd expection_4Obj_Vel(3);
    //expection_4Obj_Vel << 0.0,0.0,0.0;
    t_collision = dt * pp + dt;
//    expection_4Obj_Vel = Collision.getVel(t_collision);
    //expection_4Obj_Vel.setZero();
    //expection_4Obj_Pos << 147, -0.0, 0.5;
//    expection_4Obj_Pos = Collision.getTrajPos(t_collision);
    //expection_4Obj_Pos(0) = 1.470;
    //expection_4Obj_Pos(1) = 0;
    //expection_4Obj_Pos(2) = 0.5;

    fcl::Vector3d T;

    //T(0) = expection_4Obj_Pos(0);
    //T(1) = expection_4Obj_Pos(1);
    //T(2) = expection_4Obj_Pos(2);

    T(0) = expection_4Obj_4Next_Read(0) + 0.0;
    T(1) = expection_4Obj_4Next_Read(1) + 000.0;
    T(2) = expection_4Obj_4Next_Read(2); 


        // T(0)  =  10000.0;
        // T(1) =  10000.0;
        // T(2) = 10000;
    std::cout << "expection_4Obj_4Next_Read= " << std::endl << expection_4Obj_4Next_Read << std::endl;
   

if(pp>=10)
{
    T(0) = expection_4Obj_4Next_Read(0) + 0.0;
    T(1) = expection_4Obj_4Next_Read(1) + 000.0;
    T(2) = expection_4Obj_4Next_Read(2);
} 
    //std::cout << expection_4Obj_4Current_Read << std::endl;
    //std::cout << expection_4Obj_4Current_Oreintation_Read << std::endl;

    fcl::Vector3<double> r;
    //r[0] = axis_distance_on_(0);
    //r[1] = axis_distance_on_(1);
    //r[2] = axis_distance_on_(2); 

    //r[0] = 0.10;
    //r[1] = 0.10;
    //r[2] = 0.55; 
    if(show == 1)
    {
    r[0] = 0.195*2;
    r[1] = 0.195*2;
    r[2] = 1.7; 
    }
    else
       {
    r[0] = 0.25*2;
    r[1] = 0.25*2;
    r[2] = 1.700; 
    }

        marker.pose.position.x = T(0) + 0.0;
        marker.pose.position.y = T(1) + 000.0;
        marker.pose.position.z = T(2);
        /* marker.pose.orientation.x = expection_4Obj_4Current_Oreintation_Read[0];
        marker.pose.orientation.y = expection_4Obj_4Current_Oreintation_Read[1];
        marker.pose.orientation.z = expection_4Obj_4Current_Oreintation_Read[2];
        marker.pose.orientation.w = expection_4Obj_4Current_Oreintation_Read[3]; */

        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        marker.scale.x = r[0] * 2;
        marker.scale.y = r[1] * 2;
        marker.scale.z = r[2] * 2;

        marker.color.r = 0.8;
        marker.color.g = 0.0f;
        marker.color.b = 0.3f;
        marker.color.a = 0.9;

        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);



    //collision_detection.joint_states_input_ = Desire_position;
    collision_detection.joint_states_input_ = Position_Joint;
    collision_detection.calFowardKinematics();
    fcl::Matrix3d R_Orientation;
    

    Eigen::Quaternion<double> q_middle;
    q_middle.x() = expection_4Obj_4Next_Read[0];
    q_middle.y() = expection_4Obj_4Next_Read[1];
    q_middle.z() = expection_4Obj_4Next_Read[2];
    q_middle.w() = expection_4Obj_4Next_Read[3];

    //R_Orientation = fcl::Matrix3d::Identity();
    R_Orientation = q_middle.matrix();
    
    collision_detection.calMinCollisionDistance_2DynamicPoint(nearest_points, number_4Link, distance, 
                                                              T, R_Orientation, r, number_4Link_total);
    
    if(pp>=5)
    {   
    if(collision_detection.getCollisionStatus() == 2)
    {
    cout<< pp << " = <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< = " << collision_detection.getCollisionStatus() << endl;
    return 0;
    }
    }

    //std::cout <<"name_4Link[Nearest_Link_]: "<< name_4Link[Nearest_Link_] <<"Position_Error_=" << Position_Error_ << std::endl << "Distance = "<< Position_Error_.norm()<<std:: endl;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint * horizon); 
    Eigen:: VectorXd Vector_Constraint_4acc_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4acc_lower (Number_Joint * horizon);  

    Eigen:: VectorXd upper_velocity_joint(Number_Joint * horizon); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint * horizon); 
    MPC_OPTIMAL_.Constraint_4robot_postion_acceleration_Centaruo(Number_Joint, horizon, dt, position_joint_current, velocity_inLastTIme, 
                                             upper_velocity_joint , lower_velocity_joint,
                                             Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                             Vector_Constraint_4acc_upper, Vector_Constraint_4acc_lower);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    Eigen:: MatrixXd Matrix_Constraint_4pos_ (Number_Joint * horizon, Number_Joint * horizon) ;
    Eigen:: MatrixXd Matrix_Constraint_4acc_(Number_Joint * horizon,Number_Joint * horizon) ;
    MPC_OPTIMAL_.Matrix_Constraint_4acc_pos_acc(Number_Joint, horizon, dt,Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    Eigen:: MatrixXd A_4distance_Constraint_horizon(horizon, Number_Joint*horizon);
    A_4distance_Constraint_horizon.setZero();
    Eigen:: MatrixXd B_4distance_Constraint_horizon(horizon,1);
    B_4distance_Constraint_horizon.setZero();
    Eigen:: MatrixXd B_4distance_Constraint_horizon_upper(horizon,1);   

    for(int i=0; i< horizon;i++)
    {
    B_4distance_Constraint_horizon_upper(i,0) = 1 ;
    }

    for(int j=0; j<horizon; j++)
    {

      double T_4Predict = (j + 1) * dt;
     

      /*MPC_OPTIMAL_.caluculate_Proper(axis_distance_on_, expection_4Obj_Pos_4Next, 
                      expection_4Obj_Pos, expection_4Obj_Vel, T_4Predict);*/


        T(0) = expection_4Obj_Pos_4Next(0);
        T(1) = expection_4Obj_Pos_4Next(1);
        T(2) = expection_4Obj_Pos_4Next(2);

    if(pp<=800)
    {

        if(j<1)
        {
        T(0) = expection_4Obj_4Next_Read(0) ;
        T(1) = expection_4Obj_4Next_Read(1) ;
        T(2) = expection_4Obj_4Next_Read(2);

       // T(0)  =  10000.0;
       // T(1) =  10000.0;
       // T(2) = 10000;



        //T(2) = 100000000;
        q_middle.x() = expection_4Obj_4Next_Oreintation_Read[0];
        q_middle.y() = expection_4Obj_4Next_Oreintation_Read[1];
        q_middle.z() = expection_4Obj_4Next_Oreintation_Read[2];
        q_middle.w() = expection_4Obj_4Next_Oreintation_Read[3];

        R_Orientation = fcl::Matrix3d::Identity();


        }
        else 
        {
        T(0) = expection_4Obj_4Next_Read(3) ; 
        T(1) = expection_4Obj_4Next_Read(4) ;
        T(2) = expection_4Obj_4Next_Read(5);



        //T(2) = 100000000;
        //q_middle.x() = expection_4Obj_4Next_Oreintation_Read[4];
        //q_middle.y() = expection_4Obj_4Next_Oreintation_Read[5];
        //q_middle.z() = expection_4Obj_4Next_Oreintation_Read[6];
        //q_middle.w() = expection_4Obj_4Next_Oreintation_Read[7];

        }  
    }

    else 
    {

        if(j<1)
        {
        T(0) = expection_4Obj_4Next_Read(0);
        T(1) = expection_4Obj_4Next_Read(1) ;
        T(2) = expection_4Obj_4Next_Read(2); 

        //T(0)  =  10000.0;
        //T(1) =  10000.0;
        //T(2) = 10000;
        //T(2) = 100000000;
        q_middle.x() = expection_4Obj_4Next_Oreintation_Read[0];
        q_middle.y() = expection_4Obj_4Next_Oreintation_Read[1];
        q_middle.z() = expection_4Obj_4Next_Oreintation_Read[2];
        q_middle.w() = expection_4Obj_4Next_Oreintation_Read[3];

        R_Orientation = fcl::Matrix3d::Identity();


        }
        else 
        {
        T(0) = expection_4Obj_4Next_Read(3) ; 
        T(1) = expection_4Obj_4Next_Read(4) ;
        T(2) = expection_4Obj_4Next_Read(5);

        // T(0)  =  10000.0;
        // T(1) =  10000.0;
        // T(2) = 10000;
        //T(2) = 100000000;
        //q_middle.x() = expection_4Obj_4Next_Oreintation_Read[4];
        //q_middle.y() = expection_4Obj_4Next_Oreintation_Read[5];
        //q_middle.z() = expection_4Obj_4Next_Oreintation_Read[6];
        //q_middle.w() = expection_4Obj_4Next_Oreintation_Read[7];

        }  


    }
      //R_Orientation = q_middle.matrix();

                   // std::cout << "name_4Link[Nearest_Link_] =" << name_4Link[Nearest_Link_] << std::endl;
                   //  std::cout << "Nearest_Link_=" << Nearest_Link_ << std::endl;
                   // std::cout << "Jacobian_Nearest =" << Jacobian_Nearest << std::endl;

      

      Nearest_Link_ = collision_detection.calMinCollisionDistance_2DynamicPoint(nearest_points, number_4Link, distance, 
                                                                                T, R_Orientation, r, number_4Link_total);

      Jacobian_Nearest = MPC_OPTIMAL_.Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], Desire_position, nearest_points[0], my_tree);
      //nearest_points_Robot = nearest_points[0];
      //std::cout << "distance ==" << distance <<std::endl;
      //std::cout << "Number_Join =" << Nearest_Link_<< std::endl;
      //std::cout << "Position for obstacle" << expection_4Obj_Pos_4Next << std::endl;
               //std::cout << Jacobian_Nearest << std::endl;
      Eigen::VectorXd Position_Error_ = nearest_points[0] - nearest_points[1];
      //std::cout << Position_Error_ <<std::endl;
   

      MPC_OPTIMAL_.Grad_Taylor_Relax(Jacobian_Nearest.topRows(3), Position_Error_ , 
                                     Joint_velocity_.topRows(Nearest_Link_), A_4distance_Constraint, B_4distance_Constraint, T_4Predict);  

      //if(j <=4)
      if(j <=2)
      {   
      // B_4distance_Constraint_horizon(j,0) = (0.025000*(1)-0.00600000*(j)) - B_4distance_Constraint(0,0);
        B_4distance_Constraint_horizon(j,0) = (0.045*(1)-0.0000000*(j)) - B_4distance_Constraint(0,0); 
      }
       else 
      {
      B_4distance_Constraint_horizon(j,0) = -INFTY;
      } 
        for(int i_=0; i_<A_4distance_Constraint.cols(); i_++)
        {

            for(int t=0; t<A_4distance_Constraint.rows(); t++)
            {
               A_4distance_Constraint_horizon(j,i_+j*nj) = A_4distance_Constraint(t,i_);
                
            }
        }
    }



    Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* 2 * horizon + horizon, Number_Joint * horizon);  
    int NumberConstraint = Number_Joint * horizon;

    Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;
    Eigen_4Constraint_A.middleRows(NumberConstraint,horizon) =  A_4distance_Constraint_horizon;
    Eigen_4Constraint_A.bottomRows(NumberConstraint) = Matrix_Constraint_4acc_ ;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon + Number_Joint * horizon + horizon);  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon + Number_Joint * horizon + horizon);  

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 
    //Vector_Constraint_Aconstraint_lower.middleRows(NumberConstraint, NumberConstraint) = Vector_Constraint_4acc_lower;
    //Vector_Constraint_Aconstraint_lower.bottomRows(horizon) =    B_4distance_Constraint_horizon;
    Vector_Constraint_Aconstraint_lower.middleRows(NumberConstraint,horizon) = B_4distance_Constraint_horizon;
    Vector_Constraint_Aconstraint_lower.bottomRows(NumberConstraint) = Vector_Constraint_4acc_lower;

    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;
    Vector_Constraint_Aconstraint_upper.middleRows(NumberConstraint,horizon) = INFTY* B_4distance_Constraint_horizon_upper;
    Vector_Constraint_Aconstraint_upper.bottomRows(NumberConstraint) = Vector_Constraint_4acc_upper;

////////////////////////////////////////
	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());


	qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
  	MPC_OPTIMAL_.matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);



    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;Error_Rotation_op;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_Aconstraint_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_Aconstraint_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	MPC_OPTIMAL_.matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);


//cout << "H_= "<< endl<<H_ << endl;
//cout << "g_matrix= "<< endl<<g_matrix << endl;
/*cout << "Eigen_4Constraint_A= "<< endl<<Eigen_4Constraint_A << endl;
cout << "lower_velocity_joint_matrix= "<< endl<<lower_velocity_joint_matrix << endl;
cout << "upper_velocity_joint_matrix= "<< endl<<upper_velocity_joint_matrix << endl;
cout << "Vector_Constraint_Aconstraint_lower_matrix= "<< endl<<Vector_Constraint_Aconstraint_lower_matrix << endl;
cout << "Vector_Constraint_Aconstraint_upper_matrix= "<< endl<<Vector_Constraint_Aconstraint_upper_matrix << endl; */



	QProblem example( horizon*Number_Joint, horizon*(Number_Joint + Number_Joint) );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 1000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[(Number_Joint+Number_Joint+ Number_Joint)*horizon];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );


	printf( "\nxOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );

    for(int i=0;i<Number_Joint;i++)
    {
    velocity_inLastTIme(i) = xOpt[i];
    }

//std::cout << "velocity_inLastTIme" <<std::endl ;
//std::cout << velocity_inLastTIme <<std::endl;



    for(unsigned int i=0;i<nj;i++)
    {
     //jointpositions.data(i) = jointpositions.data(i)    +dt * velocity_inLastTIme(i);
     
     jointpositions.data(i)  = jointpositions.data(i)  + dt * velocity_inLastTIme(i);
     Desire_position(i) = jointpositions.data(i) ;
     //Desire_position(i) = q0(i) + dt* velocity_inLastTIme(i);
    
    } 



    Joint_velocity_ = velocity_inLastTIme;

    Error_Rotation_op = Rotation_desire.transpose() * Rotation_op ;
    //std::cout << "Different_"<<Rotation_desire.log() - Rotation_op.log() << std::endl;
    std::cout << "Error_Rotation_op.log()" << std::endl << Error_Rotation_op << std::endl;

    Eigen::VectorXd Desire_Rotation_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_desire.log());
    Eigen::VectorXd Rotation_op_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_op.log());

    /*if (examplefile.is_open()) {
        examplefile << cartpos.p(0) <<"\t"<<  cartpos.p(1) <<"\t" << cartpos.p(2) <<std::endl;
          }
    if (examplefile_des.is_open()) {
        examplefile_des << Position_disire_(0) <<"\t"<<  Position_disire_(1) <<"\t" << Position_disire_(2) <<std::endl;
          } */
     
       if (examplefile_Pos.is_open()) {
        examplefile_Pos << cartpos.p(0)  <<"\t"<<  cartpos.p(1) <<"\t" << cartpos.p(2)  <<std::endl;
          }
    if (examplefile_Posdes.is_open()) {
        examplefile_Posdes << Position_disire_(0) <<"\t"<<  Position_disire_(1) <<"\t" << Position_disire_(2) <<std::endl;
    }

    if (End_Orentation_desire.is_open()) {
        End_Orentation_desire << Desire_Rotation_VECTOR(0) <<"\t"<<  Desire_Rotation_VECTOR(1) <<"\t" << Desire_Rotation_VECTOR(2) <<std::endl;
          }
   /* if (End_Orentation.is_open()) {
        End_Orentation << Rotation_op_VECTOR(0) <<"\t"<<  Rotation_op_VECTOR(1) <<"\t" << Rotation_op_VECTOR(2) <<std::endl;
          } */

          if (End_Orentation.is_open()) {
        End_Orentation << distance <<std::endl;
          } 

     for(int i = 0; i<3 ;i++)
     {
     Error_(i) = cartpos.p(i) - Position_disire_(i);
     //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
          cout << "Error_ = " << Error_.norm() << endl;
     }


     //std::cout << "velocity_inLastTIme*dt=" << velocity_inLastTIme*dt <<std::endl ;    
     //std::cout << "q0=" << q0 <<std::endl ;    
     //std::cout << "Jacobian * velocity_inLastTIme *dt =" << Jacobian_4Robot_End_4velocity*velocity_inLastTIme *dt<<std::endl ;    




     pp++;
     cout << pp <<endl;
     //std::cout << "Desire_position ==" << Desire_position <<std::endl;
     //cout << "Desire_position=" << Desire_position <<endl;
     //std::cout << "cartpos ==" << cartpos <<std::endl;

     std::cout << "Desire_position =" << Desire_position <<std::endl <<std::endl;

          MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub,Desire_position);

    ros::spinOnce();

     rate.sleep();

}
    Q++;
    }

return 0; 

}