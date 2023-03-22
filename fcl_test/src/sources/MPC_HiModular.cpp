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

    // kdl_parser::treeFromFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);
    kdl_parser::treeFromFile("/home/oem/ModularBot/urdf/ModularBot.urdf",my_tree);
    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

///////////////////////////////////////////////////////////////////////////////////////////////////////

    // get config object

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

    unsigned int Number_Joint = chain.getNrOfJoints();


    // get starting position
    Eigen::VectorXd q0;
    Eigen::VectorXd tau;
    Eigen::VectorXd acceleration(Number_Joint);

    acceleration.setZero();

    Eigen::VectorXd Object_Position(3);
    //Object_Position.setZero;
    Object_Position(0) = 0.0; 
    Object_Position(1) = -0.87 ;
    Object_Position(2) = 0.49;
    // Object_Position(2) = 0.6;   
   for(int i=0 ;i<=10;i++)
   {        
        rate.sleep();

   }



   std_msgs::Float32 control_gripper;
   ros::Publisher control_gripper_pub = n.advertise<std_msgs::Float32>("/gripper/gripper_A/closure", 1000);



    robot->sense();
    robot->getPositionReference(q0);
    q0(1) = q0(1) - 0.0; 
    std::cout << q0 << std::endl;
/////////////////////////////////////////////////////////////////////////////////////////////////



    MPC_OPTIMAL MPC_OPTIMAL_;
    //MPC_OPTIMAL MPC_OPTIMAL_Orientation;    
////////////////////////////////////////////////////////////////////

    robot->sense();
    robot->getPositionReference(q0);

    traj traj;
    traj.dt = dt;
    traj._Acc = 0.1;
    traj._Vel = 0.5;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
    traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   
    
    ros::Publisher object_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1,true);
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
 
    double dt_orientation = 0.01;
    dt_orientation = 1;
    double t_segment =0;
    traj.dt_orientation = dt_orientation;

    int horizon_first = 4;
    int horizon_second = 5;
    int horizon = horizon_second;

    JntArray jointpositions(Number_Joint);
    JntArrayVel jointVelocity(Number_Joint);
    Eigen::VectorXd jointpositions_vector(Number_Joint);
    jointpositions_vector = q0;
    //cout << chain.getSegment(0).getName() << endl <<endl;

    for(unsigned int i=0;i<Number_Joint;i++)
    {
     jointpositions.data(i) = q0(i);
     //jointpositions.data(i) = 3.14/3;
    }


    Frame cartpos;
    ChainJntToJacSolver kdl_solver(chain);
    ChainJntToJacDotSolver kdl_solver_JacDot(chain);
    fksolver.JntToCart(jointpositions,cartpos);
///////////////////////////////////////////////////////////////////////////////////////////////////////////    
    int pp=0;
    int pp_=1250;

    Eigen::VectorXd velocity_inLastTIme(Number_Joint );
    velocity_inLastTIme.setZero(); 

    Eigen:: VectorXd Joint_velocity_(Number_Joint);
    Joint_velocity_.setZero();

    for(unsigned int i=0;i<Number_Joint;i++)
    {
     jointVelocity.q.data(i) = jointpositions.data(i) ;
     jointVelocity.qdot.data(i) = Joint_velocity_(i);
    }

///////////////////////////////////////////////////////////////
   FCL_TEST::state Error_date;
   ros::Publisher Error_vecotor = n.advertise<FCL_TEST::state>("/Error_state", 1000);

    ofstream examplefile;   
    ofstream examplefile_des;   
    ofstream lastJoint;   

    examplefile.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Orentation.txt");
    examplefile_des.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos.txt");
    lastJoint.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos_desLastJoin.txt");

    Jacobian jacobian_kdl(Number_Joint);
    Jacobian jacobian_Dot_kdl(Number_Joint);

    Eigen :: MatrixXd Rotation_Error;
    Eigen :: MatrixXd Rotation_op(3,3);
    Eigen :: MatrixXd Rotation_op_init(3,3);



    Eigen::VectorXd Error_(3);
    Error_.setZero();
    double t;

    Eigen::VectorXd Place_pose(3);
    Place_pose.setZero();

for(int repeat=1;repeat<=1;repeat++)
{
   Object_Position(0) = Object_Position(0)+  repeat *0.150;
   for(int task=0;task<=3;task++)
   {
    t=0;
    pp = 0;
    std::vector<Eigen::Vector3d> p_list;


    fksolver.JntToCart(jointpositions,cartpos);
    // Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.707 * M_PI, Eigen::Vector3d::UnitX());   

    // Eigen::AngleAxisd rollAngle(0.5 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.5* M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.5 * M_PI, Eigen::Vector3d::UnitX());   

    double roatation_x, roatation_y, roatation_z;
    if(task==0)
    {
    roatation_x = -1* M_PI;
    roatation_y = 0.5* M_PI;
    roatation_z = 0.5 * M_PI;
  
    roatation_x = -1* M_PI;
    // roatation_y = 0.47* M_PI;
    roatation_z = 0.5 * M_PI;
    }

    else{
    roatation_x = -1* M_PI;
    roatation_y = 0.5* M_PI;
    roatation_z = 0.5 * M_PI;
    }

     Eigen::AngleAxisd rollAngle(roatation_z, Eigen::Vector3d::UnitZ());
     Eigen::AngleAxisd yawAngle(roatation_y, Eigen::Vector3d::UnitY());
     Eigen::AngleAxisd pitchAngle(roatation_x, Eigen::Vector3d::UnitX());   



    // Eigen::AngleAxisd rollAngle(0.5 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.5* M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(-1 * M_PI, Eigen::Vector3d::UnitX());   
    
    //Eigen::AngleAxisd rollAngle(0.1 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.1 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;


	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);

	}
    Rotation_op_init = Rotation_op;


    Eigen :: MatrixXd Rotation_desire = q.matrix();
     // Rotation_desire = Rotation_op_init;




    Eigen::Vector3d p;
    Eigen::Vector3d v;
    v.setZero();

    double pos_des[3];
    
    if(task == 0)
    {

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);

    pos_des[0] = Object_Position(0);
    pos_des[1] = Object_Position(1) +  0.05;
    pos_des[2] = Object_Position(2);

    p[0] = pos_des[0];
    p[1] = pos_des[1]  ;
    p[2] = pos_des[2];

    p_list.push_back(p); 

    pp_=1000;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;
    }

    else if(task == 1)
    {

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);

    pos_des[0] = Object_Position(0) ;
    pos_des[1] = Object_Position(1) ;
    pos_des[2] = Object_Position(2);

    p[0] = pos_des[0];
    p[1] = pos_des[1]  ;
    p[2] = pos_des[2];


    p_list.push_back(p);

    pp_=700;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;
    }
    



    else if(task == 2)
    {
   // Rotation_desire= Rotation_op_init;

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);

    p[0] = pos_des[0]+0.25;
    p[1] = pos_des[1]+0.1 ;
    p[2] = pos_des[2]+0.2;

    p_list.push_back(p); 

    p[0] = pos_des[0]+0.35 ;
    p[1] = pos_des[1]+0.1 ;
    
    if(repeat==0)
    {
    p[2] = 0.405;
    }
    else  if(repeat==1)
    {
    p[2] = 0.42;
    }

    p_list.push_back(p); 
    
    pp_=1350;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;
    }


    else if(task == 3)
    {
   // Rotation_desire= Rotation_op_init;

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p); 

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
    p[2] = cartpos.p(2)+0.20;

    p_list.push_back(p); 
    
    pp_=600;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;
    }

    Eigen::VectorXd Online_Point((pp_+1)*3);
    Online_Point.setZero();
     /* traj.init_Orientation_Traj = Rotation_op_init;
     traj.Rotation_Error_Traj = Rotation_Error; */

    bool success = traj.trajGeneration(p_list,v);
    


     Eigen::MatrixXd Error_Rotation_op(3,3);
     Eigen::VectorXd Unhated_Oreintation;

     // std::cout << Rotation_Error.transpose() << std::endl;

    t_segment = 0;
    
while(pp<=pp_)
    {
    Eigen::Matrix3d Rotation33_grasp;
   
   if(task<=1)
   {
    visualization_msgs::Marker marker;
     
     marker.header.frame_id = "base_link";
     marker.header.stamp = ros::Time::now();
 
     marker.ns = "basic_shapes";
     marker.id = 0;
     
     marker.type = 3;
 
     marker.action = visualization_msgs::Marker::ADD;
 
     marker.pose.position.x = Object_Position(0);
     marker.pose.position.y = Object_Position(1);
     marker.pose.position.z = Object_Position(2);

     marker.pose.orientation.x = 0.0;
     marker.pose.orientation.y = 0.0;
     marker.pose.orientation.z = 0.0;
     marker.pose.orientation.w = 1.0;
     
     marker.scale.x = 0.025;
     marker.scale.y = 0.025;
     marker.scale.z = 0.045;
     
     marker.color.r = 0.7f;
     marker.color.g = 1.0f;
     marker.color.b = 0.5f;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();

     Rotation33_grasp = Rotation_op.inverse();
     
     object_pub.publish(marker);
   }
      
      else if(task==2) 
   {
    visualization_msgs::Marker marker;

    Eigen::Quaterniond currenet_orinetation; 
    Eigen::Matrix3d Rotation33;
    Rotation33 = Rotation_op * Rotation33_grasp;
    currenet_orinetation= Rotation33;

     marker.header.frame_id = "base_link";
     marker.header.stamp = ros::Time::now();
 
     marker.ns = "basic_shapes";
     marker.id = 0;
     
     marker.type = 3;
 
     marker.action = visualization_msgs::Marker::ADD;
 
     marker.pose.position.x = cartpos.p(0);;
     marker.pose.position.y = cartpos.p(1);
     marker.pose.position.z = cartpos.p(2)-0.0;

     Place_pose(0) = cartpos.p(0);
     Place_pose(1) = cartpos.p(1);
     Place_pose(2) = cartpos.p(2);

     marker.pose.orientation.x = currenet_orinetation.x();
     marker.pose.orientation.y = currenet_orinetation.y();
     marker.pose.orientation.z = currenet_orinetation.z();
     marker.pose.orientation.w = currenet_orinetation.w();
     
     marker.scale.x = 0.025;
     marker.scale.y = 0.025;
     marker.scale.z = 0.045;
     
     marker.color.r = 0.7f;
     marker.color.g = 1.0f;
     marker.color.b = 0.5f;
     marker.color.a = 0.50;
 
     marker.lifetime = ros::Duration();
     
     object_pub.publish(marker);
   }
    
    else if(task==3) 
   {
    visualization_msgs::Marker marker;

    Eigen::Quaterniond currenet_orinetation; 
    Eigen::Matrix3d Rotation33;
    Rotation33 = Rotation_op * Rotation33_grasp;
    currenet_orinetation= Rotation33;

     marker.header.frame_id = "base_link";
     marker.header.stamp = ros::Time::now();
 
     marker.ns = "basic_shapes";
     marker.id = 0;
     
     marker.type = 3;
 
     marker.action = visualization_msgs::Marker::ADD;
 
     marker.pose.position.x = Place_pose(0);
     marker.pose.position.y = Place_pose(1);
     marker.pose.position.z = Place_pose(2);




     marker.pose.orientation.x = currenet_orinetation.x();
     marker.pose.orientation.y = currenet_orinetation.y();
     marker.pose.orientation.z = currenet_orinetation.z();
     marker.pose.orientation.w = currenet_orinetation.w();
     
     marker.scale.x = 0.025;
     marker.scale.y = 0.025;
     marker.scale.z = 0.045;
     
     marker.color.r = 0.7f;
     marker.color.g = 1.0f;
     marker.color.b = 0.5f;
     marker.color.a = 0.50;
 
     marker.lifetime = ros::Duration();
     
     object_pub.publish(marker);
   }

     else if(task==3) 
   {
    visualization_msgs::Marker marker;

    Eigen::Quaterniond currenet_orinetation; 
    Eigen::Matrix3d Rotation33;
    Rotation33 = Rotation_op * Rotation33_grasp;
    currenet_orinetation= Rotation33;

     marker.header.frame_id = "base_link";
     marker.header.stamp = ros::Time::now();
 
     marker.ns = "basic_shapes";
     marker.id = 0;
     
     marker.type = 3;
 
     marker.action = visualization_msgs::Marker::ADD;
 
     marker.pose.position.x = Place_pose(0);
     marker.pose.position.y = Place_pose(1);
     marker.pose.position.z = Place_pose(2);

     marker.pose.orientation.x = currenet_orinetation.x();
     marker.pose.orientation.y = currenet_orinetation.y();
     marker.pose.orientation.z = currenet_orinetation.z();
     marker.pose.orientation.w = currenet_orinetation.w();
     
     marker.scale.x = 0.025;
     marker.scale.y = 0.025;
     marker.scale.z = 0.045;
     
     marker.color.r = 1.0f;
     marker.color.g = 1.0f;
     marker.color.b = 0.4f;
     marker.color.a = 1.0;
 
     marker.lifetime = ros::Duration();
     
     object_pub.publish(marker);
   }
    
    //std::cout << "t_segment =" <<t_segment << std::endl;
    // robot->sense();
    // robot->getJointPosition(q0);
    
    
    for(unsigned int i=0;i<Number_Joint;i++)
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

    /*p[0] = 0.1;
    p[1] = 0.1;
    p[2] = 0.4;
    p_list.push_back(p);*/
    
   

    Eigen::VectorXd Position_disire_(3*horizon); 
    Eigen::VectorXd Velocity_disire_(3*horizon); 
    Eigen::VectorXd Pos_Vel_disire_(6*horizon); 
    Pos_Vel_disire_.setZero();

    for(int i=0;i<horizon;i++)
    {
    
    //if( == )
    {
    t = pp*dt + dt*i  + dt;
    }


    Position_disire_.middleRows(i*3,3) = traj.getTrajPos(t);
    Velocity_disire_.middleRows(i*3,3) = traj.getVel(t);
    
    Pos_Vel_disire_.middleRows(i*6,3) = traj.getTrajPos(t) ;
    Pos_Vel_disire_.middleRows(3+i*6,3) = traj.getVel(t) ;  
    //Position_disire_.middleRows(i*3,3) = Position_disire;
    } 


    // Eigen::MatrixXd Rotation_op(3,3);

  
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}



     Eigen::VectorXd result_jointVel(Number_Joint);
     Eigen::VectorXd _velcity_FirstHierachy(Number_Joint);
     Eigen::MatrixXd _Matrix_FirstHierachy;
     
     result_jointVel.setZero();
     _velcity_FirstHierachy.setZero();



     //Unhated_Oreintation = MPC_OPTIMAL_.UnHatt_matrix(Error_Rotation_op.log());
     Eigen::VectorXd Slack_Vector(6);
     Slack_Vector.setZero();
     //std::cout << Slack_Vector <<std::endl;

   if(task==0)
   {

   // MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);

     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;


     //t_segment = 1;
            if(pp>=400)
     
     {
   
      t_segment = t_segment + dt_orientation;
     

     }

      MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
     /* if(pp>=400)
     
     {
     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 1;
     // t_segment = 1;
     }

    */

     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 

    result_jointVel =  _velcity_FirstHierachy;
     
      if(pp>=400)
     
     {
   

    
      for(int i=0;i<6;i++)
      {
        Slack_Vector(i) = 1 -Slack_Vector(i);   
      }

     Slack_Vector(5) = 0;
       MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector,
                              result_jointVel, 0);  
     
    }

 

     }



     else if(task==1)
     {
     
     t_segment = 1;

     MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);

     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 0;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0;
    // std::cout << Slack_Vector <<std::endl;

     // t_segment = 1;
    /* if(Error_(0)>=Error_(1) && Error_(0)>=Error_(2) && pp/20 ==0)
     {
     Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 0;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0;

     }

     else if(Error_(1)>=Error_(0) && Error_(1)>=Error_(2) && pp/20 ==0)
     {
     Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 0;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0;

     }

     else if(Error_(2)>=Error_(0) && Error_(2)>=Error_(1) && pp/20 ==0)
     {
     Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 0;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0;

     }
    */
   


     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 
    
    result_jointVel =  _velcity_FirstHierachy;
   
    for(int i=0;i<6;i++)
    {
     Slack_Vector(i) = 1 -Slack_Vector(i);        
    } 
     Slack_Vector(5) = 0; 

   /* Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 1;
   */


      MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector,
                                result_jointVel, 0);  
    
    // t_segment = 0;
  
     }
     
     else if(task==2)
   {
    

    
    t_segment = 1;
    //MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);
    
    // t_segment = t_segment + 0.001;
    MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
    
    // Rotation_desire = Rotation_op_init;
    // if(pp>=0)

     /*
     Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 0;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0;
     */

      Slack_Vector(0) = 0;
      Slack_Vector(1) = 0;
      Slack_Vector(2) = 0;
      Slack_Vector(3) = 1;
      Slack_Vector(4) = 1;
      Slack_Vector(5) = 0; 



    
    // std::cout << Rotation_desire <<std::endl;

     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 
    
    result_jointVel =  _velcity_FirstHierachy;

    Eigen::VectorXd Slack_Vector_(6);
    for(int i=0;i<6;i++)
    {
     Slack_Vector_(i) = 1 -Slack_Vector(i);        
    }
     Slack_Vector_(5) = 0;
    

     
    MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector_,
                                result_jointVel, 0);  

                              
     }
    
    

     else if(task==3)
   {
    

    
    t_segment = 1;
    //MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);
    
    // t_segment = t_segment + 0.001;
    MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
    
   
     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;


   /*  Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 0; 
     */
    // std::cout << Rotation_desire <<std::endl;

     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 
    
    result_jointVel =  _velcity_FirstHierachy;

    Eigen::VectorXd Slack_Vector_(6);
    for(int i=0;i<6;i++)
    {
     Slack_Vector_(i) = 1 -Slack_Vector(i);        
    }
     Slack_Vector_(5) = 0;
    

     
    MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector_,
                                result_jointVel, 0);  
  
                               
     }
    

    for(int i=0;i<Number_Joint;i++)
    {
    acceleration(i) = result_jointVel(i)/dt - velocity_inLastTIme(i)/dt;
    Joint_velocity_(i) = result_jointVel[i];
    }
    
     
    // robot->sense();
    // robot->getJointPosition(q0);

    for(unsigned int i=0;i<Number_Joint;i++)
    {
      jointpositions.data(i) = jointpositions.data(i)  + dt * Joint_velocity_(i);
     // jointpositions.data(i) = q0(i)  + dt * Joint_velocity_(i);  
     jointpositions_vector(i) = jointpositions.data(i) ;

     velocity_inLastTIme = Joint_velocity_;
    } 

   
     /* if(task==3 && repeat==0)
      {
     if (lastJoint.is_open()) {
        lastJoint << jointpositions_vector(5) <<std::endl;
       }
        
      }*/

    // std::cout << "Rotation_desire.transpose() " << Rotation_desire  <<std::endl;
    // std::cout << "Rotation_desire.transpose() " << Rotation_desire.transpose()  <<std::endl;
    Error_Rotation_op = Rotation_desire.transpose() * Rotation_op ;
    //std::cout << "Different_"<<Rotation_desire.log() - Rotation_op.log() << std::endl;
    std::cout << "Error_Rotation_op.log()" << std::endl << Error_Rotation_op.log() << std::endl;

    Eigen::VectorXd Desire_Rotation_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_desire.log());
    Eigen::VectorXd Rotation_op_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_op.log());
    Eigen::VectorXd Rotation_Error_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Error_Rotation_op.log());

    // std::cout << "Rotation_desire.transpose() " << Desire_Rotation_VECTOR  <<std::endl;
    // std::cout << "Rotation_desire.transpose() " << Rotation_op_VECTOR  <<std::endl;


     for(int i = 0; i<3 ;i++)
     {
     Error_(i) = cartpos.p(i) - Position_disire_(i);
     Online_Point(pp*3+i)=cartpos.p(i);
     //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
     // cout << "Error_ = " << Error_.norm() << endl;
     }
    

     
 
      Eigen::VectorXd end_velocity_robot_publish;
      end_velocity_robot_publish = Jacobian_4Robot_End * velocity_inLastTIme;

      Error_date.x = end_velocity_robot_publish(0);
      Error_date.y = end_velocity_robot_publish(1);
      Error_date.z = end_velocity_robot_publish(2);  
      Error_date.wx = end_velocity_robot_publish(3);
      Error_date.wy = end_velocity_robot_publish(4);
      Error_date.wz = end_velocity_robot_publish(5);  
      Error_vecotor.publish(Error_date);


   if (examplefile.is_open()) {
        examplefile << end_velocity_robot_publish(0) <<"\t"<<  end_velocity_robot_publish(1) <<"\t" << end_velocity_robot_publish(2) <<std::endl;
          }

    if (examplefile_des.is_open()) {
        examplefile_des << end_velocity_robot_publish(3) <<"\t"<<  end_velocity_robot_publish(4) <<"\t" << end_velocity_robot_publish(5) <<std::endl;
    }



      traj.visTrajectory_online(pp);

     std::cout << "Error_ = " << Error_ << std::endl;

     traj.Online_Point_ = Online_Point;
      

    if(pp>=450 && task == 1)
    {
       control_gripper.data = 13.0;
       control_gripper_pub.publish(control_gripper);
    }

       if(pp>=1200 && task == 2)
    {
       control_gripper.data = 30.0;
       control_gripper_pub.publish(control_gripper);
    }  
     pp++;

        model.setJointPosition(jointpositions_vector);
        model.setJointVelocity(velocity_inLastTIme);
        model.setJointAcceleration(acceleration);
        model.update();
        model.computeInverseDynamics(tau);

        control_robot_postion_test(robot ,model, jointpositions_vector, velocity_inLastTIme, tau);
    

    ros::spinOnce();

     rate.sleep();
    }

}

 
}    
return 0; 


}
