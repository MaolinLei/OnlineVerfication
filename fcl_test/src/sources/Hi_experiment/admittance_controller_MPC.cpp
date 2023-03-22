#include <MPC_Optimal.h>

#include <admittance_controller.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Float32.h>
#include "FCL_TEST/state.h"


#include "admittance_controller.h"

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

    // kdl_parser::treeFromFile("/home/oem/catkin_ws/src/ModularBot_5DOF_B/urdf/ModularBot.urdf",my_tree);
     kdl_parser::treeFromFile("/home/mlei/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);
    //kdl_parser::treeFromFile("/home/oem/catkin_ws/src/ModularBot_5DOF_B/urdf/ModularBot.urdf",my_tree);
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
    Object_Position(0) = 0.7; 
    Object_Position(1) = 0.06 ;
    Object_Position(2) = 0.3;


    Object_Position(0) = 0.7-0.2; 
    Object_Position(1) = 0.06 ;
    Object_Position(2) = 0.3;


   std_msgs::Float32 control_gripper;
   ros::Publisher control_gripper_pub = n.advertise<std_msgs::Float32>("/gripper/gripper_A/closure", 1000);
   
   FCL_TEST::state Error_date;
   ros::Publisher Error_vecotor = n.advertise<FCL_TEST::state>("/Error_state", 1000);


    robot->sense();
    robot->getPositionReference(q0);
    q0(1) = q0(1) - 0.0; 
    std::cout << q0 << std::endl;
/////////////////////////////////////////////////////////////////////////////////////////////////


    admittance_controller admittance_;
    MPC_OPTIMAL MPC_OPTIMAL_;
    //MPC_OPTIMAL MPC_OPTIMAL_Orientation;    
////////////////////////////////////////////////////////////////////

    robot->sense();
    robot->getPositionReference(q0);

    traj traj;
    traj.represent_base = "base_link";
    traj.dt = dt;
    traj._Acc = 0.1;
    traj._Vel = 0.5;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
    traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   
    
    ros::Publisher object_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1,true);
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
 
    double dt_orientation = 0.002;
    // dt_orientation = 1;
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
    int pp_=800;

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


    ofstream examplefile;   
    ofstream examplefile_des;   
    ofstream HierachySeq;   
    ofstream lastJoint;   

   // examplefile.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Orentation.txt");
   // examplefile_des.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos.txt");
    examplefile.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/IK_compar_/IK_compar_MPC/IKcompare_HighPos/JointVeclotiy.txt");
    examplefile_des.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/IK_compar_/IK_compar_MPC/IKcompare_HighPos/JointPosition.txt");
    // HierachySeq.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/HierachySeq.txt");
    // lastJoint.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos_desLastJoin.txt");

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

for(int repeat=0;repeat<=0;repeat++)
{
   Object_Position(0) = Object_Position(0)+  repeat *0.150;
   // for(int task=0;task<=3;task++)
   int task=0;
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


    // Eigen::AngleAxisd rollAngle(0.5 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.5* M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(-1 * M_PI, Eigen::Vector3d::UnitX());   
    
    double roatation_x, roatation_y, roatation_z;

  
    roatation_x = -0.0 * M_PI;
    roatation_y =  0.0 * M_PI;
    roatation_z = -0.5 * M_PI;




     Eigen::AngleAxisd rollAngle(roatation_z, Eigen::Vector3d::UnitZ());
     Eigen::AngleAxisd yawAngle(roatation_y, Eigen::Vector3d::UnitY());
     Eigen::AngleAxisd pitchAngle(roatation_x, Eigen::Vector3d::UnitX());   

     // Eigen::AngleAxisd rollAngle(-0.5 * M_PI, Eigen::Vector3d::UnitZ());
     // Eigen::AngleAxisd yawAngle(0.0* M_PI, Eigen::Vector3d::UnitY());
     // Eigen::AngleAxisd pitchAngle(-1.0 * M_PI, Eigen::Vector3d::UnitX());   

    //Eigen::AngleAxisd rollAngle(0.1 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.1 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;


	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);

	}
    Rotation_op_init = Rotation_op;

    Eigen::Matrix3d Rotation_desire_3d;
    Rotation_desire_3d = q.matrix();
    // yawAngle * pitchAngle * rollAngle;
    // Eigen::MatrixXd Rotation_desire =   Rotation_desire_3d;
    // Eigen :: MatrixXd Rotation_desire = q.matrix();
     // Rotation_desire = Rotation_op_init;

     Eigen :: MatrixXd Rotation_desire = Rotation_op;


    Eigen::Vector3d p;
    Eigen::Vector3d v;
    v.setZero();

    double pos_des[3];


    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);

    p[0] = cartpos.p(0)+0.4;
    p[1] = cartpos.p(1) ;
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);
    p_list.push_back(p); 
   
    bool success = traj.trajGeneration(p_list,v);
    
    pp_=2000;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;
    
    Eigen::VectorXd Online_Point((pp_+1)*3);
    Online_Point.setZero();
     /* traj.init_Orientation_Traj = Rotation_op_init;
     traj.Rotation_Error_Traj = Rotation_Error; */


 


     Eigen::MatrixXd Error_Rotation_op(3,3);
     Eigen::VectorXd Unhated_Oreintation;

     // std::cout << Rotation_Error.transpose() << std::endl;

    t_segment = 0;
    
    Eigen::VectorXd Slack_Vector(6);
    Slack_Vector.setZero();
while(pp<=pp_)
    {
    Eigen::Matrix3d Rotation33_grasp;
    
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

    Eigen::Vector3d Position_disire_middle; 
    Eigen::Vector3d Velocity_disire_middle; 
    Eigen::Vector3d Accelera_disire_middle; 

    Eigen::VectorXd Position_current(3); 
    Position_disire_middle.setZero();
    Velocity_disire_middle.setZero();
    Accelera_disire_middle.setZero();

    Position_current(0) = cartpos.p(0);
    Position_current(1) = cartpos.p(1);
    Position_current(2) = cartpos.p(2);


    Position_disire_middle = traj.getTrajPos(t);
    Velocity_disire_middle = traj.getVel(t);
    
   // admittance_.admittance_feedback_init();
   // admittance_.admittance_feedback_init_current_state( velocity_Jacobian_end_.setZero(), velocity_Jacobian_end_, velocity_Jacobian_end_, Position_current);
    //std::cout << "Robot" << endl;
   // Eigen::Vector3d Position_disire_middle_admittance = admittance_.admittance_feedback_Position(Velocity_disire_middle.setZero(),Velocity_disire_middle.setZero(),Velocity_disire_middle, Position_disire_middle);

    
    // Position_disire_.middleRows(i*3,3) = traj.getTrajPos(t);
    Position_disire_.middleRows(i*3,3) = Position_disire_middle;
    Velocity_disire_.middleRows(i*3,3) = traj.getVel(t);

    Pos_Vel_disire_.middleRows(i*6,3) = Position_disire_middle;  
    // Pos_Vel_disire_.middleRows(i*6,3) = traj.getTrajPos(t) ;
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



   // MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);

     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;

     Slack_Vector(0) = 0;
     Slack_Vector(1) = 0;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 1;
     Slack_Vector(4) = 1;
     Slack_Vector(5) = 1;
  

     //t_segment = 1;
     //       if(pp>=450)
     
     {
   
      t_segment = t_segment + dt_orientation;
      
     }
    //  t_segment = 1;
      MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
   /*  if(pp>=450)
     
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
     
    // return 0;

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

     // Slack_Vector(5) = 0;
       MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector,
                              result_jointVel, 0);  
    






    

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

      if(task==2 && repeat==0)
      {
    //  if (lastJoint.is_open()) {
    //     lastJoint << jointpositions_vector(4) <<std::endl;
    //    }
        
      }

     // jointpositions_vector(1) = jointpositions_vector(1) - 0.05;

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
    
    
     
    if (examplefile.is_open()) {
        examplefile << Joint_velocity_(0) <<"\t"<<  Joint_velocity_(1) <<"\t" << Joint_velocity_(2) <<"\t" << Joint_velocity_(3) <<"\t" << Joint_velocity_(4)<<std::endl;
          }

    if (examplefile_des.is_open()) {
        examplefile_des << jointpositions_vector(0) <<"\t"<<  jointpositions_vector(1) <<"\t" << jointpositions_vector(2) <<"\t" << jointpositions_vector(3) <<"\t" << jointpositions_vector(4)<<std::endl;
       }


   Error_(0) = fabs(Error_(0));
   Error_(1) = fabs(Error_(1));
   Error_(2) = fabs(Error_(2));
   Rotation_Error_VECTOR(0) = fabs(Rotation_Error_VECTOR(0));
   Rotation_Error_VECTOR(1) = fabs(Rotation_Error_VECTOR(1));
   Rotation_Error_VECTOR(2) = fabs(Rotation_Error_VECTOR(2));
       
      Error_date.x = Error_(0);
      Error_date.y = Error_(1);
      Error_date.z = Error_(2);  
      Error_date.wx = Rotation_Error_VECTOR(0);
      Error_date.wy = Rotation_Error_VECTOR(1);
      Error_date.wz = Rotation_Error_VECTOR(2);  


      Eigen::VectorXd end_velocity_robot_publish;
      end_velocity_robot_publish = Jacobian_4Robot_End * velocity_inLastTIme;

      Error_date.x = end_velocity_robot_publish(0);
      Error_date.y = end_velocity_robot_publish(1);
      Error_date.z = end_velocity_robot_publish(2);  
      Error_date.wx = end_velocity_robot_publish(3);
      Error_date.wy = end_velocity_robot_publish(4);
      Error_date.wz = end_velocity_robot_publish(5);  
      Error_vecotor.publish(Error_date);

      Error_vecotor.publish(Error_date);


      traj.visTrajectory_online(pp);

     std::cout << "Error_ = " << Error_ << std::endl;

     traj.Online_Point_ = Online_Point;
      

    if(pp>=450 && task == 1)
    {
       control_gripper.data = 13.0;
       control_gripper_pub.publish(control_gripper);
    }

       if(pp>=1000 && task == 2)
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
