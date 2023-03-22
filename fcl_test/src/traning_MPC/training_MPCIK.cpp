
#include <MPC_Optimal.h>

#include <admittance_controller.h>

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


void MPC_function_define(string Load_path_, Eigen::Vector3d Desired_pos)
{
   

}

void MPC_function(string Load_path_, Eigen::Vector3d Desired_pos)
{
   

}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "Training_MPC"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle nh_("xbotcore");

    ros::NodeHandle n;
    // ros::NodeHandle n_;
    


    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

///////////////////////////////////////////////////////////////////////////////////////////////////////

    // get config object
    Tree my_tree;

    // kdl_parser::treeFromFile("/home/oem/catkin_ws/src/ModularBot_5DOF_B/urdf/ModularBot.urdf",my_tree);
     kdl_parser::treeFromFile("/home/mlei/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);
    //kdl_parser::treeFromFile("/home/oem/catkin_ws/src/ModularBot_5DOF_B/urdf/ModularBot.urdf",my_tree);


    bool exit_value;
    Chain chain;
    my_tree.getChain("base_link","TCP_gripper_A",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int Number_Joint = chain.getNrOfJoints();



   
   FCL_TEST::state Error_date;
   ros::Publisher Error_vecotor = n.advertise<FCL_TEST::state>("/Error_state", 1000);



/////////////////////////////////////////////////////////////////////////////////////////////////



    //MPC_OPTIMAL MPC_OPTIMAL_Orientation;    
////////////////////////////////////////////////////////////////////


    JntArray jointpositions(Number_Joint);
    JntArrayVel jointVelocity(Number_Joint);
    Eigen::VectorXd jointpositions_vector(Number_Joint);
    jointpositions_vector.setZero();
    //cout << chain.getSegment(0).getName() << endl <<endl;

    for(unsigned int i=0;i<Number_Joint;i++)
    {
     jointpositions.data(i) = 0;
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


    Jacobian jacobian_kdl(Number_Joint);
    Jacobian jacobian_Dot_kdl(Number_Joint);

    Eigen::VectorXd Error_(3);
    Error_.setZero();
    double t;



    t=0;
    pp = 0;
    std::vector<Eigen::Vector3d> p_list;


    fksolver.JntToCart(jointpositions,cartpos);


    Eigen::MatrixXd Rotation_op(3,3) ;
    for(int i=0; i<3; i++) {
      for(int j=0; j<3; j++)
        Rotation_op(i,j) = cartpos.M(i,j);

    }
    Eigen::MatrixXd Rotation_op_init = Rotation_op;

  

     Eigen :: MatrixXd Rotation_desire = Rotation_op;

    traj traj;
    traj.represent_base = "base_link";
    traj.dt = dt;
    traj._Acc = 0.1;
    traj._Vel = 0.5;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
    traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   

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

    
    Eigen::VectorXd Online_Point((pp_+1)*3);
    Online_Point.setZero();
     /* traj.init_Orientation_Traj = Rotation_op_init;
     traj.Rotation_Error_Traj = Rotation_Error; */


 


     Eigen::MatrixXd Error_Rotation_op(3,3);
     Eigen::VectorXd Unhated_Oreintation;

     // std::cout << Rotation_Error.transpose() << std::endl;


    int horizon_first = 1;
    Eigen::VectorXd Slack_Vector(6);
    Slack_Vector.setZero();
    
    MPC_OPTIMAL MPC_OPTIMAL_;
while(pp<=pp_)
{
    
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
    
   

    Eigen::VectorXd Position_disire_(3*horizon_first); 
    Eigen::VectorXd Velocity_disire_(3*horizon_first); 
    Eigen::VectorXd Pos_Vel_disire_(6*horizon_first); 
    Pos_Vel_disire_.setZero();

    for(int i=0;i<horizon_first;i++)
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

    // return 0;

     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 

    result_jointVel =  _velcity_FirstHierachy;

   
    // robot->sense();
    // robot->getJointPosition(q0);

    for(unsigned int i=0;i<Number_Joint;i++)
    {
      jointpositions.data(i) = jointpositions.data(i)  + dt * result_jointVel(i);
     // jointpositions.data(i) = q0(i)  + dt * Joint_velocity_(i);  
     jointpositions_vector(i) = jointpositions.data(i) ;

    } 




     for(int i = 0; i<3 ;i++)
     {
     Error_(i) = cartpos.p(i) - Position_disire_(i);
     Online_Point(pp*3+i)=cartpos.p(i);
     //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
     // cout << "Error_ = " << Error_.norm() << endl;
     }
    
  


     Error_vecotor.publish(Error_date);


     traj.visTrajectory_online(pp);

     std::cout << "Error_ = " << Error_ << std::endl;

     traj.Online_Point_ = Online_Point;
      
  
     pp++;


    

    ros::spinOnce();

     rate.sleep();
    }


 

return 0; 


}
