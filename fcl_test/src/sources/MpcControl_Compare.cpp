#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
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

   kdl_parser::treeFromFile("/home/oem/ModularBot/urdf/ModularBot.urdf",my_tree);
    // kdl_parser::treeFromFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);
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

    robot->sense();
    robot->getPositionReference(q0);
    q0(1) = q0(1) - 0.0; 
    



   for(int i=0 ;i<=100;i++)
   {

   
      /* Eigen::VectorXd jointpositions_vector_(Number_Joint);
       Eigen::VectorXd jointpositions_velcity_(Number_Joint);
       acceleration.setZero();
       for(int j=0;j<Number_Joint;j++)
       {
          jointpositions_vector_(j) = q0(j) + 0.00;   
          jointpositions_velcity_(j) = 0.000;      
       }

       jointpositions_vector_(1) = q0(1);
        model.setJointPosition(jointpositions_vector_);
        model.setJointVelocity(jointpositions_velcity_);
        model.setJointAcceleration(acceleration);
        model.update();
        model.computeInverseDynamics(tau);

        // set reference to robot
        robot->setPositionReference(jointpositions_vector_);
        robot->setVelocityReference(jointpositions_velcity_);
        robot->setEffortReference(tau);

        // move() does the publishing to /xbotcore/command
        robot->move();*/ 
         
        rate.sleep();

   }




/////////////////////////////////////////////////////////////////////////////////////////////////



    MPC_OPTIMAL MPC_OPTIMAL_;
    //MPC_OPTIMAL MPC_OPTIMAL_Orientation;    
////////////////////////////////////////////////////////////////////

    robot->sense();
    //robot->getJointPosition(q0);
    robot->getPositionReference(q0);
     std::cout << q0 << std::endl;

    traj traj;
    traj.dt = dt;
    traj._Acc = 0.5;
    traj._Vel = 2.0;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
    traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   
    
    ros::Publisher object_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1,true);
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
 
    double dt_orientation = 0.01;
    double t_segment =0;
    traj.dt_orientation = dt_orientation;

    int horizon_first = 5;
    int horizon_second = 6;
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

     Eigen::MatrixXd Jacobian_robot;
    model.getJacobian("TCP_gripper_A",Jacobian_robot);
   
    Eigen::Affine3d pose;

    model.getPose("TCP_gripper_A", pose);
    Eigen::Quaterniond q1111;
    q1111 = pose.rotation();
   // model.getPose("TCP_gripper_A", pose);
    Eigen::Vector3d xyz = pose.translation();

    std::cout << " gripper pose" << pose.translation() << std::endl;
    std::cout << " gripper pose orientation" << q1111 << std::endl;

    std::cout << " jointpositions.data(i)" << jointpositions.data << std::endl;

    Frame cartpos;
    ChainJntToJacSolver kdl_solver(chain);
    ChainJntToJacDotSolver kdl_solver_JacDot(chain);
    fksolver.JntToCart(jointpositions,cartpos);

    std::cout << cartpos <<std::endl << std::endl;
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


    ofstream examplefile;   
    ofstream examplefile_des;   
    ofstream examplefile_des_velocity;   

    examplefile.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Orentation.txt");
    examplefile_des.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos.txt");
    examplefile_des_velocity.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/EndangularVelocity.txt");

    Jacobian jacobian_kdl(Number_Joint);
    Jacobian jacobian_Dot_kdl(Number_Joint);

    Eigen :: MatrixXd Rotation_Error;
    Eigen :: MatrixXd Rotation_op(3,3);
    Eigen :: MatrixXd Rotation_op_init(3,3);



    Eigen::VectorXd Error_(3);
    Error_.setZero();
    double t;


    t=0;
    pp = 0;
    std::vector<Eigen::Vector3d> p_list;

    /*robot->sense();
    robot->getJointPosition(q0);*/

    /*for(unsigned int i=0;i<Number_Joint;i++)
    {
     jointpositions.data(i) = q0(i);
     //jointpositions.data(i) = 3.14/3;
    }*/
    fksolver.JntToCart(jointpositions,cartpos);
    // Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.707 * M_PI, Eigen::Vector3d::UnitX());   

    // Eigen::AngleAxisd rollAngle(0.5 * M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.5* M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.5 * M_PI, Eigen::Vector3d::UnitX());   


     Eigen::AngleAxisd rollAngle(-0.707 * M_PI, Eigen::Vector3d::UnitZ());
     Eigen::AngleAxisd yawAngle(-1* M_PI, Eigen::Vector3d::UnitY());
     Eigen::AngleAxisd pitchAngle(-1* M_PI, Eigen::Vector3d::UnitX());   
    //Eigen::AngleAxisd rollAngle(0.1 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.1 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;


	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);

	}
    Rotation_op = pose.rotation();
    Rotation_op_init =  Rotation_op;

     
     std::cout <<"robot error" << "x=" << xyz(0) - cartpos.p(0)  << "y="<< xyz(1) - cartpos.p(1)  << "z=" << xyz(2) - cartpos.p(2)  << std:: endl<<std::endl;
    Eigen :: MatrixXd Rotation_desire = q.matrix() * Rotation_op_init;
    // Rotation_desire = Rotation_op_init;
   // Rotation_desire =  Rotation_op_init;

    MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op_init, Rotation_Error);
    traj.Rotation_Error_Traj = Rotation_Error;
    traj.init_Orientation_Traj = Rotation_op_init;

    Eigen::Vector3d p;
    Eigen::Vector3d v;
    v.setZero();

    double pos_des[3];
    

    p[0] = xyz(0);
    p[1] = xyz(1);
   // p[1] = cartpos.p(1);
    p[2] = xyz(2);

    p_list.push_back(p); 


    p[0] = cartpos.p(0)+0.0;
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2)+0.25;

    p_list.push_back(p); 

     pp_=600;
   
    // std::cout << "p_list[0]"  << p_list[0] - xyz << std::endl;

    Eigen::VectorXd Online_Point((pp_+1)*3);
    Online_Point.setZero();
     /* traj.init_Orientation_Traj = Rotation_op_init;
     traj.Rotation_Error_Traj = Rotation_Error; */

    bool success = traj.trajGeneration(p_list,v);
    
    //std::cout << success <<std::endl;

     /* robot->sense();
     robot->getJointPosition(q0);
     jointpositions_vector = q0; */
     
     std::cout <<"ddddd" <<xyz - traj.getTrajPos(0)<< std::endl; 


     Eigen::MatrixXd Error_Rotation_op(3,3);
     Eigen::VectorXd Unhated_Oreintation;

     // std::cout << Rotation_Error.transpose() << std::endl;

    t_segment = 0;
   // pp_ = -1;
while(pp<=pp_)
    {


    
   
    t_segment = t_segment + dt_orientation;
    // MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);
    // t_segment = t_segment + 0.001;


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
   

   // MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);

//if(pp<=600)
{
     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;



  //       Slack_Vector(0) = 0;
   //  Slack_Vector(1) = 0;
  //   Slack_Vector(2) = 0;
 //    Slack_Vector(3) = 1;
  //   Slack_Vector(4) = 1;
   //  Slack_Vector(5) = 1; 
     // t_segment = 1;

         MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
     //  Rotation_desire =   Rotation_op_init;                                       
}


/*if(pp>300)
{
     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;
     // t_segment = 1.0;
   //  t_segment = t_segment + 0.03;

         MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_segment , Rotation_op_init, 
                                                   Rotation_Error, Rotation_desire);
                                                        Rotation_desire =   Rotation_op_init;   
}

*/

    model.getPose("TCP_gripper_A", pose);
    xyz = pose.translation();

    cartpos.p(0) = xyz(0);
    cartpos.p(1) = xyz(1);
    cartpos.p(2) = xyz(2);

    Rotation_op = pose.rotation();
     MPC_OPTIMAL_.First_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_first),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon_first, Number_Joint, t,Slack_Vector,
                                _Matrix_FirstHierachy, _velcity_FirstHierachy, 1); 
result_jointVel =  _velcity_FirstHierachy;
if(pp<=50)
{
   result_jointVel =  _velcity_FirstHierachy.setZero(); 
}    
     // if(pp>=400)
     
 /*    {

      for(int i=0;i<6;i++)
      {
         Slack_Vector(i) = 1 -Slack_Vector(i);   
      }


    }
    // Slack_Vector.setZero();


     MPC_OPTIMAL_.Second_Hieracy(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon_second),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                _Matrix_FirstHierachy, _velcity_FirstHierachy,
                                dt, horizon_second, Number_Joint, t,Slack_Vector,
                                result_jointVel, 0);  
     */
 
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

   // jointpositions_vector = q0;

    // std::cout << "Rotation_desire.transpose() " << Rotation_desire  <<std::endl;
    // std::cout << "Rotation_desire.transpose() " << Rotation_desire.transpose()  <<std::endl;
    Error_Rotation_op = Rotation_desire.transpose() * Rotation_op ;
    //std::cout << "Different_"<<Rotation_desire.log() - Rotation_op.log() << std::endl;
    std::cout << "Error_Rotation_op.log()" << std::endl << Error_Rotation_op.log() << std::endl;

    Eigen::VectorXd Desire_Rotation_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_desire.log());
    Eigen::VectorXd Rotation_op_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_op.log());
    Eigen::VectorXd Rotation_Error_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Error_Rotation_op.log());
    Eigen::VectorXd Angular_Velocity = Jacobian_4Robot_End_4angvelocity * result_jointVel;

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
        examplefile << Error_(0) <<"\t"<<  Error_(1) <<"\t" << Error_(2) <<std::endl;
          }

    if (examplefile_des.is_open()) {
        examplefile_des << Rotation_Error_VECTOR(0) <<"\t"<<  Rotation_Error_VECTOR(1) <<"\t" << Rotation_Error_VECTOR(2) <<std::endl;
    }

    if (examplefile_des_velocity.is_open()) {
        examplefile_des_velocity << Angular_Velocity(0) <<"\t"<<  Angular_Velocity(1) <<"\t" << Angular_Velocity(2) <<std::endl;
    }

      traj.visTrajectory_online(pp);

     std::cout << "Error_ = " << Error_ << std::endl;
     std::cout << "Velocity joint = " << result_jointVel << std::endl;
     traj.Online_Point_ = Online_Point;
      

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



return 0; 

}
