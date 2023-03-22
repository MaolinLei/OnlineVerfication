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

    kdl_parser::treeFromFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf",my_tree);

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
    robot->getJointPosition(q0);
    q0(1) = q0(1) - 0.0; 
    



   for(int i=0 ;i<=10;i++)
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
    robot->getJointPosition(q0);

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
 
    double dt_orientation = 0.001;
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


     Eigen::AngleAxisd rollAngle(0.0 * M_PI, Eigen::Vector3d::UnitZ());
     Eigen::AngleAxisd yawAngle(-0.1* M_PI, Eigen::Vector3d::UnitY());
     Eigen::AngleAxisd pitchAngle(0.0 * M_PI, Eigen::Vector3d::UnitX());   
    //Eigen::AngleAxisd rollAngle(0.1 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.1 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;


	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);

	}

    Rotation_op_init =  Rotation_op;

     
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
    

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p); 


    p[0] = cartpos.p(0)+0.0;
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2)+0.25;

    p_list.push_back(p); 

     pp_=600;

    Eigen::VectorXd Online_Point((pp_+1)*3);
    Online_Point.setZero();
     /* traj.init_Orientation_Traj = Rotation_op_init;
     traj.Rotation_Error_Traj = Rotation_Error; */

    bool success = traj.trajGeneration(p_list,v);
    
    //std::cout << success <<std::endl;

     /* robot->sense();
     robot->getJointPosition(q0);
     jointpositions_vector = q0; */
     


     Eigen::MatrixXd Error_Rotation_op(3,3);
     Eigen::VectorXd Unhated_Oreintation;

     // std::cout << Rotation_Error.transpose() << std::endl;

    t_segment = 0;

    std::cout << q0 << std::endl;

    q0(4) = q0(4) + 0.1;
    
while(pp<=100)
    {

        model.setJointPosition(q0);
        model.setJointVelocity(velocity_inLastTIme);
        model.setJointAcceleration(acceleration);
        model.update();
        model.computeInverseDynamics(tau);

        control_robot_postion_test(robot ,model, q0, velocity_inLastTIme, tau);
    
   pp++;
     ros::spinOnce();

     rate.sleep();
    }



return 0; 

}
