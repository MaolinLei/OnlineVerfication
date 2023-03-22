#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>

using namespace KDL;



void control_robot_postion_test(auto robot ,auto& model, Eigen::VectorXd desire_position, Eigen::VectorXd desire_velocity, Eigen::VectorXd tau)
{

        //model.update();
        robot->setPositionReference(desire_position);
        robot->setVelocityReference(desire_velocity);       
        robot->setEffortReference(tau);
        robot->move();

}





Eigen::VectorXd Position_Joint(7);
Eigen::VectorXd Position_Joint_left(7);

 void poseCallback(const xbot_msgs::JointState& msg)
{
Eigen::VectorXf Position_Joint_float(7);
Eigen::VectorXf Position_Joint_left_float(7);
for(int j =25;j<32; j++)
{   
    //std::cout << msg.link_position[j] << std::endl;
    Position_Joint_float(j-25) = msg.link_position[j];
}

for(int j =32;j<39; j++)
{   
   // std::cout << msg.link_position[j] << std::endl;
    Position_Joint_left_float(j-32) = msg.link_position[j];
}
 //std::cout << "---------------------"<< std::endl;
 Position_Joint = Position_Joint_float.cast<double>();
 Position_Joint_left = Position_Joint_left_float.cast<double>();
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


    using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_Center"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;
    ros::NodeHandle n_;
    ros::NodeHandle nh;
    ros::NodeHandle nfind;

    ros::Subscriber sub = nh.subscribe("/xbotcore/joint_states", 800, &poseCallback);
    // ros::Subscriber sub_inshadow = nfind.subscribe("/darknet_ros/found_object", 1000, &Callshadow);

    MPC_OPTIMAL MPC_OPTIMAL_;

    ros::Publisher Cen_joint_cmd_pub = MPC_OPTIMAL_.init_Pub_Control_Node(nh);


    Tree my_tree;
    Tree my_tree_left;
    kdl_parser::treeFromFile("/home/oem/hhcm_ws/src/iit-centauro-ros-pkg/centauro_urdf/urdf/centauro.urdf",my_tree);
    YAML::Node  arm_param_config = YAML::LoadFile("/home/oem/hhcm_ws/config/Cen_Robot_dualarm_left.yaml");
    YAML::Node  arm_param_config_left = YAML::LoadFile("/home/oem/hhcm_ws/config/Cen_Robot_dualarm_right.yaml");

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> arm_link_names_;
    for (auto iter : arm_param_config["arm_link_names"])
    {
        arm_link_names_.push_back(iter.as<std::string>());
    }

    std::vector<std::string> arm_link_names_left;
    for (auto iter : arm_param_config["arm_link_names"])
    {
        arm_link_names_left.push_back(iter.as<std::string>());
    }
    // get starting position

    // fcl::CollisionType collisionStatus;

/////////////////////////////////////////////////////////////////////////////////////////////////


    traj traj_left;
    traj_left.dt = dt;
    traj_left._Acc = 1.0;
    traj_left._Vel = 2.0;
    traj_left._dev_order = 3.0;
    traj_left.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory_left",1,true);

    traj traj;
    traj.dt = dt;
    traj._Acc = 1.0;
    traj._Vel = 2.0;
    traj._dev_order = 3.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    // traj.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    // traj.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    // traj.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true);
    //traj.sphere_visual_Random = n.advertise<visualization_msgs::Marker>("ProperVolume_random",1,true);

    int horizon = 10;

    Chain chain, chain_left;
    my_tree.getChain("torso_2","ball1_tip",chain);
    my_tree.getChain("torso_2","ball2_tip",chain_left);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    ChainFkSolverPos_recursive fksolver_left = ChainFkSolverPos_recursive(chain_left);

    unsigned int nj = chain.getNrOfJoints();
    unsigned int nj_left = chain_left.getNrOfJoints();


	
    JntArray jointpositions(nj);
    JntArrayVel jointVelocity(nj);
    JntArray jointpositions_left(nj_left);
    JntArrayVel jointVelocity_left(nj_left);

    JntArray jointpositions_middle(nj);
    JntArray jointVelocity_left_middle(nj_left);
    
     // q0 = Position_Joint;

    int Wait=0;
    while(Wait <= 100)
    {
        Wait ++ ;
        
        // std::cout << Wait <<std::endl;

        // MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub, q0);
        ros::spinOnce();
        rate.sleep();

    }
   
     //return 0;


    //cout << chain.getSegment(0).getName() << endl <<endl;

    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) = Position_Joint(i);
     jointpositions_left.data(i) = Position_Joint_left(i);
    }

    Frame cartpos;
    ChainJntToJacSolver kdl_solver(chain);
    ChainJntToJacDotSolver kdl_solver_JacDot(chain);
    fksolver.JntToCart(jointpositions,cartpos);


    Frame cartpos_left;
    ChainJntToJacSolver kdl_solver_left(chain_left);
    ChainJntToJacDotSolver kdl_solver_JacDot_left(chain_left);
    fksolver_left.JntToCart(jointpositions_left,cartpos_left);
///////////////////////////////////////////////////////////////////////////////////////////////////////////    
    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);

	for(int i=0; i<3; i++) 
  {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}

    Eigen::MatrixXd Rotation_op_left(3,3);
    Eigen::MatrixXd Error_Rotation_op_left(3,3);

	for(int i=0; i<3; i++) 
  {
		for(int j=0; j<3; j++)
			Rotation_op_left(i,j) = cartpos_left.M(i,j);
	}
 
    
    
    int pp=0;
    int Number_Joint = nj ;
    int Number_Joint_left = nj_left ;

    Eigen:: VectorXd Joint_velocity_(nj);
    Joint_velocity_.setZero();

    Eigen:: VectorXd Joint_velocity_left(nj_left);
    Joint_velocity_left.setZero();

    for(unsigned int i=0;i<nj;i++)
    {
     jointVelocity.q.data(i) = jointpositions.data(i) ;
     jointVelocity.qdot.data(i) = Joint_velocity_(i);
    }
    for(unsigned int i=0;i<nj_left;i++)
    {
     jointVelocity_left.q.data(i) = jointpositions_left.data(i) ;
     jointVelocity_left.qdot.data(i) = Joint_velocity_left(i);
    }


    double t;
    Eigen ::MatrixXd Jacobian_Nearest;
    Eigen::Vector3d Position_NearPoint(0.00,0.00,0.00);
    //Jacobian_Nearest = Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], jointpositions_vector.topRows(Nearest_Link_), Position_NearPoint);
     Eigen::VectorXd Desire_position(nj);
     Desire_position.setZero();

//////////////////////////////////////////
    std::vector<std::string> arm_rotation_joints;
    ArmCollision collision_detection(my_tree, arm_param_config, arm_rotation_joints);

    std::vector<std::string> arm_rotation_joints_left;
    collision_detection.ArmCollision_dualarm(my_tree, arm_param_config_left, arm_rotation_joints_left);


    double distance;
    Eigen::Vector3d nearest_points[2];

    Eigen::VectorXd q0(7);
    Eigen::VectorXd q0_left(7);
    int number_joint_slave;
    Eigen::Vector2i number_joint;

    q0 =Position_Joint;
    q0_left = Position_Joint_left;

    collision_detection.joint_states_input_ = q0;
    collision_detection.joint_states_input_left = q0_left;
    collision_detection.calFowardKinematics();
    collision_detection.calFowardKinematics_left();

    distance = collision_detection.calMinCollisionDistance_dualarm(nearest_points, number_joint);

    // collisionStatus = collision_detection.getCollisionStatus();

////////////////////////////////////////////////////////////////////
    std::vector<Eigen::Vector3d> p_list;

    double pos_des[3];
    Eigen::Vector3d p;
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p); 

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p); 

   /* pos_des[0] = cartpos.p(0) + 0.0;
    pos_des[1] = cartpos.p(1) - 0.25;
    pos_des[2] = cartpos.p(2) + 0.1;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); 

    pos_des[0] = cartpos.p(0) + 0.15;
    pos_des[1] = cartpos.p(1) - 0.15;
    pos_des[2] = cartpos.p(2) - 0.;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list.push_back(p); 
*/




    Eigen::Vector3d v;
    v.setZero();

    bool success = traj.trajGeneration(p_list,v);



    std::vector<Eigen::Vector3d> p_list_left;
    
    //double pos_des[3];
    //Eigen::Vector3d p;
    p[0] = cartpos_left.p(0);
    p[1] = cartpos_left.p(1);
    p[2] = cartpos_left.p(2);

    p_list_left.push_back(p); 
    

   /* pos_des[0] = cartpos_left.p(0) + 0.0;
    pos_des[1] = cartpos_left.p(1) + 0.25;
    pos_des[2] = cartpos_left.p(2) + 0.1;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list_left.push_back(p); 

    pos_des[0] = cartpos_left.p(0) + 0.15 ;
    pos_des[1] = cartpos_left.p(1) - 0.15;
    pos_des[2] = cartpos_left.p(2) + 0.0;
    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];
    p_list_left.push_back(p); 
 */

    p[0] = 0.2;
    p[1] = -0.55;
    p[2] = 0;
    p_list_left.push_back(p); 

    p[0] = 0.5;
    p[1] = -0.5;
    p[2] = -0.2;
    p_list_left.push_back(p); 


    //Eigen::Vector3d v;
    v.setZero();

    bool success_left = traj_left.trajGeneration(p_list_left,v);

    //Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.707 * M_PI, Eigen::Vector3d::UnitX());   
    Eigen::AngleAxisd rollAngle(-0.0* M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.0 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.0* M_PI, Eigen::Vector3d::UnitX());  

    // Eigen::AngleAxisd rollAngle(0.0* M_PI, Eigen::Vector3d::UnitZ());
    // Eigen::AngleAxisd yawAngle(0.01 * M_PI, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd pitchAngle(0.010* M_PI, Eigen::Vector3d::UnitX());

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    Eigen :: MatrixXd Rotation_desire = q.matrix();
    Rotation_desire = Rotation_op  * Rotation_desire;

    Eigen::MatrixXd _Rotation_Error_ = (Rotation_op.transpose() * Rotation_desire).log();
    Eigen::MatrixXd Rotation_desire_record = Rotation_desire;
    Eigen::MatrixXd _Rotation_op_init = Rotation_op;

    Eigen::VectorXd desire_velocity(Number_Joint);
    desire_velocity.setZero();
    Eigen::VectorXd desire_velocity_horizon(horizon*Number_Joint);
    desire_velocity_horizon.setZero();
    Eigen::VectorXd desire_velocity_left(Number_Joint);
    desire_velocity_left.setZero();
    Eigen::VectorXd desire_velocity_horizon_left(horizon*Number_Joint);
    desire_velocity_horizon.setZero();


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

    Jacobian jacobian_kdl_left(nj);
    Jacobian jacobian_Dot_kdl_left(nj);
   
        
    double t_collision;
    double t_orientation = 0;


     // std::cout << "Position_Joint_initiatl = " <<Position_Joint <<std::endl;
    // std::cout << Position_Joint_left << std::endl;
    
    // std::cout << "---------------------"<< std::endl;
     // std::cout << "Desire_position----?" << Desire_position <<std::endl;
     // std::cout << "q0----?" << q0 <<std::endl;

    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) =  q0(i);
     


    } 
    int Q = 0;
    //while(Q <= 0)

    t = 0;
    int check_init= 4;
    int collision_horizon = 0;
    int number_collision_joint = 0;
    int p_ = 0;
    Eigen::VectorXd Position_Joint_horizon(Number_Joint*horizon);
    Eigen::VectorXd Position_Joint_left_horizon(Number_Joint*horizon);
    Position_Joint_horizon.setZero();
    Position_Joint_left_horizon.setZero();
    int t_ = 1;
    int collision_horizon_change = 100;
    Eigen::VectorXd Changing_Position(14);
    Changing_Position.setZero();
    //int collision_horizon_change = 50;

    Eigen::MatrixXd Rotation_matrix;
    Eigen::MatrixXd Rotation_matrix_left;
    MPC_OPTIMAL_.Relatvie_Orientation( traj.getVel(4) , Rotation_matrix);
    Eigen::VectorXd increase_(7);
    increase_.setZero();
    Eigen::VectorXd increase_left(7); 
    increase_left.setZero();
    Eigen::VectorXd Position_Joint_desired(7);
    Eigen::VectorXd Position_Joint_desired_left(7);
    Eigen::VectorXd Joint_pos_des_(nj);
    Eigen::VectorXd Joint_pos_des_left(nj); 

    Eigen::VectorXd cost_test_(nj);
    cost_test_.setZero();
    Eigen::VectorXd cost_test_left(nj);
    cost_test_left.setZero();

    double cost_incaseI = 0;
    double cost_incaseII = 0;

    Eigen::VectorXd DesiredPos_(3);
    Eigen::VectorXd DesiredPos_left(3); 
    DesiredPos_.setZero();
    DesiredPos_left.setZero();
    
    double waiting_loop = 0;
    double waiting_loop_left = 0;
    int decelerate = 0;
    int Decellecearte = 1;

while(pp<=600)
   {

    Eigen::VectorXd Ag1_changing(nj);
    Eigen::VectorXd Ag1_changing_left(nj); 

    Ag1_changing.setZero();
    Ag1_changing_left.setZero();

   if(decelerate == 0)
    {
    Decellecearte = 1;
    for(unsigned int i=0;i<nj;i++)
    {
    // jointpositions.data(i) = q0(i) ;

     jointVelocity.q.data(i) = jointpositions.data(i) ;
     jointVelocity.qdot.data(i) = Joint_velocity_(i);
    
    }

    for(unsigned int i=0;i<nj_left;i++)
    {
    // jointpositions.data(i) = q0(i) ;

     jointVelocity_left.q.data(i) = jointpositions_left.data(i) ;
     jointVelocity_left.qdot.data(i) = Joint_velocity_left(i);
    
    }

    fksolver.JntToCart(jointpositions,cartpos);
    kdl_solver.JntToJac(jointpositions,jacobian_kdl) ;
    kdl_solver_JacDot.JntToJacDot(jointVelocity,jacobian_Dot_kdl);


    fksolver_left.JntToCart(jointpositions_left,cartpos_left);
    kdl_solver_left.JntToJac(jointpositions_left,jacobian_kdl_left) ;
    kdl_solver_JacDot_left.JntToJacDot(jointVelocity_left,jacobian_Dot_kdl_left);

    Eigen:: VectorXd Position_End_(3);
    Eigen:: VectorXd Position_End_left(3);
    


    Eigen:: MatrixXd Jacobian_4Robot_End;
    Eigen:: MatrixXd Jacobian_4Robot_End_4velocity;
    Eigen:: MatrixXd Jacobian_4Robot_End_4angvelocity;
    Eigen:: MatrixXd velocity_Jacobian_end_;
    Eigen:: VectorXd angvelocity_Jacobian_end_;
    Eigen:: MatrixXd JacobianDot_4Robot_End_;

    Jacobian_4Robot_End = jacobian_kdl.data;
    Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);
    velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;
    JacobianDot_4Robot_End_ = jacobian_Dot_kdl.data;


    Eigen:: MatrixXd Jacobian_4Robot_End_left;
    Eigen:: MatrixXd Jacobian_4Robot_End_4velocity_left;
    Eigen:: MatrixXd Jacobian_4Robot_End_4angvelocity_left;
    Eigen:: MatrixXd velocity_Jacobian_end_left;
    Eigen:: VectorXd angvelocity_Jacobian_end_left;
    Eigen:: MatrixXd JacobianDot_4Robot_End_left;

    Jacobian_4Robot_End_left = jacobian_kdl_left.data;
    Jacobian_4Robot_End_4velocity_left = Jacobian_4Robot_End_left.topRows(3);
    Jacobian_4Robot_End_4angvelocity_left = Jacobian_4Robot_End_left.bottomRows(3);
    velocity_Jacobian_end_left = Jacobian_4Robot_End_4velocity_left * Joint_velocity_left;
    angvelocity_Jacobian_end_left = Jacobian_4Robot_End_4angvelocity_left * Joint_velocity_left;
    JacobianDot_4Robot_End_left = jacobian_Dot_kdl_left.data;


    if (t_orientation <=1)
    {
    t_orientation = (pp+1)*dt/100;
    }
    
    // std::cout << "Jacobian = " << Jacobian_4Robot_End << std::endl;

    // Rotation_desire =  _Rotation_op_init * (t_orientation * _Rotation_Error_).exp() ;

    Eigen::VectorXd Position_disire_(3*horizon); 
    Eigen::VectorXd Velocity_disire_(3*horizon); 
    Eigen::VectorXd Pos_Vel_disire_(6*horizon); 

    Eigen::VectorXd Position_disire_left(3*horizon); 
    Eigen::VectorXd Velocity_disire_left(3*horizon); 
    Eigen::VectorXd Pos_Vel_disire_left(6*horizon); 
  
    double t_qp =0 ;

    for(int i=0;i<horizon;i++)
    {
    

    t = (pp )*dt + dt*i  + 0.0;

    Position_disire_.middleRows(i*3,3) = traj.getTrajPos(t - waiting_loop_left*dt);
    Velocity_disire_.middleRows(i*3,3) = traj.getVel(t - waiting_loop_left*dt);
    
    Pos_Vel_disire_.middleRows(i*3,3) = traj.getTrajPos(t - waiting_loop_left*dt) ;
    Pos_Vel_disire_.middleRows(3+i*3,3) = traj.getVel(t - waiting_loop_left*dt) ;   

  
    Position_disire_left.middleRows(i*3,3) = traj_left.getTrajPos(t- waiting_loop*dt);
    Velocity_disire_left.middleRows(i*3,3) = traj_left.getVel(t- waiting_loop*dt);
    
    Pos_Vel_disire_left.middleRows(i*3,3) = traj_left.getTrajPos(t- waiting_loop*dt) ;
    Pos_Vel_disire_left.middleRows(3+i*3,3) = traj_left.getVel(t- waiting_loop*dt) ;   
    }



  
	  for(int i=0; i<3; i++) 
    {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	  }
  

     p_=0;
     MPC_OPTIMAL_.optimal_online(Jacobian_4Robot_End, JacobianDot_4Robot_End_,  
                    cartpos, Pos_Vel_disire_,
                    Rotation_desire, Rotation_op,
                    Joint_velocity_, Position_Joint,
                    dt, horizon, Number_Joint, t,
                    desire_velocity_horizon);

     double cost_right = MPC_OPTIMAL_.cost_int_ ;

     MPC_OPTIMAL_.optimal_online(Jacobian_4Robot_End_left, JacobianDot_4Robot_End_left,  
                    cartpos_left, Pos_Vel_disire_left,
                    Rotation_desire, Rotation_op_left,
                    Joint_velocity_left, Position_Joint_left,
                    dt, horizon, Number_Joint_left, t,
                    desire_velocity_horizon_left); 

     // double cost_left = MPC_OPTIMAL_.cost_int_ ;

      // std::cout << "Error_" << Position_End_left.topRows(3) - Position_disire_left.topRows(3) << std::endl;
      // std::cout << "Error_" << Position_End_.topRows(3) - Position_disire_.topRows(3) << std::endl;


     for(int i=0; i< horizon;i++)
     {
       Position_Joint_horizon.middleRows(i*Number_Joint, Number_Joint) = Position_Joint;
       Position_Joint_left_horizon.middleRows(i*Number_Joint, Number_Joint) = Position_Joint_left;
     }


    Eigen::VectorXd distance_(horizon-check_init);
    distance_.setZero();
    Eigen::VectorXd nearest_points_((horizon-check_init)*3);
    Eigen::VectorXd nearest_points_left((horizon-check_init)*3);
    nearest_points_left.setZero();
    nearest_points_.setZero();
    Eigen::VectorXi number_joint_((horizon-check_init)*3);
    Eigen::VectorXi number_joint_left((horizon-check_init)*3);
    
    Eigen::VectorXd NearPoint_Vector(6);
    Eigen::VectorXd Collision_Joint_position(7);
    Eigen::VectorXd Collision_Joint_position_left(7);

    number_collision_joint = collision_detection.Onlineverification(Number_Joint, horizon, dt, check_init,
                                                                      Position_Joint_horizon, Position_Joint_left_horizon,
                                                                      desire_velocity_horizon, desire_velocity_horizon_left,
                                                                      distance_,  nearest_points_, nearest_points_left,
                                                                      number_joint_, number_joint_left,
                                                                      NearPoint_Vector, Collision_Joint_position, Collision_Joint_position_left);

    Eigen::Vector3d NearPoint[2];
    NearPoint[0] =  NearPoint_Vector.topRows(3);
    NearPoint[1] =  NearPoint_Vector.bottomRows(3);
    
     for(int i=0;i<nj;i++)
    {

     // std::cout << "ddddddddddddd" << std::endl;
     desire_velocity(i) = desire_velocity_horizon(i+p_*nj);
     desire_velocity_left(i) = desire_velocity_horizon_left(i+p_*nj);

    }

  // number_collision_joint = 1;
   if(number_collision_joint)
   {
 /*  std::cout << "collision_horizon" << collision_horizon << std::endl; 
     std::cout << "nearest_points_" << nearest_points_ << std::endl; 
     std::cout << "nearest_points_left" << nearest_points_left << std::endl; 
     std::cout << "distance_" << distance_ << std::endl; 
     std::cout << "NearPoint" << NearPoint[0] << std::endl;
     std::cout << "NearPoint[1]" << NearPoint[1] << std::endl;

     std::cout << "NearPoint_" << NearPoint_Vector.topRows(3) << std::endl;
     std::cout << "NearPoint_]" << NearPoint_Vector.bottomRows(3) << std::endl;

     std::cout << "NearPoint" << distance_(collision_horizon) << std::endl;
     std::cout << "number_joint_" << number_joint_(collision_horizon) << std::endl;
     std::cout << "number_joint_left" << number_joint_left(collision_horizon) << std::endl;
*/


        t_qp = pp * dt;
        q0 = Position_Joint;
        q0_left = Position_Joint_left;

        collision_detection.joint_states_input_ = q0;
        collision_detection.joint_states_input_left = q0_left;
        collision_detection.calFowardKinematics();
        collision_detection.calFowardKinematics_left();
        

        for(int p=0;p<3;p++)
        {
        Position_End_(p) = cartpos.p(p);
        Position_End_left(p) = cartpos_left.p(p);   
        } 
        // Eigen::VectorXd number_joint(horizon-4);
    
        distance = collision_detection.calMinCollisionDistance_dualarm(nearest_points, number_joint);
        
        Eigen::VectorXd Difference_reference_;
        Eigen::VectorXd Difference_reference_left;
        
       // Difference_reference_ = Position_End_ -Position_disire_.topRows(3);
       // Difference_reference_left = Position_End_left -Position_disire_left.topRows(3);
       
        if(t_qp-0.01>=0)
        {
            t_qp = 0.01;
        }

       // Difference_reference_ = traj.getTrajPos(t_qp) - traj.getTrajPos(t_qp-0.01) ;
       // Difference_reference_left = traj_left.getTrajPos(t_qp) - traj_left.getTrajPos(t_qp-0.01);

        MPC_OPTIMAL_.Relatvie_Orientation(traj.getVel(t_qp+0.0) , Rotation_matrix);
        MPC_OPTIMAL_.Relatvie_Orientation(traj_left.getVel(t_qp+0.0) , Rotation_matrix_left);

       /* Difference_reference_ = traj.getTrajPos(t_qp+0.01) - traj.getTrajPos(t_qp) ;
        Difference_reference_left = traj_left.getTrajPos(t_qp+0.01) - traj_left.getTrajPos(t_qp);
        MPC_OPTIMAL_.Relatvie_Orientation(Difference_reference_ , Rotation_matrix);
        MPC_OPTIMAL_.Relatvie_Orientation(Difference_reference_left, Rotation_matrix_left);
      */
        // Rotation_matrix = Eigen::MatrixXd::Identity(3,3);
        // Rotation_matrix_left = Eigen::MatrixXd::Identity(3,3);

        Jacobian_4Robot_End.topRows(3) = Rotation_matrix * Jacobian_4Robot_End.topRows(3);
        Jacobian_4Robot_End_left.topRows(3) = Rotation_matrix_left * Jacobian_4Robot_End_left.topRows(3); 
        Position_disire_.topRows(3) = Rotation_matrix *   Position_disire_.topRows(3) ;   
        Position_disire_left.topRows(3) = Rotation_matrix_left *   Position_disire_left.topRows(3) ;  
        Position_End_.topRows(3)  = Rotation_matrix *   Position_End_.topRows(3) ;  
        Position_End_left.topRows(3)  = Rotation_matrix_left *   Position_End_left.topRows(3) ;         
        
        MPC_OPTIMAL_.QP_online_DualArm(number_joint(0), number_joint(1), 
                                    arm_link_names_[number_joint(0)],  arm_link_names_left[number_joint(1)],
                                    Position_End_, Position_End_left,
                                    Jacobian_4Robot_End, Jacobian_4Robot_End_left,
                                    Position_Joint, Position_Joint_left, 
                                    Joint_velocity_, Joint_velocity_left, 
                                    nearest_points[0], nearest_points[1],
                                    Position_disire_.topRows(3), Position_disire_left.topRows(3),
                                    my_tree,  distance,
                                    Changing_Position);

    JntArray jointpositions_middle(nj);
    JntArray jointpositions_left_middle(nj_left);

      for(unsigned int i=0;i<nj;i++)
    {  
    cost_test_(i) = jointpositions.data(i) +   Changing_Position(i);
    cost_test_left(i) = jointpositions_left.data(i) +   Changing_Position(i+nj);

    jointpositions_middle.data(i) = cost_test_(i);
    jointpositions_left_middle.data(i) = cost_test_left(i);

    }


    fksolver.JntToCart(jointpositions_middle,cartpos);
    fksolver_left.JntToCart(jointpositions_left_middle,cartpos_left);

    Eigen::VectorXd Error_middle_(3);
    Eigen::VectorXd Error_middle_left(3);
    Error_middle_.setZero();
    Error_middle_left.setZero();
    
    Error_middle_(0) = cartpos.p(0) - Position_disire_(0);
    Error_middle_left(0) = cartpos_left.p(0) - Position_disire_left(0);

    Error_middle_(1) = cartpos.p(1) - Position_disire_(1);
    Error_middle_left(1) = cartpos_left.p(1) - Position_disire_left(1);

    Error_middle_(2) = cartpos.p(2) - Position_disire_(2);
    Error_middle_left(2) = cartpos_left.p(2) - Position_disire_left(2);

    collision_detection.State_input(cost_test_,cost_test_left);
    collision_detection.callDistance_cost();
    cost_incaseI = collision_detection.Distance_sum_average;
    cost_incaseI = 1*cost_incaseI - 0.5 * (Error_middle_.norm() + Error_middle_left.norm())/1.8;
    
    Ag1_changing = Changing_Position.topRows(nj); 
    Ag1_changing_left = Changing_Position.bottomRows(nj) ;


    std::cout << "cost_incaseI =" << cost_incaseI <<std::endl;
   }


     if(number_collision_joint)
      {
        DesiredPos_ = traj.getTrajPos(t);
        DesiredPos_left = traj_left.getTrajPos(t) ;
        
        // waiting_loop_left ++;
        waiting_loop ++ ;
        
        Changing_Position.setZero();
        // desire_velocity_left.setZero();

        for(int i=0;i<nj;i++)
        {
        Changing_Position(i) = desire_velocity(i)*dt;
        }


        for(int i=0;i<nj;i++)
        {
          Changing_Position(i+nj) = 0;
        }
      
            for(unsigned int i=0;i<nj;i++)
        {  
        cost_test_(i) = jointpositions.data(i) +   Changing_Position(i);
        cost_test_left(i) = jointpositions_left.data(i) +    Changing_Position(i+nj);
        }

        collision_detection.State_input(cost_test_,cost_test_left);
        collision_detection.callDistance_cost();
        cost_incaseII = collision_detection.Distance_sum_average;
        cost_incaseII = 1.0 * cost_incaseII - 0.01 * (waiting_loop + waiting_loop_left);

        std::cout << "cost_incaseII =" << cost_incaseII <<std::endl;
        if(cost_incaseII <= cost_incaseI)
        {
         Changing_Position.topRows(nj) = Ag1_changing ; 
         Changing_Position.bottomRows(nj)  = Ag1_changing_left ;
         waiting_loop -- ;

       //   return 0;
        } 
      }



        if(number_collision_joint == 0)
      {
          check_init = 4;
      } 


    for(unsigned int i=0;i<nj;i++)
    {  

      if(number_collision_joint)
      {

        increase_(i) = Changing_Position(i);
        increase_left(i) = Changing_Position(i+nj);

        jointpositions.data(i)  = jointpositions.data(i) + 0 * dt * desire_velocity(i)  +   Changing_Position(i);
        Position_Joint(i) = jointpositions.data(i) ;

        Joint_velocity_(i) = desire_velocity(i)/dt;

        jointpositions_left.data(i)  = jointpositions_left.data(i) +  0 *  dt * desire_velocity_left(i)  +  Changing_Position(i+nj);
        Position_Joint_left(i) = jointpositions_left.data(i) ;

        Joint_velocity_left(i) = desire_velocity_left(i)/dt;
        
        Position_Joint_desired_left = Position_Joint_left;    
        Position_Joint_desired = Position_Joint;     


      // std::cout << "collision_horizon_change" << collision_horizon_change << std::endl;
      // collision_horizon_change = 500;

      check_init = 3;
      //return 0;
      }
      else 
      {

        increase_(i) =  dt * desire_velocity(i);
        increase_left(i) = dt * desire_velocity_left(i);

        jointpositions.data(i)  = jointpositions.data(i)  + dt * desire_velocity(i);
        Position_Joint(i) = jointpositions.data(i) ;
        Joint_velocity_(i) = desire_velocity(i);

        jointpositions_left.data(i)  = jointpositions_left.data(i)  + dt * desire_velocity_left(i);
        Position_Joint_left(i) = jointpositions_left.data(i) ;
        Joint_velocity_left(i) = desire_velocity_left(i);

        Position_Joint_desired_left = Position_Joint_left;    
        Position_Joint_desired = Position_Joint;    

      }
     

    } 
       pp++;
     
  }

  Eigen::MatrixXd::Index maxRow, maxCol;
	Eigen::MatrixXd::Index minRow, minCol;

	double max_ = increase_.maxCoeff(&minRow);
	double max_left = increase_left.maxCoeff(&maxRow);



   if(max_ >=200 ||max_left>=200 && decelerate == 0)
  {
    std::cout << "increase=" << increase_ << std::endl << std::endl;
    std::cout << "increase_left=" << increase_left << std::endl << std::endl;
    increase_ = increase_/1;
    increase_left = increase_left/1;
    decelerate = 3;
    
    Joint_pos_des_ = Position_Joint_desired;
    Joint_pos_des_left = Position_Joint_desired_left;
    std::cout << increase_ << std::endl << std::endl;
    std::cout << increase_left << std::endl << std::endl;
 

  }

   if( decelerate >=1)
   {
    Position_Joint_desired = Joint_pos_des_ - increase_ * decelerate;
    Position_Joint_desired_left = Joint_pos_des_ - increase_left * decelerate;
    decelerate = decelerate - 1;

    if(decelerate == 1)
      {
        decelerate = 0;
      }
    
   // return 0;
   // for(int i=1;i>=0;i--)
   //  Decellecearte =  Decellecearte - 1;
    std::cout << decelerate << std::endl<< std::endl;
    std::cout << "increase=" << increase_ << std::endl << std::endl;
    std::cout << "increase_left=" << increase_left << std::endl << std::endl;
   }

   MPC_OPTIMAL_.CenControl_Pub_Desire_left(Cen_joint_cmd_pub,Position_Joint_desired);
   MPC_OPTIMAL_.CenControl_Pub_Desire_right(Cen_joint_cmd_pub,Position_Joint_desired_left);
     //cout << pp <<endl;
     // std::cout << "(Rotation_op.transpose() * Rotation_desire).log();" << std::endl << (Rotation_op.transpose() * Rotation_desire).log() << std::endl;
     // std::cout << "Desire_position ==" << Desire_position <<std::endl;
     //cout << "Desire_position=" << Desire_position <<endl;
     //std::cout << "cartpos ==" << cartpos <<std::endl;
    // std::cout << "Desire_position =" << Desire_position <<std::endl <<std::endl;


    ros::spinOnce();

    rate.sleep();


   }

return 0; 

}