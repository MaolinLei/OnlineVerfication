#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>
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

    /*expection_4Obj_4Current_Read(0) = msg.poses[0].pose.position.x + 0.0;
    expection_4Obj_4Current_Read(1) = msg.poses[0].pose.position.y;
    expection_4Obj_4Current_Read(2) = msg.poses[0].pose.position.z;

    expection_4Obj_4Current_Oreintation_Read(0) = msg.poses[0].pose.orientation.x;
    expection_4Obj_4Current_Oreintation_Read(1) = msg.poses[0].pose.orientation.y;
    expection_4Obj_4Current_Oreintation_Read(2) = msg.poses[0].pose.orientation.z;
    expection_4Obj_4Current_Oreintation_Read(3) = msg.poses[0].pose.orientation.w;*/


    int n = floor(msg.poses.size()/2);
    expection_4Obj_4Next_Read(0) = msg.poses[n].pose.position.x - 0.5;
    expection_4Obj_4Next_Read(1) = msg.poses[n].pose.position.y;
    expection_4Obj_4Next_Read(2) = msg.poses[n].pose.position.z;

    expection_4Obj_4Next_Read(3) = msg.poses[msg.poses.size()-1].pose.position.x - 0.5 ;
    expection_4Obj_4Next_Read(4) = msg.poses[msg.poses.size()-1].pose.position.y;
    expection_4Obj_4Next_Read(5) = msg.poses[msg.poses.size()-1].pose.position.z;

    expection_4Obj_4Next_Oreintation_Read(0) = msg.poses[n].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(1) = msg.poses[n].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(2) = msg.poses[n].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(3) = msg.poses[n].pose.orientation.w;

    expection_4Obj_4Next_Oreintation_Read(4) = msg.poses[msg.poses.size()-1].pose.orientation.x;
    expection_4Obj_4Next_Oreintation_Read(5) = msg.poses[msg.poses.size()-1].pose.orientation.y;
    expection_4Obj_4Next_Oreintation_Read(6) = msg.poses[msg.poses.size()-1].pose.orientation.z;
    expection_4Obj_4Next_Oreintation_Read(7) = msg.poses[msg.poses.size()-1].pose.orientation.w;


   std::cout << "ROTTOPIC_INIT" << std::endl;


}




void GTCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    for (int i=0; i<msg->name.size(); i++)
    {
        if (msg->name[i].compare("actor") == 0)
        {

            expection_4Obj_4Current_Read(0) = msg->pose[i].position.x-0.5;
            expection_4Obj_4Current_Read(1) = msg->pose[i].position.y;
            expection_4Obj_4Current_Read(2) = expection_4Obj_4Next_Read(2);
            expection_4Obj_4Current_Oreintation_Read(0)= msg->pose[i].orientation.x;
            expection_4Obj_4Current_Oreintation_Read(1) = msg->pose[i].orientation.y;
            expection_4Obj_4Current_Oreintation_Read(2) = msg->pose[i].orientation.z;
            expection_4Obj_4Current_Oreintation_Read(3) = msg->pose[i].orientation.w;

            //std::cout << "gt_.position.x =="<<std::endl;
            //std::cout << "gt_.position.y =="<<std::endl;

            break;
        }
    }
}




    using namespace std;


int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_modular"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle nh_("xbotcore");

    ros::NodeHandle n;
    ros::NodeHandle n_;
    
    ros::Subscriber topic_sub = n.subscribe("obstacle/path", 1000, Topic_Subscrib);
    ros::Subscriber ground_truth_sub_ = n_.subscribe("gazebo/model_states", 1000, &GTCallback);

    Tree my_tree;

    kdl_parser::treeFromFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_6DOF/urdf/ModularBot.urdf",my_tree);

    YAML::Node arm_param_config = YAML::LoadFile("/home/oem/catkin_ws/src/mpc/modularbots_fraunhofer/ModularBot_6DOF/config/ModularBot_6Dof.yaml");

    double dt;
    dt = 0.01;
    ros::Rate rate(1./dt);

///////////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<std::string> arm_link_names_;
    for (auto iter : arm_param_config["arm_link_names"])
    {
        arm_link_names_.push_back(iter.as<std::string>());


    }
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

    // get starting position
    Eigen::VectorXd q0;
    Eigen::VectorXd tau;
    Eigen::VectorXd acceleration(6);

    acceleration.setZero();

    robot->sense();
    robot->getJointPosition(q0);
    q0(1) = q0(1) - 0.15; 
    
   for(int i=0 ;i<=10;i++)
   {

   
       Eigen::VectorXd jointpositions_vector_(6);
       Eigen::VectorXd jointpositions_velcity_(6);
       acceleration.setZero();
       for(int j=0;j<6;j++)
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
        robot->move();
         
        rate.sleep();

   }




/////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<std::string> arm_rotation_joints;
    ArmCollision collision_detection(my_tree, arm_param_config, arm_rotation_joints);


    MPC_OPTIMAL MPC_OPTIMAL_;
    //MPC_OPTIMAL MPC_OPTIMAL_Orientation;    
////////////////////////////////////////////////////////////////////

  traj Collision;
    Collision.dt = dt;
    Collision._Acc = 0.05;
    Collision._Vel = 0.8;
    Collision._dev_order = 4.0;
    Collision.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    Collision.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    Collision.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    Collision.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true); 

    traj traj;
    traj.dt = dt;
    traj._Acc = 0.5;
    traj._Vel = 2.0;
    traj._dev_order = 4.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    traj.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    traj.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true);
    //traj.sphere_visual_Random = n.advertise<visualization_msgs::Marker>("ProperVolume_random",1,true);

    int horizon = 8;

    bool exit_value;
    Chain chain;
    exit_value = my_tree.getChain("base_link","pen_A",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();

    string name_4Link[nj+1];

    //cout<<"nj="<<nj<<endl;
	
    JntArray jointpositions(nj);
    JntArrayVel jointVelocity(nj);
    Eigen::VectorXd jointpositions_vector(nj);
    jointpositions_vector = q0;
    //cout << chain.getSegment(0).getName() << endl <<endl;

    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) = q0(i);
     //jointpositions.data(i) = 3.14/3;

     jointpositions_vector(i)=jointpositions.data(i) ;
    }


    for(unsigned int i=0;i<nj+1;i++)
    {
    name_4Link[i] = arm_link_names_[i];
        std::cout << "arm_link_names_[]=" << arm_link_names_[i] << std::endl;
    }

    Frame cartpos;
    ChainJntToJacSolver kdl_solver(chain);
    ChainJntToJacDotSolver kdl_solver_JacDot(chain);
    fksolver.JntToCart(jointpositions,cartpos);
///////////////////////////////////////////////////////////////////////////////////////////////////////////    
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


    std::vector<Eigen::Vector3d> p_list_collision;
    Eigen::Vector3d p;
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
    p[2] = cartpos.p(2) + 6000 ;
    p_list_collision.push_back(p);

    p[0] = cartpos.p(0) - 0.3 ;
    p[1] = cartpos.p(1) + 1.2  ;
    p[2] = cartpos.p(2) + 0.800 ;
    p[2] = cartpos.p(2) + 800 ;
    p_list_collision.push_back(p);


    Eigen::Vector3d v;
    v.setZero();

    Collision.trajGeneration(p_list_collision,v);

    std::vector<Eigen::Vector3d> p_list;
    double pos_des[3];

    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);
    
    pos_des[0] = cartpos.p(0) - 0.0 ;
    pos_des[1] = cartpos.p(1) - 0.3  ;
    pos_des[2] = cartpos.p(2) + 0.6 ;

    p[0] = pos_des[0];
    p[1] = pos_des[1]  ;
    p[2] = pos_des[2];

   /* p_list.push_back(p);

    p[0] = cartpos.p(0) - 0. ;
    p[1] = cartpos.p(1) - 0.  ;
    p[2] = cartpos.p(2) + 0. ;*/

    /*p_list.push_back(p);

    p[0] = cartpos.p(0) - 0.2 ;
    p[1] = cartpos.p(1) + 0.2  ;
    p[2] = cartpos.p(2) + 0.4 ;*/

    p_list.push_back(p);



    bool success = traj.trajGeneration(p_list,v);

    t = 0;

    ofstream examplefile;   
    ofstream examplefile_des;   
    ofstream examplefile_des_EndVeclocity;   
    examplefile.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos.txt");
    examplefile_des.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_Pos_des.txt");
    examplefile_des_EndVeclocity.open("/home/oem/catkin_ws/src/mpc/fcl_test/src/sources/End_des_EndVeclocity.txt");

    Jacobian jacobian_kdl(nj);
    Jacobian jacobian_Dot_kdl(nj);
    
    Eigen::VectorXd Error_(3);
    Error_.setZero();
   

    Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.707 * M_PI, Eigen::Vector3d::UnitX());   
    //Eigen::AngleAxisd rollAngle(0.1 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(-0.1 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
     Eigen :: MatrixXd Rotation_op_(3,3);
	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op_(i,j) = cartpos.M(i,j);
	}



    Eigen :: MatrixXd Rotation_desire = q.matrix();
    //Eigen :: MatrixXd Rotation_desire = Rotation_op_;

     Desire_position= q0;

        Eigen::VectorXd nearest_points_Robot(3);
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 125245;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
    double t_collision;






while(pp<=2000)
    {

   // robot->sense();
   // robot->getJointPosition(q0);

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
    //cout << traj.getTrajPos(t) <<endl << endl;

    Position_disire_.middleRows(i*3,3) = traj.getTrajPos(t);
    Velocity_disire_.middleRows(i*3,3) = traj.getVel(t);
    
    Pos_Vel_disire_.middleRows(i*3,3) = traj.getTrajPos(t) ;
    Pos_Vel_disire_.middleRows(3+i*3,3) = traj.getVel(t) ;   
    //Position_disire_.middleRows(i*3,3) = Position_disire;
    }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);

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


    /* MPC_OPTIMAL_.costFunction_VelocityandPos_Orientation( nj, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_op, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t); */
    
    
    
    /*MPC_OPTIMAL_.Quaternion_Cost_Function(nj, horizon, dt , 
                                          Rotation_op , angvelocity_Jacobian_end_, 
                                          Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                                          H_, g_); */
    //std::cout << H_ << std::endl;
    //std::cout << g_ << std::endl;
    
     MPC_OPTIMAL_.costFunction_VelocityandPos(nj, horizon, dt ,cartpos,
                           velocity_Jacobian_end_ , Joint_velocity_, 
                           Pos_Vel_disire_, 
                           Jacobian_4Robot_End_4velocity , JacobianDot_4Robot_End_,  
                           H_, g_, t); 

    //std::cout << H_ << std::endl;
    //std::cout << g_ << std::endl;
    //H_ = H_ + 0.0 * H_Orientation;
    //g_ = g_ + 0.0 * g_Orientation;
////////////////////////////////////////////////////////////////////////////////////////////

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

    T(0) = expection_4Obj_4Current_Read(0) + 2000;
    T(1) = expection_4Obj_4Current_Read(1) + 2000;
    T(2) = expection_4Obj_4Current_Read(2);

    //std::cout << expection_4Obj_4Current_Read << std::endl;
    //std::cout << expection_4Obj_4Current_Oreintation_Read << std::endl;

    fcl::Vector3<double> r;
    //r[0] = axis_distance_on_(0);
    //r[1] = axis_distance_on_(1);
    //r[2] = axis_distance_on_(2); 

    //r[0] = 0.10;
    //r[1] = 0.10;
    //r[2] = 0.55; 
    
    r[0] = 0.250;
    r[1] = 0.25;
    r[2] = 0.3500; 


        marker.pose.position.x = T(0) + 2000;
        marker.pose.position.y = T(1) + 2000;
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


    robot->sense();
    robot->getJointPosition(q0);
    //collision_detection.joint_states_input_ = Desire_position;
    collision_detection.joint_states_input_ = Desire_position;
    collision_detection.calFowardKinematics();
    fcl::Matrix3d R_Orientation;
    

    Eigen::Quaternion<double> q_middle;
    q_middle.x() = expection_4Obj_4Current_Oreintation_Read[0];
    q_middle.y() = expection_4Obj_4Current_Oreintation_Read[1];
    q_middle.z() = expection_4Obj_4Current_Oreintation_Read[2];
    q_middle.w() = expection_4Obj_4Current_Oreintation_Read[3];

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
    MPC_OPTIMAL_.Constraint_4robot_postion_acceleration(Number_Joint, horizon, dt, position_joint_current, velocity_inLastTIme, 
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

      double T_4Predict = (j + 2) * dt;
     

      /*MPC_OPTIMAL_.caluculate_Proper(axis_distance_on_, expection_4Obj_Pos_4Next, 
                      expection_4Obj_Pos, expection_4Obj_Vel, T_4Predict);*/


        T(0) = expection_4Obj_Pos_4Next(0) + 2000;
        T(1) = expection_4Obj_Pos_4Next(1) + 2000;
        T(2) = expection_4Obj_Pos_4Next(2);
        if(j<1)
        {
        T(0) = expection_4Obj_4Next_Read(0) + 2000;
        T(1) = expection_4Obj_4Next_Read(1) + 2000;
        T(2) = expection_4Obj_4Next_Read(2);
        q_middle.x() = expection_4Obj_4Next_Oreintation_Read[0];
        q_middle.y() = expection_4Obj_4Next_Oreintation_Read[1];
        q_middle.z() = expection_4Obj_4Next_Oreintation_Read[2];
        q_middle.w() = expection_4Obj_4Next_Oreintation_Read[3];

    //R_Orientation = fcl::Matrix3d::Identity();


        }
        else 
        {
        T(0) = expection_4Obj_4Next_Read(3) + 2000;
        T(1) = expection_4Obj_4Next_Read(4) + 2000;
        T(2) = expection_4Obj_4Next_Read(5);

        q_middle.x() = expection_4Obj_4Next_Oreintation_Read[4];
        q_middle.y() = expection_4Obj_4Next_Oreintation_Read[5];
        q_middle.z() = expection_4Obj_4Next_Oreintation_Read[6];
        q_middle.w() = expection_4Obj_4Next_Oreintation_Read[7];

        }  

      R_Orientation = q_middle.matrix();
      //std::cout << R_Orientation << std::endl;
      Nearest_Link_ = collision_detection.calMinCollisionDistance_2DynamicPoint(nearest_points, number_4Link, distance, 
                                                              T, R_Orientation, r, number_4Link_total);
      Jacobian_Nearest = MPC_OPTIMAL_.Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], Desire_position, nearest_points[0], my_tree);
      //nearest_points_Robot = nearest_points[0];
      //std::cout << "distance ==" << distance <<std::endl;
      //std::cout << "Number_Join =" << Nearest_Link_<< std::endl;
      //std::cout << "Position for obstacle" << expection_4Obj_Pos_4Next << std::endl;
      Eigen::VectorXd Position_Error_ = nearest_points[0] - nearest_points[1];
      //std::cout << Position_Error_ <<std::endl;

      MPC_OPTIMAL_.Grad_Taylor_Relax(Jacobian_Nearest.topRows(3), Position_Error_ , Joint_velocity_.topRows(Nearest_Link_), A_4distance_Constraint, B_4distance_Constraint, T_4Predict);  
      if(j <=1)
      {   
      B_4distance_Constraint_horizon(j,0) = (0.025000*(1)-0.02000000*(j)) - B_4distance_Constraint(0,0);
         
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

    qpOASES::real_t* ub_qpoases;
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
    acceleration(i) = xOpt[i]/dt - velocity_inLastTIme(i)/dt;
    velocity_inLastTIme(i) = xOpt[i];
    }

//std::cout << "velocity_inLastTIme" <<std::endl ;
//std::cout << velocity_inLastTIme <<std::endl;
    robot->sense();
    robot->getJointPosition(q0);


    for(unsigned int i=0;i<nj;i++)
    {
     jointpositions.data(i) = jointpositions.data(i)    +dt * velocity_inLastTIme(i);
     
     Desire_position(i) = jointpositions.data(i) ;
    // Desire_position(i) = q0(i) + dt* velocity_inLastTIme(i);
    
    } 



    Joint_velocity_ = velocity_inLastTIme;

    Error_Rotation_op = Rotation_desire.transpose() * Rotation_op ;
    //std::cout << "Different_"<<Rotation_desire.log() - Rotation_op.log() << std::endl;
    //std::cout << "Error_Rotation_op.log()" << std::endl << Error_Rotation_op.log() << std::endl;

    Eigen::VectorXd Desire_Rotation_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_desire.log());
    Eigen::VectorXd Rotation_op_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_op.log());

    /*if (examplefile.is_open()) {
        examplefile << cartpos.p(0) <<"\t"<<  cartpos.p(1) <<"\t" << cartpos.p(2) <<std::endl;
          }
    if (examplefile_des.is_open()) {
        examplefile_des << Position_disire_(0) <<"\t"<<  Position_disire_(1) <<"\t" << Position_disire_(2) <<std::endl;
          } */
     
       if (examplefile.is_open()) {
        examplefile << Desire_Rotation_VECTOR(0) <<"\t"<<  Desire_Rotation_VECTOR(1) <<"\t" << Desire_Rotation_VECTOR(2) <<std::endl;
          }
    if (examplefile_des.is_open()) {
        examplefile_des << Rotation_op_VECTOR(0) <<"\t"<<  Rotation_op_VECTOR(1) <<"\t" << Rotation_op_VECTOR(2) <<std::endl;
    }

    if (examplefile_des_EndVeclocity.is_open()) {
        examplefile_des_EndVeclocity << angvelocity_Jacobian_end_(0) <<"\t"<<  angvelocity_Jacobian_end_(1) <<"\t" << angvelocity_Jacobian_end_(2) <<std::endl;
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


        model.setJointPosition(Desire_position);
        model.setJointVelocity(velocity_inLastTIme);
        model.setJointAcceleration(acceleration);
        model.update();
        model.computeInverseDynamics(tau);

        control_robot_postion_test(robot ,model, Desire_position, velocity_inLastTIme, tau);
    

    ros::spinOnce();

     rate.sleep();

}


return 0; 

}
