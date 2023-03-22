#include <MPC_Optimal.h>

#include <iostream>
#include <fstream>
#include <nav_msgs/Path.h>
#include <gazebo_msgs/ModelStates.h>

//using namespace std;
//using namespace Eigen;
using namespace KDL;
using namespace std;
USING_NAMESPACE_QPOASES

void control_robot_postion_test(auto robot ,auto& model, Eigen::VectorXd desire_position, Eigen::VectorXd desire_velocity, Eigen::VectorXd tau)
{

        //model.update();
        robot->setPositionReference(desire_position);
        robot->setVelocityReference(desire_velocity);       
        robot->setEffortReference(tau);
        robot->move();

}
void matrix_to_real(qpOASES::real_t* dst, Eigen :: Matrix<double,Eigen::Dynamic,Eigen::Dynamic> src, int rows, int cols)
{
  int a = 0;
        
  for(int r = 0; r < rows; r++)
  {
    for(int c = 0; c < cols; c++)
    {
      dst[a] = (double)src(r,c);
      a++;
    }
  }

}
Eigen::VectorXd expection_4Obj_4Current_Read(3);
Eigen::VectorXd expection_4Obj_4Current_Oreintation_Read(4);

Eigen::VectorXd expection_4Obj_4Next_Read(6);
Eigen::VectorXd expection_4Obj_4Next_Oreintation_Read(8);




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

    


int main(int argc, char **argv)
{

    ros::init(argc, argv, "MPC_Center"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle n;
    ros::NodeHandle n_;
    ros::NodeHandle nh;


    ros::Subscriber sub = nh.subscribe("/xbotcore/joint_states", 1000, &poseCallback);

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
    Eigen::VectorXd q0;
    q0 =Position_Joint;
     

/////////////////////////////////////////////////////////////////////////////////////////////////

    std::vector<std::string> arm_rotation_joints;
    ArmCollision collision_detection(my_tree, arm_param_config, arm_rotation_joints);



////////////////////////////////////////////////////////////////////


    traj traj;
    traj.dt = dt;
    traj._Acc = 0.5;
    traj._Vel = 2.5;
    traj._dev_order = 3.0;
    traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
    traj.sphere_visual_ = n.advertise<visualization_msgs::Marker>("sphere",1,true);
    traj.val_visual_ = n.advertise<visualization_msgs::MarkerArray>("ProperVolume",1,true);
    traj.sphere_visual_Random = n.advertise<visualization_msgs::MarkerArray>("ProperVolume_random",1,true);
    //traj.sphere_visual_Random = n.advertise<visualization_msgs::Marker>("ProperVolume_random",1,true);

    int horizon = 5;

    Chain chain;
    my_tree.getChain("torso_2","ball1_tip",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);

    unsigned int nj = chain.getNrOfJoints();

    string name_4Link[nj+1];

    // cout<<"nj="<<nj<<endl;
	
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

    
     q0 = Position_Joint;

    Wait=0;
    while(Wait <= 10)
    {
        Wait ++ ;
        
        std::cout << Wait <<std::endl;

        MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub, q0);
        ros::spinOnce();
        rate.sleep();

    }

    q0 = Position_Joint;

     //return 0;
    jointpositions_vector_ = Position_Joint;
    // jointpositions_vector = Position_Joint;
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
    // std::cout << "arm_link_names_[]=" << arm_link_names_[i] << std::endl; 
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

    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);
    

	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
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
    double pos_des[3];
    Eigen::Vector3d p;
    p[0] = cartpos.p(0);
    p[1] = cartpos.p(1);
   // p[1] = cartpos.p(1);
    p[2] = cartpos.p(2);

    p_list.push_back(p);

    pos_des[0] = cartpos.p(0) + 0.0;
    pos_des[1] = cartpos.p(1) + 0.2;
    pos_des[2] = cartpos.p(2) + 0.2;

    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];

    p_list.push_back(p);

    pos_des[0] = cartpos.p(0) + 0.0;
    pos_des[1] = cartpos.p(1) + 0.4;
    pos_des[2] = cartpos.p(2) + 0.0;

    p[0] = pos_des[0];
    p[1] = pos_des[1];
    p[2] = pos_des[2];

    p_list.push_back(p);

    Eigen::Vector3d v;
    v.setZero();
   

    bool success = traj.trajGeneration(p_list,v);
     std::cout << "success" << success << std::endl;




    Eigen::VectorXd _Rotation_op_vector(3);
    Eigen::VectorXd _Rotation_des_vector(3);
    Eigen::MatrixXd _Rotation_Error_(3,3);

    Eigen::MatrixXd _Rotation_op_log(3,3);
    Eigen::MatrixXd _Rotation_des_log(3,3);

    //Eigen::AngleAxisd rollAngle(0.333 * M_PI, Eigen::Vector3d::UnitZ());
    //Eigen::AngleAxisd yawAngle(0.707 * M_PI, Eigen::Vector3d::UnitY());
    //Eigen::AngleAxisd pitchAngle(0.707 * M_PI, Eigen::Vector3d::UnitX());   
    Eigen::AngleAxisd rollAngle(0.8 * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(0.8 * M_PI, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(0.0 * M_PI, Eigen::Vector3d::UnitX()); 

    Eigen::Quaternion<double> q = rollAngle * yawAngle * pitchAngle;

    Eigen::MatrixXd Rotation_desire = q.matrix();
    
    _Rotation_Error_ = (Rotation_op.transpose() * Rotation_desire).log();
    _Rotation_op_log = Rotation_op;
    Eigen::MatrixXd Rotation_desire_record = Rotation_desire;
    //std::cout << Rotation_desire <<std::endl;




    Eigen::MatrixXd Current_Orientation = Rotation_op;

    t = 0;

    ofstream examplefile;   
    ofstream examplefile_des;   
    ofstream examplefile_des_EndVeclocity;   

    Jacobian jacobian_kdl(nj);
    Jacobian jacobian_Dot_kdl(nj);
    
    Eigen::VectorXd Error_(3);
    Error_.setZero();
   


     Desire_position= Position_Joint;

        Eigen::VectorXd nearest_points_Robot(3);
        ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker",1);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "torso_2";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 125245;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        
   // double t_collision;

    std::cout << Position_Joint << std::endl;
    Eigen::MatrixXd _Matrix_FirstHierachy;


double t_orientation = 0;

while(pp<=20)
    {

   // robot->sense();
   // robot->getJointPosition(q0);
   // std::cout << Rotation_desire <<std::endl;

    
    if (t_orientation <=1)
    {
    t_orientation = t_orientation + (pp+1)*dt;
    }

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
    

	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}



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

    
    Rotation_desire =  _Rotation_op_log * (t_orientation * _Rotation_Error_).exp() ;

    


     // std::cout << "Error_trajectory=" <<Rotation_desire_record.log() - Rotation_desire.log() << std::endl;
     
    //Rotation_desire = _Rotation_des_log.exp();

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
  //  t_collision = dt * pp + dt;
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

    T(0) = 0.6;
    T(0) = 0.6;
    T(0) = 0.4;

    
    //std::cout << expection_4Obj_4Current_Read << std::endl;
    //std::cout << expection_4Obj_4Current_Oreintation_Read << std::endl;

    fcl::Vector3<double> r;

    r[0] = 0.1;
    r[1] = 0.1;
    r[2] = 0.1; 


        marker.pose.position.x = T(0);
        marker.pose.position.y = T(1);
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
    collision_detection.joint_states_input_ = Desire_position;
    collision_detection.calFowardKinematics();
    fcl::Matrix3d R_Orientation;
    Eigen::MatrixXd wholearm_Jacobian;
    wholearm_Jacobian = Jacobian_4Robot_End_4velocity;
    wholearm_Jacobian.setZero();

    Eigen::Quaternion<double> q_middle;
    q_middle.x() = expection_4Obj_4Current_Oreintation_Read[0];
    q_middle.y() = expection_4Obj_4Current_Oreintation_Read[1];
    q_middle.z() = expection_4Obj_4Current_Oreintation_Read[2];
    q_middle.w() = expection_4Obj_4Current_Oreintation_Read[3];

    //R_Orientation = fcl::Matrix3d::Identity();
    R_Orientation = q_middle.matrix();
    
    Nearest_Link_ = collision_detection.calMinCollisionDistance_2DynamicPoint(nearest_points, number_4Link, distance, 
                                                              T, R_Orientation, r, number_4Link_total);   
    std::cout << "Nearest_Link_" <<Nearest_Link_ << std::endl;
    std::cout << "distance" <<distance << std::endl;
    Jacobian_Nearest = MPC_OPTIMAL_.Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], Desire_position, nearest_points[0], my_tree);
    std::cout << Jacobian_Nearest << std::endl;




    wholearm_Jacobian.block(0,0,
                            3, Jacobian_Nearest.cols()) = Jacobian_Nearest.topRows(3);
    Eigen::VectorXd Position_Error_ = nearest_points[0] - nearest_points[1];


    Eigen::MatrixXd Eigen_4Constraint_A = Eigen::MatrixXd :: Zero(2*Jacobian_4Robot_End_4velocity.rows(), 
                                                                    Jacobian_4Robot_End_4velocity.cols()+3);
    Eigen_4Constraint_A.block(0,0, Jacobian_4Robot_End_4velocity.rows(),Jacobian_4Robot_End_4velocity.cols()) 
                                        = Jacobian_4Robot_End_4velocity;
    Eigen_4Constraint_A.block(0,0, 3,3) = Eigen::MatrixXd::Identity(3,3);
    Eigen_4Constraint_A.block(Jacobian_4Robot_End_4velocity.rows(),0,
                              wholearm_Jacobian.rows(),wholearm_Jacobian.cols()) = wholearm_Jacobian;
    Eigen::VectorXd lb_qpoases_(6);
    lb_qpoases_.setZero();
    lb_qpoases_(3) = -0.050 - Position_Error_(0);
    lb_qpoases_(4) = -0.050 - Position_Error_(1);
    lb_qpoases_(5) = -0.050 - Position_Error_(2);

    Eigen::VectorXd ub_qpoases_(6);
    ub_qpoases_.setZero();
    ub_qpoases_(3) = 0.050 - Position_Error_(0);;
    ub_qpoases_(4) = 0.050 - Position_Error_(1);; 
    ub_qpoases_(5) = 0.050 - Position_Error_(2);; 


    Eigen::VectorXd Vector_Constraint_Aconstraint_lower(3+Number_Joint);
    Vector_Constraint_Aconstraint_lower.setZero();
    for(int i=0;i<Number_Joint;i++)
    {
        Vector_Constraint_Aconstraint_lower(i) = -0.0025;
    }
    Vector_Constraint_Aconstraint_lower(7) = -0.005;
    Vector_Constraint_Aconstraint_lower(8) = -0.005;
    Vector_Constraint_Aconstraint_lower(9) = -0.005;

    Eigen::VectorXd Vector_Constraint_Aconstraint_upper;
    Vector_Constraint_Aconstraint_upper = - Vector_Constraint_Aconstraint_lower;
    
    Eigen::VectorXd g_online(3+Number_Joint);
    g_online.setZero();
    
    Eigen::MatrixXd H_online; 
    H_online = 10 * Eigen::MatrixXd::Identity(3+Number_Joint,3+Number_Joint); 
    H_online.block(0,0,Number_Joint,Number_Joint) = 0.00*Eigen::MatrixXd::Identity(Number_Joint,Number_Joint);
    

   



    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_online.rows()* H_online.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(H_qpoases, H_online, H_online.rows(), H_online.cols());

	  qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_online;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
  	matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);

    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = Vector_Constraint_Aconstraint_lower;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = Vector_Constraint_Aconstraint_upper;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = lb_qpoases_;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix =  ub_qpoases_;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper_matrix.rows() , 1);




	QProblem example( Number_Joint+3, Vector_Constraint_Aconstraint_lower_matrix.rows() );
    //QProblemB example( Number_Joint+3 );

	Options options;

	 //options.initialStatusBounds = ST_INACTIVE;
	 //options.numRefinementSteps = 1000000;
	 //options.enableCholeskyRefactorisation = 1;   
	
    //Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 1000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);
    //example.init( H_qpoases,g_qpoases,lb_qpoases,ub_qpoases,nWSR, 0);
	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[(Number_Joint+Number_Joint+ Number_Joint)*horizon];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );


	printf( "\nxOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );

   
    // std::cout << wholearm_Jacobian <<std::endl;                                                         
  /*  if(pp>=5)
    {   
    if(collision_detection.getCollisionStatus() == 2)
    {
    cout<< pp << " = <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< = " << collision_detection.getCollisionStatus() << endl;
    return 0;
    }
    }
*/
    //std::cout <<"name_4Link[Nearest_Link_]: "<< name_4Link[Nearest_Link_] <<"Position_Error_=" << Position_Error_ << std::endl << "Distance = "<< Position_Error_.norm()<<std:: endl;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



    /* Eigen:: MatrixXd A_4distance_Constraint_horizon(horizon, Number_Joint*horizon);
    A_4distance_Constraint_horizon.setZero();
    Eigen:: MatrixXd B_4distance_Constraint_horizon(horizon,1);
    B_4distance_Constraint_horizon.setZero();
    Eigen:: MatrixXd B_4distance_Constraint_horizon_upper(horizon,1);   
    for(int i=0; i< horizon;i++)
    {
    B_4distance_Constraint_horizon_upper(i,0) = 100000000 ;
    }

    for(int j=0; j<horizon; j++)
    {

      double T_4Predict = (j + 2) * dt;
     
    

      Nearest_Link_ = collision_detection.calMinCollisionDistance_2DynamicPoint(nearest_points, number_4Link, distance, 
                                                                                T, R_Orientation, r, number_4Link_total);

      Jacobian_Nearest = MPC_OPTIMAL_.Get_Jacobian_4Point(Nearest_Link_, name_4Link[Nearest_Link_], Desire_position, nearest_points[0], my_tree);

      Eigen::VectorXd Position_Error_ = nearest_points[0] - nearest_points[1];
   

      MPC_OPTIMAL_.Grad_Taylor_Relax(Jacobian_Nearest.topRows(3), Position_Error_ , 
                                     Joint_velocity_.topRows(Nearest_Link_), A_4distance_Constraint, B_4distance_Constraint, T_4Predict);  

      if(j <=2)
      {   
      B_4distance_Constraint_horizon(j,0) = (0.02000*(1)-0.015000000*(j)) - B_4distance_Constraint(0,0);
         
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
    }*/


     Eigen::VectorXd Slack_Vector(6);
     Slack_Vector.setZero();
     //std::cout << Slack_Vector <<std::endl;

   // MPC_OPTIMAL_.Orientation_Trajectory_Error(Rotation_desire, Rotation_op, Rotation_Error);

     Slack_Vector(0) = 1;
     Slack_Vector(1) = 1;
     Slack_Vector(2) = 1;
     Slack_Vector(3) = 0;
     Slack_Vector(4) = 0;
     Slack_Vector(5) = 0;




      MPC_OPTIMAL_.Orientation_Trajectory_Generation(t_orientation , Rotation_op, 
                                                   _Rotation_Error_, Rotation_desire);

    
     Eigen::VectorXd result_jointVel(Number_Joint);
     /* MPC_OPTIMAL_.optimal_online(Jacobian_4Robot_End, JacobianDot_4Robot_End_, 
                                cartpos, Pos_Vel_disire_.topRows(6*horizon),
                                Rotation_desire ,Rotation_op,
                                Joint_velocity_ ,  jointpositions_vector, 
                                dt, horizon, Number_Joint, t,Slack_Vector,
                                 result_jointVel, 1); 
     */

    for(int i=0;i<Number_Joint;i++)
    {
    result_jointVel(i) =  xOpt[i];
    }

    for(unsigned int i=0;i<nj;i++)
    {
    // jointpositions.data(i) = jointpositions.data(i)    + dt * result_jointVel(i);
    jointpositions.data(i) = jointpositions.data(i)    +   result_jointVel(i);
     Desire_position(i) = jointpositions.data(i) ;
     //Desire_position(i) = q0(i) + dt* velocity_inLastTIme(i);
    
    } 



    Joint_velocity_ = velocity_inLastTIme;

    Error_Rotation_op = Rotation_desire.transpose() * Rotation_op ;
    std::cout << "Different_"<<Rotation_desire.log() - Rotation_op.log() << std::endl;
    
    //std::cout << "Error_Rotation_op.log()" << std::endl << Error_Rotation_op << std::endl;

    Eigen::VectorXd Desire_Rotation_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_desire.log());
    Eigen::VectorXd Rotation_op_VECTOR = MPC_OPTIMAL_.UnHatt_matrix(Rotation_op.log());

    /*if (examplefile.is_open()) {
        examplefile << cartpos.p(0) <<"\t"<<  cartpos.p(1) <<"\t" << cartpos.p(2) <<std::endl;
          }
    if (examplefile_des.is_open()) {
        examplefile_des << Position_disire_(0) <<"\t"<<  Position_disire_(1) <<"\t" << Position_disire_(2) <<std::endl;
          } */
     


     for(int i = 0; i<3 ;i++)
     {
     Error_(i) = cartpos.p(i) - Position_disire_(i);
     //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
          // cout << "Error_ = " << Error_.norm() << endl;
     }

      cout << "Error_ = " << xOpt[7] << "," << xOpt[8]<<"," << xOpt[9] << endl;
     //std::cout << "velocity_inLastTIme*dt=" << velocity_inLastTIme*dt <<std::endl ;    
     //std::cout << "q0=" << q0 <<std::endl ;    
     //std::cout << "Jacobian * velocity_inLastTIme *dt =" << Jacobian_4Robot_End_4velocity*velocity_inLastTIme *dt<<std::endl ;    




     pp++;
     cout << pp <<endl;
     //std::cout << "Desire_position ==" << Desire_position <<std::endl;
     //cout << "Desire_position=" << Desire_position <<endl;
     //std::cout << "cartpos ==" << cartpos <<std::endl;

    MPC_OPTIMAL_.CenControl_Pub_Desire_(Cen_joint_cmd_pub,Desire_position);

    ros::spinOnce();

     rate.sleep();

}


return 0; 

}
