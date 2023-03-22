#include "Training_MPC.h"

using namespace KDL;

Training_MPC::Training_MPC()
{

}

Training_MPC::~Training_MPC()
{

}


void MPC_function_define(string Load_path_, Eigen::Vector3d Desired_pos)
{
    my_tree.getChain("base_link","TCP_gripper_A",chain);
    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    unsigned int Number_Joint = chain.getNrOfJoints();

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

    Jacobian jacobian_kdl(Number_Joint);
    Jacobian jacobian_Dot_kdl(Number_Joint);


}



void MPC_function_trajectory(Eigen::Vector3d Desired_pos)
{
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

    p[0] = Desired_pos(0);
    p[1] = Desired_pos(1);
   // p[1] = cartpos.p(1);
    p[2] = Desired_pos(2);
    p_list.push_back(p); 


    traj.trajGeneration(p_list,v);

}
