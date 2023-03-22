#ifndef _MPCopt_H_
#define _MPCopt_H_

#include "ros/ros.h"
#include <ros/package.h>

#include "yaml-cpp/yaml.h"
#include "arm_collision.h"

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>

#include <qpOASES.hpp>

//traj_generate
#include <traj.h>

#include <vector>
#include <algorithm>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h> 

#include <unsupported/Eigen/MatrixFunctions>

#include <Eigen/Dense>
#include <xbot_msgs/JointCommand.h>
#include <xbot_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>


USING_NAMESPACE_QPOASES

class MPC_OPTIMAL{

private:
 // Eigen::MatrixXd Transform_Skew_Matrix(9,3);
 // Transform_Skew_Matrix << 0, 0, 0, 0, 0, 1, 0,-1, 0,
  //                       0, 0,-1, 0, 0, 0, 1, 0, 0,
  //                       0, 1, 0,-1, 0, 0, 0, 0, 0;

public:

MPC_OPTIMAL();
~MPC_OPTIMAL();

double cost_int_;

double cost_function(double distance_cost, double MPC_cost);

ros::Publisher init_Pub_Control_Node(ros::NodeHandle nh_);

void matrix_to_real(qpOASES::real_t* dst, Eigen :: Matrix<double,Eigen::Dynamic,Eigen::Dynamic> src, int rows, int cols);

void Grad_Taylor_Relax(Eigen::MatrixXd Jacobian_collision_Point, Eigen::VectorXd Position_Error_ , Eigen::VectorXd Initial_joint_Vel, Eigen::MatrixXd& A_4distance_Constraint, Eigen::MatrixXd& B_4distance_Constraint, double dt);


void costFunction_Velocity(int Number_Joint, int horizon, double dt, KDL::Frame cartpos ,
                           Eigen::VectorXd Position_disire_ ,Eigen::MatrixXd Jacobian_4Robot_End_4velocity ,
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_);

void costFunction_VelocityandPos_ModularRobot_MPCLearning (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity ,
                           Eigen::VectorXd PositionVelocity_disire_, Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector);

void costFunction_VelocityandPos_Orientation_ModularRobot_MPClearning (int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector);



void Constraint_4robot_postion_acceleration(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower);

void Constraint_4robot_postion_acceleration_ModularRobot(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower);                       

void Constraint_4robot_postion_acceleration_ModularRobot_MPCLearning(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower);



void Matrix_Constraint_4acc_pos_acc_ModularRobot(int Number_Joint, int horizon, double dt, Eigen::MatrixXd& Matrix_Constraint_4pos_, Eigen::MatrixXd& Matrix_Constraint_4acc_);

void Matrix_Constraint_4acc_pos_acc(int Number_Joint, int horizon, double dt, Eigen::MatrixXd& Matrix_Constraint_4pos_, Eigen::MatrixXd& Matrix_Constraint_4acc_);

void Constraint_4robot_postion_acceleration_Centaruo(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower);


void caluculate_Proper(Eigen::VectorXd& axis_distance_on_, Eigen::VectorXd& expection_4Obj_Pos_4Next, 
                       Eigen::VectorXd expection_4Obj_Pos, Eigen::VectorXd expection_4Obj_Vel, double dt);


void Quaternion_Cost_Function(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation);

void ConstructCost_Orientation(int Number_Joint, int horizon, double dt, 
                               Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                               Eigen::MatrixXd Rotation_desire,
                               Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                               Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation);


void ConstructCost_Orientation_ModularRobot(int Number_Joint, int horizon, double dt, 
                               Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                               Eigen::MatrixXd Rotation_desire,
                               Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                               Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation,Eigen::VectorXd Slack_Vector);

void costFunction_VelocityandPos_ModularRobot(int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity, 
                           Eigen::VectorXd PositionVelocity_disire_,
                           Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector);

void costFunction_VelocityandPos(int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity, 
                           Eigen::VectorXd PositionVelocity_disire_,
                           Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time);

void costFunction_VelocityandPos_Orientation (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time);

void costFunction_VelocityandPos_Orientation_ModularRobot (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector);

void Quaternion_Cost_Function_ModularRobot(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation, Eigen::VectorXd Slack_Vector);

void costFunction_VelocityandPos_Orientation_ModularRobotCompare (int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector);


void ConstructCost_StateSpace_Orientation(Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op, Eigen::MatrixXd Rotation_desire, double dt, 
                                          Eigen::MatrixXd& C_eta, Eigen::MatrixXd& C_w, Eigen::MatrixXd& C_c, Eigen::VectorXd& Error_with_Orientation_vector);

void Error_Orientation_Vector( Eigen::MatrixXd Rotation_op, Eigen::MatrixXd Rotation_desire,Eigen::VectorXd& Error_with_Orientation_vector);

void Cost_Minimum_Jerk(int Number_Joint, int horizon, Eigen::VectorXd Current_Joint_Velcity,
                                   Eigen::MatrixXd& H_Matrix_Jerk, Eigen::MatrixXd& g_Matrix_Jerk);

void CenControl_Pub_Desire_left(ros::Publisher head_camera_joint_cmd_pub, Eigen::VectorXd position);
void CenControl_Pub_Desire_right(ros::Publisher head_camera_joint_cmd_pub, Eigen::VectorXd position);

void Second_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    Eigen::MatrixXd Jacobian_input, Eigen::MatrixXd Desire_velocity, 
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::VectorXd& result_jointVel, int orientation);

void First_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::MatrixXd& Desire_Matrix, Eigen::VectorXd& First_Result, int orientation);

void optimal_online(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t,
                    Eigen::VectorXd& First_Result);

void MPC_Learning(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::MatrixXd& Desire_Matrix, Eigen::VectorXd& First_Result, int orientation);

void Relatvie_Orientation(Eigen::VectorXd current_dir_frameOri, Eigen::MatrixXd& Rotation_matrix);
void QP_online_DualArm(int Nearest_Link_left, int Nearest_Link_right, 
                                    std::string left_Arm_Link,  std::string right_Arm_Link,
                                    Eigen::VectorXd Position_left, Eigen::VectorXd Position_right,
                                    Eigen::MatrixXd Jacobian_left, Eigen::MatrixXd Jacobian_right,
                                    Eigen::VectorXd Joint_Position_current_left, Eigen::VectorXd Joint_Position_current_right, 
                                    Eigen::VectorXd Joint_Velocity_current_left, Eigen::VectorXd Joint_Velocity_current_right, 
                                    Eigen::Vector3d nearest_points_left, Eigen::Vector3d nearest_points_right,
                                    Eigen::VectorXd position_desired_left, Eigen::VectorXd position_desired_right,
                                    KDL::Tree _tree, double distance,
                                    Eigen::VectorXd& Changing_Position);
// void optimal_online(Eigen::VectorXd Jacobian_4Robot_End, Eigen::MatrixXd Jacobian_collision,  
//                                 Eigen::VectorXd jointvelocity);
void Constraint_dualarm_posandvel(Eigen::VectorXd Vector_Current_4pos_left_right, 
                                 Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                 Eigen::VectorXd& Vector_Constraint_4vel_upper, Eigen::VectorXd& Vector_Constraint_4vel_lower);

void Orientation_Trajectory_Generation(double t_segment , Eigen::MatrixXd init_Orientation, 
                                       Eigen::MatrixXd _Rotation_Error_, Eigen::MatrixXd &Desire_Oritation);

void Orientation_Trajectory_Error(Eigen::MatrixXd Desire_Oritation, Eigen::MatrixXd init_Orientation, 
                                  Eigen::MatrixXd& _Rotation_Error_); 

Eigen::Matrix3d Tramsform_axis_2rotationMarix(double x, double y, double z, double angle);

Eigen::MatrixXd Skew_symmetric_matrix(Eigen::VectorXd Position_Error_ );

Eigen::MatrixXd Grad_Taylor_Relax_dualarm(int Nearest_Link_left, int Nearest_Link_right, 
                                                      std::string left_Arm_Link,  std::string right_Arm_Link,
                                                      Eigen::MatrixXd Jacobian_left, Eigen::MatrixXd Jacobian_right,
                                                      Eigen::VectorXd Joint_Position_current_left, Eigen::VectorXd Joint_Position_current_right, 
                                                      Eigen::Vector3d nearest_points_left, Eigen::Vector3d nearest_points_right,
                                                      KDL::Tree _tree, double distance,
                                                      double& Max_distance_);

Eigen::MatrixXd Get_Jacobian_4Point(int number_Link, std::string Link_Name, Eigen::VectorXd jointpositions_ ,Eigen::Vector3d Position_NearPoint, KDL::Tree _tree);

Eigen::MatrixXd KroneckerProductSparse(Eigen::MatrixXd I , Eigen::MatrixXd X );

Eigen::VectorXd UnHatt_matrix(Eigen::MatrixXd Eigen_Skew_symmetric);

Eigen::MatrixXd ErrorWithOrientation(Eigen::MatrixXd Desire_Orientation, Eigen::MatrixXd Current_Orientation);

Eigen::MatrixXd Calulated_pseudo_inverse_left(Eigen::MatrixXd Pesudo_matrix);

Eigen::MatrixXd Calculated_inverse_with_N();

Eigen::VectorXd Vecterization(Eigen::MatrixXd Matrix_Need_Vection);


Eigen::VectorXd solvIK(Eigen::VectorXd Position_disire_, std::string path_urdf, Eigen::VectorXd& First_Result);
void IK_solution(Eigen::MatrixXd Jacobian_4Robot_End, 
                    KDL::Frame cartpos, Eigen::VectorXd Position_disire_,
                    Eigen::VectorXd Joint_Position_current,
                    int Number_Joint, 
                    Eigen::VectorXd& First_Result);
void Constraint_4modular_IK(int Number_Joint, Eigen::VectorXd position_joint_current, 
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint);

void costFunction_Velocity_IK_solve(int Number_Joint,  Eigen::VectorXd current_end_position ,
                           Eigen::VectorXd Position_disire_ ,Eigen::MatrixXd Jacobian_4Robot_End_4velocity ,
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_);


};
#endif