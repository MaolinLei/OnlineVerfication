#ifndef _IKopt_H_
#define _IKopt_H_

// #include "ros/ros.h"
// #include <ros/package.h>

#include "yaml-cpp/yaml.h"


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

#include <RobotInterfaceROS/ConfigFromParam.h> 

#include <unsupported/Eigen/MatrixFunctions>

#include <Eigen/Dense>

#include<python3.8/Python.h>

USING_NAMESPACE_QPOASES

class inverser_kinematical{

private:
 // Eigen::MatrixXd Transform_Skew_Matrix(9,3);
 // Transform_Skew_Matrix << 0, 0, 0, 0, 0, 1, 0,-1, 0,
  //                       0, 0,-1, 0, 0, 0, 1, 0, 0,
  //                       0, 1, 0,-1, 0, 0, 0, 0, 0;

public:


public:

inverser_kinematical();
~inverser_kinematical();

void matrix_to_real(qpOASES::real_t* dst, Eigen :: Matrix<double,Eigen::Dynamic,Eigen::Dynamic> src, int rows, int cols);
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
