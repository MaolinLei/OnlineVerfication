#ifndef _MPCHIR_H_
#define _MPCHIR_H_
#include "MPC_Optimal.h"





class SECOND_HIER{

private:
 // Eigen::MatrixXd Transform_Skew_Matrix(9,3);
 // Transform_Skew_Matrix << 0, 0, 0, 0, 0, 1, 0,-1, 0,
  //                       0, 0,-1, 0, 0, 0, 1, 0, 0,
  //                       0, 1, 0,-1, 0, 0, 0, 0, 0;

public:

SECOND_HIER();
~SECOND_HIER();

void Second_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    Eigen::MatrixXd Rotation_desire ,Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    Eigen::MatrixXd Jacobian_input, Eigen::MatrixXd Desire_velocity, 
                    double dt, double horizon, int number_joint,
                    Eigen::MatrixXd& acceleration, Eigen::MatrixXd& velocity_inLastTIme);

};
#endif