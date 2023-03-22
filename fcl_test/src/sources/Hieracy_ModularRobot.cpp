#include "Hieracy_ModularRobot.h"


using namespace KDL;
USING_NAMESPACE_QPOASES

SECOND_HIER::SECOND_HIER()
{

}

SECOND_HIER::~SECOND_HIER()
{

}



void SECOND_HIER::Second_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    Eigen::MatrixXd Rotation_desire ,Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    Eigen::MatrixXd Jacobian_input, Eigen::MatrixXd Desire_velocity, 
                    double dt, double horizon, int Number_Joint,
                    Eigen::MatrixXd& acceleration, Eigen::MatrixXd& velocity_inLastTIme)
{

    Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);
    velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);

	for(int i=0; i<3; i++) {
		for(int j=0; j<3; j++)
			Rotation_op(i,j) = cartpos.M(i,j);
	}


   Eigen::VectorXd position_joint_current(Number_Joint*horizon);

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }

   //std::cout << "<<<<<<<<<<<<<<" <<std::endl;
    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 
 
    MPC_OPTIMAL_.ConstructCost_Orientation(Number_Joint, horizon, dt , 
                                          Rotation_op , angvelocity_Jacobian_end_, 
                                          Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                                          H_, g_); 


    /* MPC_OPTIMAL_.costFunction_VelocityandPos_Orientation( number_joint, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_op, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t); */
    
    
    
    /*MPC_OPTIMAL_.Quaternion_Cost_Function(number_joint, horizon, dt , 
                                          Rotation_op , angvelocity_Jacobian_end_, 
                                          Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                                          H_, g_); */

    
    /* MPC_OPTIMAL_.costFunction_VelocityandPos_ModularRobot(number_joint, horizon, dt ,cartpos,
                           velocity_Jacobian_end_ , Joint_velocity_, 
                           Pos_Vel_disire_, 
                           Jacobian_4Robot_End_4velocity , JacobianDot_4Robot_End_,  
                           H_, g_, t); */

////////////////////////////////////////////////////////////////////////////////////////////

 

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
    MPC_OPTIMAL_.Matrix_Constraint_4acc_pos_acc_ModularRobot(Number_Joint, horizon, dt, Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon + Jacobian_input.rows(), Number_Joint * horizon);  
    int NumberConstraint = Number_Joint * horizon;

    Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;
    Eigen_4Constraint_A.bottomLeftCorner(Jacobian_input.rows(),Number_Joint) = Jacobian_input;


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon );  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon );  

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 
   // Vector_Constraint_Aconstraint_lower.bottomRows(NumberConstraint) = Vector_Constraint_4acc_lower;
    Vector_Constraint_Aconstraint_lower.bottomRows(Jacobian_input.rows()) = Desire_velocity; 

    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;
    Vector_Constraint_Aconstraint_upper.bottomRows(Jacobian_input.rows()) = Desire_velocity;

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



	QProblem example( horizon*Number_Joint, horizon*Number_Joint + Jacobian_input.rows());

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 1000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[(Number_Joint)*horizon + Jacobian_input.rows()];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );


	printf( "\nxOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal());
     

    for(int i=0;i<Number_Joint;i++)
    {
    acceleration(i) = xOpt[i]/dt - Joint_velocity_(i)/dt;
    velocity_inLastTIme(i) = xOpt[i];
    }



}
