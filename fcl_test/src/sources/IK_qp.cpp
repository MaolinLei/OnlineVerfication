#include "IK_qp.h"

using namespace KDL;
USING_NAMESPACE_QPOASES

inverser_kinematical::inverser_kinematical()
{

}

inverser_kinematical::~inverser_kinematical()
{

}



void inverser_kinematical::matrix_to_real(qpOASES::real_t* dst, Eigen :: Matrix<double,Eigen::Dynamic,Eigen::Dynamic> src, int rows, int cols)
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


void inverser_kinematical:: costFunction_Velocity_IK_solve(int Number_Joint,  Eigen::VectorXd current_end_position ,
                           Eigen::VectorXd Position_disire_ ,Eigen::MatrixXd Jacobian_4Robot_End_4velocity ,
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_)
{
     Eigen::MatrixXd H_QP_4Kin_Center;
     H_QP_4Kin_Center = Jacobian_4Robot_End_4velocity ;
    // H_QP_4Kin = Jacobian_4Robot_End_4velocity.transpose() * Jacobian_4Robot_End_4velocity *dt*dt;

  //  std::cout << "Jacobian_4Robot_End_4velocity *dt = " << std::endl << Jacobian_4Robot_End_4velocity *dt << std::endl;

    Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity( 3 , 3);

    Eigen :: MatrixXd H_QP_4Kin;
    H_QP_4Kin = H_QP_4Kin_Center.transpose() *Q_MaritxXd_Weight * H_QP_4Kin_Center ;

    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint,Number_Joint);
    //cout << H_QP_4Kin <<endl <<endl;
    // H_QP_4Norm.setZero();
    H_ = 2*1*H_QP_4Kin + 0.0005 * H_QP_4Norm;

    Eigen::VectorXd Curre_End(3);
    Eigen::VectorXd Diff_DesAndCur(3);

    Curre_End = current_end_position;

    Diff_DesAndCur = 0.75 * (Curre_End - Position_disire_);
    // std::cout << "Diff_DesAndCur = " <<Diff_DesAndCur << std::endl;

    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
    g_ = 2 * H_QP_4Kin_Center.transpose()*Q_MaritxXd_Weight * Diff_DesAndCur;
    
}


void inverser_kinematical::Constraint_4modular_IK(int Number_Joint, Eigen::VectorXd position_joint_current, 
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint)
{
   Eigen::VectorXd joint_poistion_upper(Number_Joint);
   Eigen::VectorXd joint_poistion_lower(Number_Joint);
   joint_poistion_upper.setZero();
   joint_poistion_lower.setZero();

   for(int i = 0 ;i < Number_Joint; i++)
   {
     joint_poistion_upper(i) = 2.000;
     joint_poistion_lower(i) = -2.000;
   }

   upper_velocity_joint = joint_poistion_upper - position_joint_current;
   lower_velocity_joint = joint_poistion_lower - position_joint_current;

}



void inverser_kinematical::IK_solution(Eigen::MatrixXd Jacobian_4Robot_End, 
                    KDL::Frame cartpos, Eigen::VectorXd Position_disire_,
                    Eigen::VectorXd Joint_Position_current,
                    int Number_Joint, 
                    Eigen::VectorXd& First_Result)
  {

    Eigen::MatrixXd Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Eigen::MatrixXd Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);
    
    Eigen::VectorXd Joint_position_result(Number_Joint); 
    First_Result = Joint_position_result.setZero();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    // Eigen::MatrixXd Rotation_op(3,3);

    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 

   
  // Desire_Matrix = Eigen::MatrixXd::Zero(number_slack,Jacobian_4Robot_End.cols());

    Eigen::VectorXd Curre_End(3);

    Curre_End(0) = cartpos.p(0);
    Curre_End(1) = cartpos.p(1);
    Curre_End(2) = cartpos.p(2);


    costFunction_Velocity_IK_solve(Number_Joint, Curre_End ,
                                   Position_disire_ ,Jacobian_4Robot_End_4velocity,
                                   H_, g_);

    // std::cout << H_ <<std::endl;
    // std::cout << g_ <<std::endl;

    Eigen:: VectorXd upper_velocity_joint(Number_Joint); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint); 

    Constraint_4modular_IK(Number_Joint,  Joint_Position_current,
                           upper_velocity_joint, lower_velocity_joint );
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   // std::cout << upper_velocity_joint <<std::endl;
   // std::cout << lower_velocity_joint <<std::endl;

   // Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon , Number_Joint * horizon);  
    int NumberConstraint = Number_Joint;
   // Eigen_4Constraint_A.setZero();
   // Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;


    // Matrix_Constraint_4pos_.setZero();
////////////////////////////////////////
	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());

	  qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
  	matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);


   // std::cout << "Matrix_Constraint_4pos_ = " << std::endl << Matrix_Constraint_4pos_ << std::endl;
   // std::cout << "Vector_Constraint_Aconstraint_lower_matrix = " << std::endl << Vector_Constraint_Aconstraint_lower_matrix << std::endl;
   // std::cout << "Vector_Constraint_Aconstraint_upper_matrix = " << std::endl << Vector_Constraint_Aconstraint_upper_matrix << std::endl;

		// QProblem example( horizon*Number_Joint, horizon*Number_Joint );

	  QProblemB example( Number_Joint );

	  Options options;
	  //options.initialStatusBounds = ST_INACTIVE;
	  //options.numRefinementSteps = 100000;
	  //options.enableCholeskyRefactorisation = 1;
    example.setOptions( options );
   // example.setPrintLevel( PL_NONE ); 
	/* Solve first QP. */
	 int_t nWSR = 100;
   real_t cputime = 1.0;
	 // example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);
   // example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases, lbA_qpoases, ubA_qpoases, nWSR, &cputime);
   example.init( H_qpoases,g_qpoases,lb_qpoases,ub_qpoases,nWSR, &cputime);

	 /* Get and print solution of first QP. */
	 real_t xOpt[Number_Joint];
	 real_t yOpt[Number_Joint];
	 example.getPrimalSolution( xOpt );
	 example.getDualSolution( yOpt );

    example.getObjVal();

	 printf( "\n xOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n"  , 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );
	 printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

   for(int i =0 ; i < Number_Joint; i++)
      First_Result(i) = xOpt[i];
   // std::cout << First_Result << std::endl;  
      // std::cout << "yOpt[] = " << yOpt[i] << std::endl


}




Eigen::VectorXd inverser_kinematical::solvIK(Eigen::VectorXd Position_disire_, std::string path_urdf ,Eigen::VectorXd& First_Result)
{
    Tree my_tree;
    kdl_parser::treeFromFile(path_urdf,my_tree);
    Chain chain;
    bool exit_value = my_tree.getChain("base_link","ee_A",chain);

    ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
    unsigned int Number_Joint = chain.getNrOfJoints();

    JntArray jointpositions(Number_Joint);
    // JntArrayVel jointVelocity(Number_Joint);
    // Eigen::VectorXd jointpositions_vector(Number_Joint);
    // jointpositions_vector.setZero();
    // cout << chain.getSegment(0).getName() << endl <<endl;
    // jointVelocity.setZero();
    for(unsigned int i=0;i<Number_Joint;i++)
    {
     jointpositions.data(i) = 0;
     //jointpositions.data(i) = 3.14/3;
    }

    Frame cartpos;
    Jacobian jacobian_kdl(Number_Joint);    
    ChainJntToJacSolver kdl_solver(chain);
    kdl_solver.JntToJac(jointpositions,jacobian_kdl) ;    
    fksolver.JntToCart(jointpositions,cartpos);

    Eigen::MatrixXd Jacobian_4Robot_End = jacobian_kdl.data;
    Eigen::VectorXd Joint_Position_current(Number_Joint);
    Joint_Position_current.setZero();

   
    for(int loop = 0; loop < 2000; loop ++)
    {
       
    IK_solution(Jacobian_4Robot_End, 
                cartpos, Position_disire_,
                Joint_Position_current,
                Number_Joint, 
                First_Result);

    for(int i = 0; i< Number_Joint; i++)            
       jointpositions.data(i) =  Joint_Position_current(i) + First_Result(i);

    for(int i = 0; i< Number_Joint; i++)            
       Joint_Position_current(i) =  jointpositions.data(i);
    
     

    kdl_solver.JntToJac(jointpositions,jacobian_kdl) ;    
    fksolver.JntToCart(jointpositions,cartpos);

    Jacobian_4Robot_End = jacobian_kdl.data;       

    }
    
    Eigen::VectorXd current_position_endeffector(3);

    current_position_endeffector(0) = cartpos.p(0);
    current_position_endeffector(1) = cartpos.p(1);
    current_position_endeffector(2) = cartpos.p(2);

    std::cout << cartpos << std::endl;
    std::cout << Position_disire_(0) - cartpos.p(0) << std::endl;
    std::cout << Position_disire_(1) - cartpos.p(1) << std::endl;
    std::cout << Position_disire_(2)-  cartpos.p(2) << std::endl;    
    
    return current_position_endeffector;
}