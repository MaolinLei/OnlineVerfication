#include "MPC_Optimal.h"

using namespace KDL;
USING_NAMESPACE_QPOASES

MPC_OPTIMAL::MPC_OPTIMAL()
{

}

MPC_OPTIMAL::~MPC_OPTIMAL()
{

}

ros::Publisher MPC_OPTIMAL::init_Pub_Control_Node(ros::NodeHandle nh_)
{
  
  auto head_camera_joint_cmd_pub = nh_.advertise<xbot_msgs::JointCommand>("/xbotcore/command",100);
  return head_camera_joint_cmd_pub;
}


void MPC_OPTIMAL::matrix_to_real(qpOASES::real_t* dst, Eigen :: Matrix<double,Eigen::Dynamic,Eigen::Dynamic> src, int rows, int cols)
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

void MPC_OPTIMAL::Grad_Taylor_Relax(Eigen::MatrixXd Jacobian_collision_Point, Eigen::VectorXd Position_Error_ , Eigen::VectorXd Initial_joint_Vel, Eigen::MatrixXd& A_4distance_Constraint, Eigen::MatrixXd& B_4distance_Constraint, double dt)
{
   // std::cout << "Initial_joint_Vel = " <<Initial_joint_Vel <<std::endl;
   // std::cout << "Jacobian_collision_Point" << Jacobian_collision_Point <<std::endl;

     // dt = 0;

   Eigen::MatrixXd Tylor_first_order;
   //Position_Error_ = -Position_Error_;
   //std::cout << "Tylor_first_order" << (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_).transpose()  <<std::endl;
   Tylor_first_order = (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_).transpose() * (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_);
   //std::cout <<Position_Error_ <<std::endl;  
            
   //std::cout <<"dddddd="<< 2 * (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_).transpose()  << std::endl;
   //std::cout <<"Initial_joint_Vel="<< Initial_joint_Vel  << std::endl;


   A_4distance_Constraint = 2 * (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_).transpose()* Jacobian_collision_Point  ;
   //Eigen::MatrixXd Tylor_second_order_B;
   //B_4distance_Constraint = Tylor_first_order -2 * (Jacobian_collision_Point * dt * Initial_joint_Vel + Position_Error_).transpose()* Jacobian_collision_Point  * Initial_joint_Vel;   
   B_4distance_Constraint = Tylor_first_order -2 * A_4distance_Constraint  * Initial_joint_Vel;  
   //std::cout<< "B_4distance_Constraint = " <<100 - B_4distance_Constraint(0,0) <<std::endl <<std::endl;
   //Eigen::VectorXd B_4distance_Constraint;Initial_joint_Vel
   //Eigen::VectorXd A_4distance_Constraint;   
   //std::cout<< "B_4distance_Constraint = " <<B_4distance_Constraint<<std::endl;
   //std::cout<< "A_4distance_Constraint = " <<A_4distance_Constraint<<std::endl;
}



Eigen::MatrixXd MPC_OPTIMAL::Get_Jacobian_4Point(int number_Link, std::string Link_Name, Eigen::VectorXd jointpositions_ ,
                                                 Eigen::Vector3d Position_NearPoint, KDL::Tree _tree)
{
  Chain chain_;
  Chain chain_Mid;
  //std::cout  << Link_Name <<std::endl;
    

  // bool exit_value = _tree.getChain("base_link",Link_Name,chain_);

  //bool exit_value = _tree.getChain("base_link",Link_Name,chain_);
  bool exit_value = _tree.getChain("torso_2",Link_Name,chain_);

  //bool exit_value_ = my_tree. getChain("base_link",Link_Name,chain_Mid);
  
  JntArray jointpositions(chain_.getNrOfJoints());

  for(int i=0;i<chain_.getNrOfJoints();i++)
  {
     jointpositions.data(i) = jointpositions_(i);
  }

  Frame cartpos_;
  //ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain_Mid);
  ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain_);
  fksolver.JntToCart(jointpositions,cartpos_);
  // Eigen::Vector3d Rotation_LinkEnd_vector ;
  // cartpos_.M.GetEulerZYX(Rotation_LinkEnd_vector(0), Rotation_LinkEnd_vector(1), Rotation_LinkEnd_vector(2)); //z y x

  Eigen::Matrix3d orientation_;
  Eigen::Vector3d pose_;  
  for(int i=0; i<3; i++) 
    {
		for(int j=0; j<3; j++)
			orientation_(i,j) = cartpos_.M(i,j);
	  }
  pose_(0) = cartpos_.p(0);
  pose_(1) = cartpos_.p(1);
  pose_(2) = cartpos_.p(2);

  // Eigen_= Eigen::AngleAxisd(Rotation_LinkEnd_vector(0), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(Rotation_LinkEnd_vector(1),  Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(Rotation_LinkEnd_vector(2), Eigen::Vector3d::UnitX());
  Eigen::Vector3d Position_NearPoint_EndFrame;
  Position_NearPoint_EndFrame = orientation_.transpose()* (Position_NearPoint-pose_);
  
  Eigen::Quaterniond quaternion2(orientation_);

  // std::cout << "quaternion2 =" <<quaternion2 << std::endl;
  // std::cout << "Eigen_=" <<orientation_ << std::endl;
  // std::cout << "Position_NearPoint_EndFrame=" <<Position_NearPoint_EndFrame << std::endl;

  chain_.addSegment(Segment("tcp", Joint(Joint::None),Frame(Vector(Position_NearPoint_EndFrame(0), Position_NearPoint_EndFrame(1), Position_NearPoint_EndFrame(2)))));
     
  //std:: cout << "Eigen_=" <<Eigen_ << std::endl;
  ChainJntToJacSolver kdl_solver_(chain_);
  Jacobian jacobian_kdl_(chain_.getNrOfJoints());

  //std::cout<< "cartpos_="  << cartpos_ <<std::endl ; 

  kdl_solver_.JntToJac(jointpositions,jacobian_kdl_);
  
  Eigen :: MatrixXd Jacobian_;
  Jacobian_ = jacobian_kdl_.data;
  //std::cout  << jacobian_kdl_.data <<std::endl<<std::endl;
  //std::cout <<  Jacobian_ <<std::endl;
  return Jacobian_;
}

void MPC_OPTIMAL::ConstructCost_Orientation(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation)
{
// std::cout << "Jacobian_4Robot_End_4angularVec" << std::endl <<Jacobian_4Robot_End_4angularVec << std::endl;   
  Eigen::MatrixXd C_eta;   
  Eigen::MatrixXd C_w;
  Eigen::MatrixXd C_c;

  Eigen::MatrixXd Cost_Orientation;
  Eigen::VectorXd Error_with_Orientation_vector;
  Eigen::MatrixXd Middle_Cal_B, Middle_Cal_A;

 // std::cout << "First_Hieracy=" <<Rotation_op <<std::endl;

  ConstructCost_StateSpace_Orientation(Rotation_op, AangularVelocity_VECTOR_op, Rotation_desire, dt, 
                                       C_eta, C_w, C_c, Error_with_Orientation_vector);

  Eigen::MatrixXd Identity_33_ = Eigen::MatrixXd::Identity(3,3);  

  
  //std::cout << "C_eta" << std::endl <<C_eta << std::endl;   
  //std::cout << "C_w" << std::endl <<C_w << std::endl;   
  //std::cout << "C_c" << std::endl <<C_c << std::endl;    

  Eigen::MatrixXd B_Orientation;

  B_Orientation = C_w * Jacobian_4Robot_End_4angularVec;

  Eigen::MatrixXd H_QP_4Kin_Center = Eigen::MatrixXd::Zero(horizon*B_Orientation.rows(),horizon*B_Orientation.cols());

  Eigen::MatrixXd Current_Euler = Rotation_op.log() ;
  Eigen::VectorXd Current_Euler_ = UnHatt_matrix(Current_Euler) ;
  Eigen::MatrixXd C_eta_;


 for (int i=0;i<horizon;i++)
    {
        Middle_Cal_B = B_Orientation *(i+1);

        for (int j=i;j<horizon;j++)
        {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
      if( j ==i )
      {
        H_QP_4Kin_Center.block(j*B_Orientation.rows(), i*B_Orientation.cols(), 
                               B_Orientation.rows(), B_Orientation.cols()) = Middle_Cal_B;
      }
        C_eta_ = Identity_33_ + (i+1) * C_eta;
        Middle_Cal_B = C_eta_ * (i+1) * Middle_Cal_B *(i+1); 
        H_QP_4Kin_Center.block(j*B_Orientation.rows(), i*B_Orientation.cols(), 
                               B_Orientation.rows(), B_Orientation.cols()) = Middle_Cal_B;
        }
    }
  
  Eigen :: MatrixXd H_QP_4Kin_orientation;
  H_QP_4Kin_orientation = H_QP_4Kin_Center.transpose() * H_QP_4Kin_Center;

  Eigen::VectorXd Diff_DesAndCur(3*horizon);
  Diff_DesAndCur.setZero();
  
  Middle_Cal_A = Eigen::MatrixXd::Identity(3,3);
  Eigen :: MatrixXd Middle_Cal_Constant = C_c;


  for(int i=0;i<horizon;i++)
    {
      C_eta_ = Identity_33_ + (i+1) * C_eta;
      C_c = (i+1) * C_c;

      Middle_Cal_A = C_eta_ * dt * (i+1) * Middle_Cal_A;



   if(i == 0)
   {
      Diff_DesAndCur.segment(i * 3, 3) = Error_with_Orientation_vector + Middle_Cal_A * Current_Euler_ + C_c;
   } 
   else
   {
      Middle_Cal_Constant = C_c + C_eta_ *  Middle_Cal_Constant;
      Diff_DesAndCur.segment(i * 3, 3) = Error_with_Orientation_vector + Middle_Cal_A * Current_Euler_ + Middle_Cal_Constant;
   } 
    }

    //std::cout << "Diff_DesAndCur= " <<std::endl << Diff_DesAndCur <<std::endl;



    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
   
    // H_QP_4Norm = 0.1 * H_QP_4Norm;

   for(int i=0;i<horizon;i++)
   {

      // number 4 needs constraint
     //H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.080;
     //H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 14.000;
     //H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.010;
     //H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 14.000;
     //H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.0001;
     //H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.0001;

     /*H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.10;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.1000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.10;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.1000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.1;
     H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.1; */

   }
   // H_QP_4Norm.setZero();
   // H_QP_4Norm = 0.5*H_QP_4Norm;
    // H_QP_4Norm = 0.005*H_QP_4Norm;
   Eigen::MatrixXd H_Matrix_Jerk;
   Eigen::MatrixXd g_Matrix_Jerk;
   Cost_Minimum_Jerk(Number_Joint, horizon, Current_Joint_Velocity,
                                   H_Matrix_Jerk, g_Matrix_Jerk);
  


    H_Orientation = 1* H_QP_4Kin_orientation + 0.04* H_QP_4Norm.transpose()  * H_QP_4Norm ;
    g_Orientation = 2 * H_QP_4Kin_Center.transpose() * Diff_DesAndCur;


}


void MPC_OPTIMAL::ConstructCost_Orientation_ModularRobot(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation, Eigen::VectorXd Slack_Vector)
{
// std::cout << "Jacobian_4Robot_End_4angularVec" << std::endl <<Jacobian_4Robot_End_4angularVec << std::endl;   
  Eigen::MatrixXd C_eta;   
  Eigen::MatrixXd C_w;
  Eigen::MatrixXd C_c;

  Eigen::MatrixXd Cost_Orientation;
  Eigen::VectorXd Error_with_Orientation_vector;
  Eigen::MatrixXd Middle_Cal_B, Middle_Cal_A;

  ConstructCost_StateSpace_Orientation(Rotation_op, AangularVelocity_VECTOR_op, Rotation_desire, dt, 
                                       C_eta, C_w, C_c, Error_with_Orientation_vector);

  Eigen::MatrixXd Identity_33_ = Eigen::MatrixXd::Identity(3,3);  

  //std::cout << "C_eta" << std::endl <<Rotation_op << std::endl;
  //std::cout << "C_eta" << std::endl <<C_eta << std::endl;   
  //std::cout << "C_w" << std::endl <<C_w << std::endl;   
  //std::cout << "C_c" << std::endl <<C_c << std::endl;   

  Eigen::MatrixXd B_Orientation;

  B_Orientation = C_w * Jacobian_4Robot_End_4angularVec;

  Eigen::MatrixXd H_QP_4Kin_Center = Eigen::MatrixXd::Zero(horizon*B_Orientation.rows(),horizon*B_Orientation.cols());

  Eigen::MatrixXd Current_Euler = Rotation_op.log() ;
  Eigen::VectorXd Current_Euler_ = UnHatt_matrix(Current_Euler) ;
  Eigen::MatrixXd C_eta_;

 for (int i=0;i<horizon;i++)
    {
        Middle_Cal_B = B_Orientation *(i+1);

        for (int j=i;j<horizon;j++)
        {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
      if( j ==i )
      {
        H_QP_4Kin_Center.block(j*B_Orientation.rows(), i*B_Orientation.cols(), 
                               B_Orientation.rows(), B_Orientation.cols()) = Middle_Cal_B;
      }
        C_eta_ = Identity_33_ + (i+1) * C_eta;
        Middle_Cal_B = C_eta_ * (i+1) * Middle_Cal_B *(i+1); 
        H_QP_4Kin_Center.block(j*B_Orientation.rows(), i*B_Orientation.cols(), 
                               B_Orientation.rows(), B_Orientation.cols()) = Middle_Cal_B;
        }
    }
  
  Eigen :: MatrixXd H_QP_4Kin_orientation;
  Eigen::MatrixXd state_Weights(3*horizon,3*horizon);
  state_Weights.setZero();


  for(int i=0;i<horizon;i++)
  {
    state_Weights(0+3*i,0+3*i) = Slack_Vector(0);
    state_Weights(1+3*i,1+3*i) = Slack_Vector(1);
    state_Weights(2+3*i,2+3*i) = Slack_Vector(2);

  }

  H_QP_4Kin_Center = state_Weights * H_QP_4Kin_Center;
  // std::cout << "state_Weights=" << state_Weights <<std::endl;
  // std::cout << "H_QP_4Kin_orientation=" << H_QP_4Kin_Center.transpose()<<std::endl;
  H_QP_4Kin_orientation = H_QP_4Kin_Center.transpose()  * H_QP_4Kin_Center;

  Eigen::VectorXd Diff_DesAndCur(3*horizon);
  Diff_DesAndCur.setZero();
  
  Middle_Cal_A = Eigen::MatrixXd::Identity(3,3);
  Eigen :: MatrixXd Middle_Cal_Constant = C_c;


  for(int i=0;i<horizon;i++)
    {
      C_eta_ = Identity_33_ + (i+1) * C_eta;
      C_c = (i+1) * C_c;

      Middle_Cal_A = C_eta_ * dt * (i+1) * Middle_Cal_A;



   if(i == 0)
   {
      Diff_DesAndCur.segment(i * 3, 3) = Error_with_Orientation_vector + Middle_Cal_A * Current_Euler_ + C_c;
   } 
   else
   {
      Middle_Cal_Constant = C_c + C_eta_ *  Middle_Cal_Constant;
      Diff_DesAndCur.segment(i * 3, 3) = Error_with_Orientation_vector + Middle_Cal_A * Current_Euler_ + Middle_Cal_Constant;
   } 
    }

    //std::cout << "Diff_DesAndCur= " <<std::endl << Diff_DesAndCur <<std::endl;



    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
   
  // H_QP_4Norm = 0.001 * H_QP_4Norm;
   // H_QP_4Norm = 0.5*H_QP_4Norm;
   H_QP_4Norm.setZero();
    // H_QP_4Norm = 0.005*H_QP_4Norm;
  
    H_Orientation = 1* H_QP_4Kin_orientation + H_QP_4Norm.transpose()  * H_QP_4Norm ;
    g_Orientation = 2 * H_QP_4Kin_Center.transpose() * Diff_DesAndCur;


}



void MPC_OPTIMAL::Quaternion_Cost_Function(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation)
{
     Eigen::Matrix3d Identity_ = Eigen::MatrixXd::Identity(3,3);
     Eigen::Matrix3d Rotation_op_ = Rotation_op;
     Eigen::Matrix3d Rotation_desire_ = Rotation_desire.transpose();    

     Eigen::Quaterniond Quaternion_Desire(Rotation_desire_);   
     Eigen::Quaterniond Quaternion_Current(Rotation_op_);        

     Eigen::Quaterniond Error_DesireAndCurrent = Quaternion_Current * Quaternion_Desire;
     Eigen::Quaterniond Error_DesireAndCurrent_(Rotation_op_ * Rotation_desire_);

     // std::cout << Error_DesireAndCurrent << std::endl;
     // std::cout << Error_DesireAndCurrent_ << std::endl;

     Eigen::MatrixXd State_B;
     
     Eigen::MatrixXd State_B_FIRESTsection;
     Eigen::MatrixXd Screw_anguler_QUATERION;

     Eigen::VectorXd Quaterion_XIGEMA(3);
     Quaterion_XIGEMA(0) = Quaternion_Current.x();
     Quaterion_XIGEMA(1) = Quaternion_Current.y();
     Quaterion_XIGEMA(2) = Quaternion_Current.z();

     Eigen::VectorXd Quaterion_XIGEMA_Error(3);
     Quaterion_XIGEMA_Error(0) = Error_DesireAndCurrent.x();
     Quaterion_XIGEMA_Error(1) = Error_DesireAndCurrent.y();
     Quaterion_XIGEMA_Error(2) = Error_DesireAndCurrent.z();
     
     int number_Error = 3*horizon;
     Eigen::VectorXd Error_Oreintation(number_Error);
     Error_Oreintation.setZero();
     //State_B_FIRESTsection = -Skew_symmetric_matrix(Quaterion_XIGEMA) + Error_DesireAndCurrent.w() * Identity_;
     //Screw_anguler_QUATERION = State_B_FIRESTsection * Jacobian_4Robot_End_4angularVec * dt * 0.5 ;
     State_B_FIRESTsection = -Skew_symmetric_matrix(Quaterion_XIGEMA) + Error_DesireAndCurrent.w() * Identity_;
     Screw_anguler_QUATERION = State_B_FIRESTsection * Jacobian_4Robot_End_4angularVec * dt * 0.5 ;

     Eigen::MatrixXd H_Orientation_ = Eigen::MatrixXd::Zero(horizon*Screw_anguler_QUATERION.rows(),horizon*Screw_anguler_QUATERION.cols());


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
   for(int i=0;i<horizon;i++)
   {

      // number 4 needs constraint
     //H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.080;
     //H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 14.000;
     //H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.010;
     //H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 14.000;
     //H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.0001;
     //H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.0001;

     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.080;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.1500;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.20;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.1500;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.2;
     H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.2;
   }
    // 
     for(int i=0;i<horizon;i++)
   {
     Error_Oreintation.segment(i * 3, 3) = Quaterion_XIGEMA_Error;

   }
    H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    H_QP_4Norm.setZero();
   // std::cout << Screw_anguler_QUATERION << std::endl;
     for(int i=0;i<horizon;i++)
     {

      H_Orientation_.block( i * Screw_anguler_QUATERION.rows(), i * Screw_anguler_QUATERION.cols(),
                            Screw_anguler_QUATERION.rows(), Screw_anguler_QUATERION.cols()) =  Screw_anguler_QUATERION ;
     }

      H_Orientation = H_Orientation_.transpose() * H_Orientation_  + 0.01 * H_QP_4Norm.transpose() * H_QP_4Norm;
      g_Orientation = 2 * H_Orientation_.transpose() * Error_Oreintation;
     
}



void MPC_OPTIMAL::Quaternion_Cost_Function_ModularRobot(int Number_Joint, int horizon, double dt, 
                                            Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op,
                                            Eigen::MatrixXd Rotation_desire,
                                            Eigen::MatrixXd Jacobian_4Robot_End_4angularVec , Eigen::VectorXd Current_Joint_Velocity ,
                                            Eigen::MatrixXd& H_Orientation, Eigen::MatrixXd& g_Orientation, Eigen::VectorXd Slack_Vector)
{
     Eigen::Matrix3d Identity_ = Eigen::MatrixXd::Identity(3,3);
     Eigen::Matrix3d Rotation_op_ = Rotation_op;
     Eigen::Matrix3d Rotation_desire_ = Rotation_desire.transpose();    

     Eigen::Quaterniond Quaternion_Desire(Rotation_desire_);   
     Eigen::Quaterniond Quaternion_Current(Rotation_op_);        

     Eigen::Quaterniond Error_DesireAndCurrent = Quaternion_Current * Quaternion_Desire;
     Eigen::Quaterniond Error_DesireAndCurrent_(Rotation_op_ * Rotation_desire_);

     // std::cout << Error_DesireAndCurrent << std::endl;
     // std::cout << Error_DesireAndCurrent_ << std::endl;

     Eigen::MatrixXd State_B;
     
     Eigen::MatrixXd State_B_FIRESTsection;
     Eigen::MatrixXd Screw_anguler_QUATERION;

     Eigen::VectorXd Quaterion_XIGEMA(3);
     Quaterion_XIGEMA(0) = Quaternion_Current.x();
     Quaterion_XIGEMA(1) = Quaternion_Current.y();
     Quaterion_XIGEMA(2) = Quaternion_Current.z();

     Eigen::VectorXd Quaterion_XIGEMA_Error(3);
     Quaterion_XIGEMA_Error(0) = Error_DesireAndCurrent.x();
     Quaterion_XIGEMA_Error(1) = Error_DesireAndCurrent.y();
     Quaterion_XIGEMA_Error(2) = Error_DesireAndCurrent.z();
     
     int number_Error = 3*horizon;
     Eigen::VectorXd Error_Oreintation(number_Error);
     Error_Oreintation.setZero();
     //State_B_FIRESTsection = -Skew_symmetric_matrix(Quaterion_XIGEMA) + Error_DesireAndCurrent.w() * Identity_;
     //Screw_anguler_QUATERION = State_B_FIRESTsection * Jacobian_4Robot_End_4angularVec * dt * 0.5 ;
     State_B_FIRESTsection = -Skew_symmetric_matrix(Quaterion_XIGEMA) + Error_DesireAndCurrent.w() * Identity_;
     Screw_anguler_QUATERION = State_B_FIRESTsection * Jacobian_4Robot_End_4angularVec * dt * 0.5 ;

     Eigen::MatrixXd H_Orientation_ = Eigen::MatrixXd::Zero(horizon*Screw_anguler_QUATERION.rows(),horizon*Screw_anguler_QUATERION.cols());


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    Eigen::MatrixXd state_Weights(3*horizon,3*horizon);
    state_Weights.setZero();


     for(int i=0;i<horizon;i++)
    {
    state_Weights(0+3*i,0+3*i) = Slack_Vector(0);
    state_Weights(1+3*i,1+3*i) = Slack_Vector(1);
    state_Weights(2+3*i,2+3*i) = Slack_Vector(2);
    } 


    //cout << H_QP_4Kin <<endl <<endl;
    H_QP_4Norm.setZero();
     for(int i=0;i<horizon;i++)
   {
     Error_Oreintation.segment(i * 3, 3) = Quaterion_XIGEMA_Error;

   }

   // std::cout << Screw_anguler_QUATERION << std::endl;
     for(int i=0;i<horizon;i++)
     {

      H_Orientation_.block( i * Screw_anguler_QUATERION.rows(), i * Screw_anguler_QUATERION.cols(),
                            Screw_anguler_QUATERION.rows(), Screw_anguler_QUATERION.cols()) =  Screw_anguler_QUATERION ;
     }
      H_Orientation_ = state_Weights * H_Orientation_;
      H_Orientation = H_Orientation_.transpose() * H_Orientation_  + 4 * H_QP_4Norm.transpose() * H_QP_4Norm;
      g_Orientation = 2 * H_Orientation_.transpose() * Error_Oreintation;
     
}



void MPC_OPTIMAL::costFunction_VelocityandPos_ModularRobot_MPCLearning (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity ,
                           Eigen::VectorXd PositionVelocity_disire_, Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector)
{
     
       //std::cout << "PositionVelocity_disire_ =" <<  PositionVelocity_disire_ <<std::endl;
     //std::cout << "Jacobian_4Robot_End_4velocity =" <<  Jacobian_4Robot_End_4velocity <<std::endl;     
     //std::cout << "current_velocity_End =" <<  current_velocity_End <<std::endl;   
     Eigen::MatrixXd B_MaritxXd_ = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_Dot = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());

     B_MaritxXd_.topRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity;

     B_MaritxXd_.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity_dot.topRows(Jacobian_4Robot_End_4velocity.rows()) + Jacobian_4Robot_End_4velocity;

     B_MaritxXd_Dot.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = - Jacobian_4Robot_End_4velocity;

     B_MaritxXd_ADD = B_MaritxXd_ + B_MaritxXd_Dot;



     Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity(horizon * 6 , horizon *6);

     for(int i=0; i<horizon;i++)
     {
        Q_MaritxXd_Weight(0+6*i,0+6*i) = 6.25 * Slack_Vector(0);
        Q_MaritxXd_Weight(1+6*i,1+6*i) = 6.25 * Slack_Vector(1);
        Q_MaritxXd_Weight(2+6*i,2+6*i) = 6.25 * Slack_Vector(2);

        Q_MaritxXd_Weight(3+6*i,3+6*i) = 0.030*0.1000 * Slack_Vector(0);
        Q_MaritxXd_Weight(4+6*i,4+6*i) = 0.030*0.1000 * Slack_Vector(1);
        Q_MaritxXd_Weight(5+6*i,5+6*i) = 0.030*0.1000 * Slack_Vector(2);
     }
     
    // std::cout << "Q_MaritxXd_Weight=" << Q_MaritxXd_Weight <<std::endl;



     Eigen::MatrixXd B_MaritxXd_Dot_Vecotr(horizon*B_MaritxXd_Dot.rows(),B_MaritxXd_Dot.cols());
     for(int i=0;i<horizon;i++)
     {
     B_MaritxXd_Dot_Vecotr.block(i*B_MaritxXd_.rows(),0,
                                   B_MaritxXd_.rows(),B_MaritxXd_.cols()) = B_MaritxXd_Dot;
     }

     Eigen::MatrixXd H_QP_4Kin_Center = Eigen::MatrixXd::Zero(horizon*B_MaritxXd_.rows(),horizon*B_MaritxXd_.cols());

        for (int i=0;i<horizon;i++)
    {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
        H_QP_4Kin_Center.block(i*B_MaritxXd_.rows(), i*B_MaritxXd_.cols(), 
                                 B_MaritxXd_.rows(), B_MaritxXd_.cols()) = B_MaritxXd_;
    }


        for (int i=0;i<horizon;i++)
    {
          for(int j=1;j<horizon;j++)
          {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
              H_QP_4Kin_Center.block(j*B_MaritxXd_ADD.rows(), i*B_MaritxXd_ADD.cols(), 
                                       B_MaritxXd_ADD.rows(),   B_MaritxXd_ADD.cols()) = B_MaritxXd_ADD;
          }
    }

    //H_QP_4Kin = Jacobian_4Robot_End_4velocity.transpose() * Jacobian_4Robot_End_4velocity *dt*dt;
 
    
    Eigen :: MatrixXd H_QP_4Kin;
    H_QP_4Kin = H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight * H_QP_4Kin_Center;
    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);


    H_QP_4Norm.setZero();

    H_ = 2*1*H_QP_4Kin + 0.0002000*H_QP_4Norm;

    Eigen::VectorXd Current_End_PosAndVelocity(6*horizon);
    Eigen::VectorXd Diff_DesAndCur(6*horizon);

    for(int i=0;i<horizon;i++)
    {
    Current_End_PosAndVelocity(0+6*i) = cartpos.p(0);
    Current_End_PosAndVelocity(1+6*i) = cartpos.p(1);
    Current_End_PosAndVelocity(2+6*i) = cartpos.p(2);

    Current_End_PosAndVelocity(3+6*i) = current_velocity_End(0);
    Current_End_PosAndVelocity(4+6*i) = current_velocity_End(1);
    Current_End_PosAndVelocity(5+6*i) = current_velocity_End(2);
    /*Desir_End(0+3*i) = 0.5;
    Desir_End(1+3*i) = 0.3;
    Desir_End(2+3*i) = 0.2;*/
    }

    Diff_DesAndCur = Current_End_PosAndVelocity - PositionVelocity_disire_ + B_MaritxXd_Dot_Vecotr * Current_Joint_velocity;
  

    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
   g_ = 2 * H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight *Diff_DesAndCur;



}





void MPC_OPTIMAL::costFunction_VelocityandPos_ModularRobot (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity ,
                           Eigen::VectorXd PositionVelocity_disire_, Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector)
{
     
       //std::cout << "PositionVelocity_disire_ =" <<  PositionVelocity_disire_ <<std::endl;
     //std::cout << "Jacobian_4Robot_End_4velocity =" <<  Jacobian_4Robot_End_4velocity <<std::endl;     
     //std::cout << "current_velocity_End =" <<  current_velocity_End <<std::endl;   
     Eigen::MatrixXd B_MaritxXd_ = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_Dot = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());

     B_MaritxXd_.topRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity;

     B_MaritxXd_.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity_dot.topRows(Jacobian_4Robot_End_4velocity.rows()) + Jacobian_4Robot_End_4velocity;

     B_MaritxXd_Dot.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = - Jacobian_4Robot_End_4velocity;

     B_MaritxXd_ADD = B_MaritxXd_ + B_MaritxXd_Dot;



     Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity(horizon * 6 , horizon *6);

     for(int i=0; i<horizon;i++)
     {
        Q_MaritxXd_Weight(0+6*i,0+6*i) = 6.25 * Slack_Vector(0);
        Q_MaritxXd_Weight(1+6*i,1+6*i) = 6.25 * Slack_Vector(1);
        Q_MaritxXd_Weight(2+6*i,2+6*i) = 6.25 * Slack_Vector(2);

        Q_MaritxXd_Weight(3+6*i,3+6*i) = 0.030*0.1000 * Slack_Vector(0);
        Q_MaritxXd_Weight(4+6*i,4+6*i) = 0.030*0.1000 * Slack_Vector(1);
        Q_MaritxXd_Weight(5+6*i,5+6*i) = 0.030*0.1000 * Slack_Vector(2);
     }
     
    // std::cout << "Q_MaritxXd_Weight=" << Q_MaritxXd_Weight <<std::endl;



     Eigen::MatrixXd B_MaritxXd_Dot_Vecotr(horizon*B_MaritxXd_Dot.rows(),B_MaritxXd_Dot.cols());
     for(int i=0;i<horizon;i++)
     {
     B_MaritxXd_Dot_Vecotr.block(i*B_MaritxXd_.rows(),0,
                                   B_MaritxXd_.rows(),B_MaritxXd_.cols()) = B_MaritxXd_Dot;
     }

     Eigen::MatrixXd H_QP_4Kin_Center = Eigen::MatrixXd::Zero(horizon*B_MaritxXd_.rows(),horizon*B_MaritxXd_.cols());

        for (int i=0;i<horizon;i++)
    {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
        H_QP_4Kin_Center.block(i*B_MaritxXd_.rows(), i*B_MaritxXd_.cols(), 
                                 B_MaritxXd_.rows(), B_MaritxXd_.cols()) = B_MaritxXd_;
    }


        for (int i=0;i<horizon;i++)
    {
          for(int j=1;j<horizon;j++)
          {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
              H_QP_4Kin_Center.block(j*B_MaritxXd_ADD.rows(), i*B_MaritxXd_ADD.cols(), 
                                       B_MaritxXd_ADD.rows(),   B_MaritxXd_ADD.cols()) = B_MaritxXd_ADD;
          }
    }

    //H_QP_4Kin = Jacobian_4Robot_End_4velocity.transpose() * Jacobian_4Robot_End_4velocity *dt*dt;
 
    
    Eigen :: MatrixXd H_QP_4Kin;
    H_QP_4Kin = H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight * H_QP_4Kin_Center;
    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
       for(int i=0;i<horizon;i++)
   {

      // number 4 needs constraint

     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.02000;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.05000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.0200;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.02000000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.02; 
     //H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 

   }

    H_QP_4Norm.setZero();

    H_ = 2*1*H_QP_4Kin + 0.0002000*H_QP_4Norm;

    Eigen::VectorXd Current_End_PosAndVelocity(6*horizon);
    Eigen::VectorXd Diff_DesAndCur(6*horizon);

    for(int i=0;i<horizon;i++)
    {
    Current_End_PosAndVelocity(0+6*i) = cartpos.p(0);
    Current_End_PosAndVelocity(1+6*i) = cartpos.p(1);
    Current_End_PosAndVelocity(2+6*i) = cartpos.p(2);

    Current_End_PosAndVelocity(3+6*i) = current_velocity_End(0);
    Current_End_PosAndVelocity(4+6*i) = current_velocity_End(1);
    Current_End_PosAndVelocity(5+6*i) = current_velocity_End(2);
    /*Desir_End(0+3*i) = 0.5;
    Desir_End(1+3*i) = 0.3;
    Desir_End(2+3*i) = 0.2;*/
    }

    Diff_DesAndCur = Current_End_PosAndVelocity - PositionVelocity_disire_ + B_MaritxXd_Dot_Vecotr * Current_Joint_velocity;
  

    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
   g_ = 2 * H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight *Diff_DesAndCur;



}

void MPC_OPTIMAL::costFunction_VelocityandPos (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity ,
                           Eigen::VectorXd PositionVelocity_disire_, Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time)
{
     
     //std::cout << "PositionVelocity_disire_ =" <<  PositionVelocity_disire_ <<std::endl;
     //std::cout << "Jacobian_4Robot_End_4velocity =" <<  Jacobian_4Robot_End_4velocity <<std::endl;     
     //std::cout << "current_velocity_End =" <<  current_velocity_End <<std::endl;   
     Eigen::MatrixXd B_MaritxXd_ = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_Dot = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
    // Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());

     Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , 2*Jacobian_4Robot_End_4velocity.cols());

     B_MaritxXd_.topRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity;

     B_MaritxXd_.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity_dot.topRows(Jacobian_4Robot_End_4velocity.rows()) + Jacobian_4Robot_End_4velocity;

     B_MaritxXd_Dot.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = - Jacobian_4Robot_End_4velocity;

     B_MaritxXd_ADD.leftCols(B_MaritxXd_Dot.cols()) = B_MaritxXd_Dot;
     B_MaritxXd_ADD.rightCols(B_MaritxXd_.cols()) = B_MaritxXd_;
     



    //  = B_MaritxXd_ + B_MaritxXd_Dot;




     Eigen::MatrixXd B_MaritxXd_choose = Eigen::MatrixXd::Zero(horizon * Number_Joint * 2,  horizon * Number_Joint);

        // for (int i=0;i<horizon;i++)

          for(int j=1;j<horizon;j++)
          {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
              B_MaritxXd_choose.block(j*Number_Joint*2, (j-1)*Number_Joint, 
                                       Number_Joint*2,   Number_Joint*2) = Eigen::MatrixXd::Identity(  Number_Joint * 2 , Number_Joint * 2);
          }
      B_MaritxXd_choose.block(Number_Joint,0 , 
                             Number_Joint, Number_Joint) = Eigen::MatrixXd::Identity(Number_Joint, Number_Joint);;

    // std::cout << B_MaritxXd_choose << std::endl;

     Eigen::MatrixXd B_MaritxXd_Dot_Vector(horizon*B_MaritxXd_ADD.rows(),horizon*B_MaritxXd_ADD.cols());
     B_MaritxXd_Dot_Vector = Eigen::MatrixXd::Zero(horizon*B_MaritxXd_ADD.rows(),horizon*B_MaritxXd_ADD.cols());

     for(int i=0;i<horizon;i++)
     {
       for (int j=0;i<horizon;i++)
       {
         B_MaritxXd_Dot_Vector.block(j * B_MaritxXd_ADD.rows(), i* B_MaritxXd_ADD.cols(),
                                         B_MaritxXd_ADD.rows(),    B_MaritxXd_ADD.cols()) = B_MaritxXd_ADD;
       }
     }
     
     Eigen::MatrixXd H_QP_4Kin_Center;
     
     H_QP_4Kin_Center = B_MaritxXd_Dot_Vector * B_MaritxXd_choose;

     Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity(horizon * 6 , horizon *6);

     for(int i=0; i<horizon;i++)
     {
        Q_MaritxXd_Weight(0+6*i,0+6*i) = 6.25;
        Q_MaritxXd_Weight(1+6*i,1+6*i) = 6.25;
        Q_MaritxXd_Weight(2+6*i,2+6*i) = 6.25;

        Q_MaritxXd_Weight(3+6*i,3+6*i) = 0.030 *0.0;
        Q_MaritxXd_Weight(4+6*i,4+6*i) = 0.030 *0.0;
        Q_MaritxXd_Weight(5+6*i,5+6*i) = 0.030 *0.0;
     }


    Eigen :: MatrixXd H_QP_4Kin;
     
     // std::cout << "H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight" <<std::endl;

    H_QP_4Kin = H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight * H_QP_4Kin_Center;
    

    


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
   
       for(int i=0;i<horizon;i++)
   {
     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.02000;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.02000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.0200;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.02000000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.02; 
     H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 
     H_QP_4Norm(6+i*Number_Joint,6+i*Number_Joint) = 0.02; 
   }

    // if (current_time >= 2.0)
        //  if (current_time >= 1.8)
    // H_QP_4Norm = H_QP_4Norm * 0.500;
         //H_QP_4Norm = H_QP_4Norm * 0.03;
    // std::cout << H_QP_4Kin.rows() << "," << H_QP_4Kin.cols()   <<std::endl;
    // std::cout << H_QP_4Norm.rows() << "," << H_QP_4Norm.cols()   <<std::endl;
    H_ = 2*1*H_QP_4Kin + 0.0000*H_QP_4Norm;//0.045
    //H_ = 2*1*H_QP_4Kin + 0.500*H_QP_4Norm;//0.045 no hierachy
     

   // std::cout << H_ << std::endl;

    Eigen::VectorXd Current_End_PosAndVelocity(6*horizon);
    Current_End_PosAndVelocity.setZero();
    Eigen::VectorXd Diff_DesAndCur(6*horizon);
    Diff_DesAndCur.setZero();
    Eigen::VectorXd Diff_DesAndCur_C(6*horizon);
    Diff_DesAndCur_C.setZero();

    //   std::cout << Jacobian_4Robot_End_4velocity * Current_Joint_velocity << std::endl; 
   
    for(int i=0;i<horizon;i++)
    {
    Current_End_PosAndVelocity(0+6*i) = cartpos.p(0);
    Current_End_PosAndVelocity(1+6*i) = cartpos.p(1);
    Current_End_PosAndVelocity(2+6*i) = cartpos.p(2);

    Current_End_PosAndVelocity(3+6*i) = current_velocity_End(0);
    Current_End_PosAndVelocity(4+6*i) = current_velocity_End(1);
    Current_End_PosAndVelocity(5+6*i) = current_velocity_End(2);
    /*Desir_End(0+3*i) = 0.5;
    Desir_End(1+3*i) = 0.3;
    Desir_End(2+3*i) = 0.2;
    }*/
    }
   //    std::cout << Jacobian_4Robot_End_4velocity * Current_Joint_velocity << std::endl; 

   Diff_DesAndCur_C.topRows(3) = Jacobian_4Robot_End_4velocity * Current_Joint_velocity ;

   

   Diff_DesAndCur = Current_End_PosAndVelocity - PositionVelocity_disire_ + Diff_DesAndCur_C;
  

   Eigen::MatrixXd H_Matrix_Jerk;
   Eigen::MatrixXd g_Matrix_Jerk;
  // Cost_Minimum_Jerk(Number_Joint, horizon, Current_Joint_velocity,
  //                                 H_Matrix_Jerk, g_Matrix_Jerk);

   // H_ = H_ ;
    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
  g_ = 2 * H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight *Diff_DesAndCur ;
    
} 


/* void MPC_OPTIMAL::costFunction_VelocityandPos (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , Eigen::VectorXd current_velocity_End , Eigen::VectorXd Current_Joint_velocity ,
                           Eigen::VectorXd PositionVelocity_disire_, Eigen::MatrixXd Jacobian_4Robot_End_4velocity , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time)
{
     
     //std::cout << "PositionVelocity_disire_ =" <<  PositionVelocity_disire_ <<std::endl;
     //std::cout << "Jacobian_4Robot_End_4velocity =" <<  Jacobian_4Robot_End_4velocity <<std::endl;     
     //std::cout << "current_velocity_End =" <<  current_velocity_End <<std::endl;   
     Eigen::MatrixXd B_MaritxXd_ = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
     Eigen::MatrixXd B_MaritxXd_Dot = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());
    // Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , Jacobian_4Robot_End_4velocity.cols());

     Eigen::MatrixXd B_MaritxXd_ADD = Eigen::MatrixXd::Zero(2 * Jacobian_4Robot_End_4velocity.rows() , 2*Jacobian_4Robot_End_4velocity.cols());

     B_MaritxXd_.topRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity;

     B_MaritxXd_.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = dt * Jacobian_4Robot_End_4velocity_dot.topRows(Jacobian_4Robot_End_4velocity.rows()) + Jacobian_4Robot_End_4velocity;

     B_MaritxXd_Dot.bottomRows(Jacobian_4Robot_End_4velocity.rows()) = - Jacobian_4Robot_End_4velocity;

     B_MaritxXd_ADD.leftCols(B_MaritxXd_Dot.cols()) = B_MaritxXd_Dot;
     B_MaritxXd_ADD.rightCols(B_MaritxXd_.cols()) = B_MaritxXd_;
     



    //  = B_MaritxXd_ + B_MaritxXd_Dot;




     Eigen::MatrixXd B_MaritxXd_choose = Eigen::MatrixXd::Zero(horizon * Number_Joint * 2,  horizon * Number_Joint);

        // for (int i=0;i<horizon;i++)

          for(int j=1;j<horizon;j++)
          {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
              B_MaritxXd_choose.block(j*Number_Joint*2, (j-1)*Number_Joint, 
                                       Number_Joint*2,   Number_Joint*2) = Eigen::MatrixXd::Identity(  Number_Joint * 2 , Number_Joint * 2);
          }
      B_MaritxXd_choose.block(Number_Joint,0 , 
                             Number_Joint, Number_Joint) = Eigen::MatrixXd::Identity(Number_Joint, Number_Joint);;

    // std::cout << B_MaritxXd_choose << std::endl;

     Eigen::MatrixXd B_MaritxXd_Dot_Vector(horizon*B_MaritxXd_ADD.rows(),horizon*B_MaritxXd_ADD.cols());
     B_MaritxXd_Dot_Vector = Eigen::MatrixXd::Zero(horizon*B_MaritxXd_ADD.rows(),horizon*B_MaritxXd_ADD.cols());

     for(int i=0;i<horizon;i++)
     {
       for (int j=0;i<horizon;i++)
       {
         B_MaritxXd_Dot_Vector.block(j * B_MaritxXd_ADD.rows(), i* B_MaritxXd_ADD.cols(),
                                         B_MaritxXd_ADD.rows(),    B_MaritxXd_ADD.cols()) = B_MaritxXd_ADD;
       }
     }
     
     Eigen::MatrixXd H_QP_4Kin_Center;
     
     H_QP_4Kin_Center = B_MaritxXd_Dot_Vector * B_MaritxXd_choose;

     Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity(horizon * 6 , horizon *6);

     for(int i=0; i<horizon;i++)
     {
        Q_MaritxXd_Weight(0+6*i,0+6*i) = 6.25;
        Q_MaritxXd_Weight(1+6*i,1+6*i) = 6.25;
        Q_MaritxXd_Weight(2+6*i,2+6*i) = 6.25;

        Q_MaritxXd_Weight(3+6*i,3+6*i) = 0.030 *0.0;
        Q_MaritxXd_Weight(4+6*i,4+6*i) = 0.030 *0.0;
        Q_MaritxXd_Weight(5+6*i,5+6*i) = 0.030 *0.0;
     }


    Eigen :: MatrixXd H_QP_4Kin;
     
     // std::cout << "H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight" <<std::endl;

    H_QP_4Kin = H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight * H_QP_4Kin_Center;
    

    


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
   
       for(int i=0;i<horizon;i++)
   {
     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.02000;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.02000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.0200;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.02000000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.02; 
     H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 
     H_QP_4Norm(6+i*Number_Joint,6+i*Number_Joint) = 0.02; 
   }

    // if (current_time >= 2.0)
        //  if (current_time >= 1.8)
    // H_QP_4Norm = H_QP_4Norm * 0.500;
         //H_QP_4Norm = H_QP_4Norm * 0.03;
    std::cout << H_QP_4Kin.rows() << "," << H_QP_4Kin.cols()   <<std::endl;
    std::cout << H_QP_4Norm.rows() << "," << H_QP_4Norm.cols()   <<std::endl;
    H_ = 2*1*H_QP_4Kin + 0.04500*H_QP_4Norm;//0.045
    //H_ = 2*1*H_QP_4Kin + 0.500*H_QP_4Norm;//0.045 no hierachy
     

   // std::cout << H_ << std::endl;

    Eigen::VectorXd Current_End_PosAndVelocity(6*horizon);
    Current_End_PosAndVelocity.setZero();
    Eigen::VectorXd Diff_DesAndCur(6*horizon);
    Diff_DesAndCur.setZero();
    Eigen::VectorXd Diff_DesAndCur_C(6*horizon);
    Diff_DesAndCur_C.setZero();

       std::cout << Jacobian_4Robot_End_4velocity * Current_Joint_velocity << std::endl; 
   
    for(int i=0;i<horizon;i++)
    {
    Current_End_PosAndVelocity(0+6*i) = cartpos.p(0);
    Current_End_PosAndVelocity(1+6*i) = cartpos.p(1);
    Current_End_PosAndVelocity(2+6*i) = cartpos.p(2);

    Current_End_PosAndVelocity(3+6*i) = current_velocity_End(0);
    Current_End_PosAndVelocity(4+6*i) = current_velocity_End(1);
    Current_End_PosAndVelocity(5+6*i) = current_velocity_End(2);
    /*Desir_End(0+3*i) = 0.5;
    Desir_End(1+3*i) = 0.3;
    Desir_End(2+3*i) = 0.2;
    }*/
 /*      std::cout << Jacobian_4Robot_End_4velocity * Current_Joint_velocity << std::endl; 

   Diff_DesAndCur_C.topRows(3) = Jacobian_4Robot_End_4velocity * Current_Joint_velocity ;

   

   Diff_DesAndCur = Current_End_PosAndVelocity - PositionVelocity_disire_ + Diff_DesAndCur_C;
  

   Eigen::MatrixXd H_Matrix_Jerk;
   Eigen::MatrixXd g_Matrix_Jerk;
  // Cost_Minimum_Jerk(Number_Joint, horizon, Current_Joint_velocity,
  //                                 H_Matrix_Jerk, g_Matrix_Jerk);

   // H_ = H_ ;
    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
  g_ = 2 * H_QP_4Kin_Center.transpose() * Q_MaritxXd_Weight *Diff_DesAndCur ;
    
} */



void MPC_OPTIMAL::costFunction_VelocityandPos_Orientation (int Number_Joint, int horizon, double dt,  KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time)
{

    Eigen::MatrixXd H_Orietnation;
    Eigen::MatrixXd g_Orientation;
    Eigen::MatrixXd H_Position;
    Eigen::MatrixXd g_Position;

   ConstructCost_Orientation(Number_Joint, horizon, dt , 
                             Rotation_op , current_angvelocity_End, 
                             Rotation_desire, Jacobian_4Robot_End.bottomRows(3) , Current_Joint_velocity,
                             H_Orietnation, g_Orientation); 
                           
  /* costFunction_VelocityandPos(Number_Joint, horizon, dt , cartpos,
                               current_velocity_End , Current_Joint_velocity, 
                               PositionVelocity_desire_, 
                               Jacobian_4Robot_End.topRows(3) , Jacobian_4Robot_End_4velocity_dot,  
                               H_Position, g_Position, current_time) ; 
    */
    costFunction_Velocity(Number_Joint, horizon, dt, cartpos ,
                          PositionVelocity_desire_.topRows(3*horizon) ,Jacobian_4Robot_End.topRows(3) ,
                          H_Position, g_Position);
    /*  Quaternion_Cost_Function( Number_Joint,  horizon,  dt, 
                              Rotation_op, current_angvelocity_End,
                              Rotation_desire,
                              Jacobian_4Robot_End.bottomRows(3) , Current_Joint_velocity,
                              H_Orietnation, g_Orientation); */ 

   // H_ =   1.0*H_Position;
   // g_ =    1.0*g_Position;
   std:: cout << "H_Position" << std::endl << H_Position << std::endl;
   std:: cout << "g_Position" << std::endl << g_Position << std::endl;   

  {
   H_ =  2.000 * 2.000 * H_Orietnation + 0*  1.0*H_Position;
   g_ =  2.000 * g_Orientation +  0 *1.0*g_Position;
    //H_ =   0.0000 * 0.000 * H_Orietnation +  1.0*H_Position;
    //g_ =   0.000 * g_Orientation +  1.0*g_Position;
  }
  }


void MPC_OPTIMAL::costFunction_VelocityandPos_Orientation_ModularRobot_MPClearning (int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector)
{

    Eigen::MatrixXd H_Orietnation;
    Eigen::MatrixXd g_Orientation;
    Eigen::MatrixXd H_Position;
    Eigen::MatrixXd g_Position;

    

    
  costFunction_VelocityandPos_ModularRobot_MPCLearning (Number_Joint, horizon, dt , cartpos,
                               current_velocity_End , Current_Joint_velocity, 
                               PositionVelocity_desire_, 
                               Jacobian_4Robot_End.topRows(3) , Jacobian_4Robot_End_4velocity_dot,  
                               H_Position, g_Position, current_time, Slack_Vector.topRows(3)) ;

                              
  ConstructCost_Orientation_ModularRobot(Number_Joint, horizon, dt , 
                             Rotation_op , current_angvelocity_End, 
                             Rotation_desire, Jacobian_4Robot_End.bottomRows(3) , Current_Joint_velocity,
                             H_Orietnation, g_Orientation, Slack_Vector.bottomRows(3));
     
 // std::cout << Rotation_op <<std::endl;   


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl
   
     /// 0.005
  
   H_ =  2.500 * 2.50 * H_Orietnation  +  1.0*H_Position + 0.0001* H_QP_4Norm.transpose() * H_QP_4Norm;
   g_ =  2.50 * g_Orientation  +  1.0*g_Position ;

  
  }





void MPC_OPTIMAL::costFunction_VelocityandPos_Orientation_ModularRobot (int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector)
{

    Eigen::MatrixXd H_Orietnation;
    Eigen::MatrixXd g_Orientation;
    Eigen::MatrixXd H_Position;
    Eigen::MatrixXd g_Position;

    

    
  costFunction_VelocityandPos_ModularRobot(Number_Joint, horizon, dt , cartpos,
                               current_velocity_End , Current_Joint_velocity, 
                               PositionVelocity_desire_, 
                               Jacobian_4Robot_End.topRows(3) , Jacobian_4Robot_End_4velocity_dot,  
                               H_Position, g_Position, current_time, Slack_Vector.topRows(3)) ;

                              
  ConstructCost_Orientation_ModularRobot(Number_Joint, horizon, dt , 
                             Rotation_op , current_angvelocity_End, 
                             Rotation_desire, Jacobian_4Robot_End.bottomRows(3) , Current_Joint_velocity,
                             H_Orietnation, g_Orientation, Slack_Vector.bottomRows(3));
     
 // std::cout << Rotation_op <<std::endl;   


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
       for(int i=0;i<horizon;i++)
   {

      // number 4 needs constraint

     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.02000;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.05000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.0500;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.02000000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.02; 
     //H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 


    // H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 
    // H_QP_4Norm(6+i*Number_Joint,6+i*Number_Joint) = 0.02; 
   }

   
     /// 0.005
  
   H_ =  2.500 * 2.50 * H_Orietnation  +  1.0*H_Position + 1* H_QP_4Norm.transpose() * H_QP_4Norm;
   g_ =  2.50 * g_Orientation  +  1.0*g_Position ;


    // H_ =   5.00 * 5.00 * H_Orietnation  +  1.0*H_Position + 0.1* H_QP_4Norm.transpose() * H_QP_4Norm;
    // g_ =   5.00 * g_Orientation  +  1.0*g_Position ;
   // std::cout <<"g_Orientation " << g_Orientation <<std::endl;
   // std::cout <<"H_Orietnation " <<H_Orietnation <<std::endl;
    //H_ =   0.0000 * 0.000 * H_Orietnation +  1.0*H_Position;
    //g_ =   0.000 * g_Orientation +  1.0*g_Position;
  
  }



void MPC_OPTIMAL::costFunction_VelocityandPos_Orientation_ModularRobotCompare (int Number_Joint, int horizon, double dt, KDL::Frame cartpos , 
                           Eigen::VectorXd current_velocity_End , Eigen::VectorXd current_angvelocity_End,
                           Eigen::VectorXd Current_Joint_velocity , Eigen::MatrixXd Rotation_op,
                           Eigen::VectorXd PositionVelocity_desire_, Eigen::MatrixXd Rotation_desire,
                           Eigen::MatrixXd Jacobian_4Robot_End , Eigen::MatrixXd Jacobian_4Robot_End_4velocity_dot,  
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_, double current_time, Eigen::VectorXd Slack_Vector)
{

    Eigen::MatrixXd H_Orietnation;
    Eigen::MatrixXd g_Orientation;
    Eigen::MatrixXd H_Position;
    Eigen::MatrixXd g_Position;

    

    
    costFunction_VelocityandPos_ModularRobot(Number_Joint, horizon, dt , cartpos,
                               current_velocity_End , Current_Joint_velocity, 
                               PositionVelocity_desire_, 
                               Jacobian_4Robot_End.topRows(3) , Jacobian_4Robot_End_4velocity_dot,  
                               H_Position, g_Position, current_time, Slack_Vector.topRows(3)) ;
    
      Quaternion_Cost_Function_ModularRobot(Number_Joint,  horizon,  dt, 
                              Rotation_op, current_angvelocity_End,
                              Rotation_desire,
                              Jacobian_4Robot_End.bottomRows(3) , Current_Joint_velocity,
                              H_Orietnation, g_Orientation, Slack_Vector.bottomRows(3)); 
     
 // std::cout << Rotation_op <<std::endl;   


    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
       for(int i=0;i<horizon;i++)
   {

      // number 4 needs constraint

     H_QP_4Norm(0+i*Number_Joint,0+i*Number_Joint) = 0.02000;
     H_QP_4Norm(1+i*Number_Joint,1+i*Number_Joint) = 0.05000;
     H_QP_4Norm(2+i*Number_Joint,2+i*Number_Joint) = 0.0500;
     H_QP_4Norm(3+i*Number_Joint,3+i*Number_Joint) = 0.02000000;
     H_QP_4Norm(4+i*Number_Joint,4+i*Number_Joint) = 0.02; 
     //H_QP_4Norm(5+i*Number_Joint,5+i*Number_Joint) = 0.02; 

   }

   
     /// 0.005
  
   H_ =   0.8000 * 0.80 * H_Orietnation  +  1.0*H_Position + 0.008* H_QP_4Norm.transpose() * H_QP_4Norm;
   g_ =   0.8000 * g_Orientation  +  1.0*g_Position ;

   // std::cout <<"g_Orientation " << g_Orientation <<std::endl;
   // std::cout <<"H_Orietnation " <<H_Orietnation <<std::endl;
    //H_ =   0.0000 * 0.000 * H_Orietnation +  1.0*H_Position;
    //g_ =   0.000 * g_Orientation +  1.0*g_Position;
  
  }



 void MPC_OPTIMAL:: Orientation_Trajectory_Generation(double t_segment , Eigen::MatrixXd init_Orientation,  
                                                     Eigen::MatrixXd _Rotation_Error_, Eigen::MatrixXd& Desire_Oritation)
{
  if(t_segment<=1)
  {
  // Eigen::MatrixXd _Rotation_Error_ = (init_Orientation.transpose() * Desire_Oritation).log();
  Desire_Oritation =  init_Orientation * (t_segment * _Rotation_Error_).exp() ;
  }
  else
  {
  t_segment =1; 
  Desire_Oritation =  init_Orientation * _Rotation_Error_.exp() ;

  }


}

void MPC_OPTIMAL:: Orientation_Trajectory_Error(Eigen::MatrixXd Desire_Oritation, Eigen::MatrixXd init_Orientation, 
                                               Eigen::MatrixXd& _Rotation_Error_)
{
   
     _Rotation_Error_ = (init_Orientation.transpose() * Desire_Oritation).log();
}

void MPC_OPTIMAL:: costFunction_Velocity(int Number_Joint, int horizon, double dt, Frame cartpos ,
                           Eigen::VectorXd Position_disire_ ,Eigen::MatrixXd Jacobian_4Robot_End_4velocity ,
                           Eigen::MatrixXd& H_, Eigen::MatrixXd& g_)
{
     Eigen::MatrixXd H_QP_4Kin_Center = Eigen::MatrixXd::Zero(horizon*Jacobian_4Robot_End_4velocity.rows(),horizon*Jacobian_4Robot_End_4velocity.cols());

        for (int i=0;i<horizon;i++)
    {
        for (int j=i;j<horizon;j++)
        {
        //H_QP_4Kin_Center.block(j*3  , 6*i, 3, 6)  = Jacobian_4Robot_End_4velocity;
        H_QP_4Kin_Center.block(j*Jacobian_4Robot_End_4velocity.rows(), i*Jacobian_4Robot_End_4velocity.cols(), 
                            Jacobian_4Robot_End_4velocity.rows(), Jacobian_4Robot_End_4velocity.cols()) = Jacobian_4Robot_End_4velocity *dt;
        }
    }

    // H_QP_4Kin = Jacobian_4Robot_End_4velocity.transpose() * Jacobian_4Robot_End_4velocity *dt*dt;

  //  std::cout << "Jacobian_4Robot_End_4velocity *dt = " << std::endl << Jacobian_4Robot_End_4velocity *dt << std::endl;

     Eigen::MatrixXd Q_MaritxXd_Weight = Eigen::MatrixXd::Identity(horizon * 3 , horizon *3);

     for(int i=0; i<horizon;i++)
     {
        Q_MaritxXd_Weight(0+3*i,0+3*i) = 6.25;
        Q_MaritxXd_Weight(1+3*i,1+3*i) = 6.25;
        Q_MaritxXd_Weight(2+3*i,2+3*i) = 6.25;
     }

    Eigen :: MatrixXd H_QP_4Kin;
    H_QP_4Kin = H_QP_4Kin_Center.transpose() *Q_MaritxXd_Weight * H_QP_4Kin_Center ;
    Eigen::MatrixXd H_QP_4Norm =  Eigen::MatrixXd::Identity(Number_Joint*horizon,Number_Joint*horizon);
    //cout << H_QP_4Kin <<endl <<endl;
    H_QP_4Norm.setZero();
    H_ = 2*1*H_QP_4Kin + 0.030800000*H_QP_4Norm;

    Eigen::VectorXd Curre_End(3*horizon);
    Eigen::VectorXd Diff_DesAndCur(3*horizon);

    for(int i=0;i<horizon;i++)
    {
    Curre_End(0+3*i) = cartpos.p(0);
    Curre_End(1+3*i) = cartpos.p(1);
    Curre_End(2+3*i) = cartpos.p(2);

    /*Desir_End(0+3*i) = 0.5;
    Desir_End(1+3*i) = 0.3;
    Desir_End(2+3*i) = 0.2;*/
    }

    Diff_DesAndCur = Curre_End - Position_disire_;
    // std::cout << "Diff_DesAndCur = " <<Diff_DesAndCur << std::endl;

    //g_ = 2 * Jacobian_4Robot_End_4velocity.transpose() * Diff_DesAndCur *dt;
    g_ = 2 * H_QP_4Kin_Center.transpose()*Q_MaritxXd_Weight * Diff_DesAndCur;
    
}

void MPC_OPTIMAL::Constraint_4robot_postion_acceleration(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower)
{
    Eigen::VectorXd upper_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd upper_acceletion_joint(Number_Joint*horizon); 

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    upper_position_joint(i+j*Number_Joint) = 2.5;
    upper_velocity_joint(i+j*Number_Joint) = 2.5;
    upper_acceletion_joint(i+j*Number_Joint) = 3.14 ;
    }
    
     /*upper_position_joint(1+j*Number_Joint) = 1.5;
     upper_position_joint(3+j*Number_Joint) = 1.7;
     upper_position_joint(5+j*Number_Joint) = 1.7;

     upper_velocity_joint(0+j*Number_Joint) = 2.0;   
     upper_velocity_joint(1+j*Number_Joint) = 2.0;   
     upper_velocity_joint(5+j*Number_Joint) = 1.6;
     //upper_acceletion_joint(5+j*Number_Joint) = 0.5*3.14;
     
     //upper_acceletion_joint(1+j*Number_Joint) = 3.14;
     upper_acceletion_joint(5+j*Number_Joint) = 1.1; */

    }
    
    Eigen::VectorXd lower_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd lower_acceletion_joint(Number_Joint*horizon); 

    lower_position_joint = -upper_position_joint;
    lower_velocity_joint = -upper_velocity_joint;
    lower_acceletion_joint = -upper_acceletion_joint;



    Vector_Constraint_4pos_upper = upper_position_joint - position_joint_current; 
    Vector_Constraint_4pos_lower = lower_position_joint - position_joint_current; 

    Vector_Constraint_4acc_upper. setZero();
    Vector_Constraint_4acc_lower. setZero();
    for(int i=0;i < horizon ;i++)
    {
    Vector_Constraint_4acc_upper.middleRows(i*Number_Joint, Number_Joint) = upper_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    Vector_Constraint_4acc_lower.middleRows(i*Number_Joint, Number_Joint) = lower_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    }

}



void MPC_OPTIMAL::Constraint_dualarm_posandvel(Eigen::VectorXd Vector_Current_4pos_left_right, 
                                               Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                               Eigen::VectorXd& Vector_Constraint_4vel_upper, Eigen::VectorXd& Vector_Constraint_4vel_lower)
{
  int Number_Joint = 7;
  double dt = 0.01;
  

  Vector_Constraint_4pos_upper(0) = 1.2;
  Vector_Constraint_4pos_upper(1) = 3.40;
  Vector_Constraint_4pos_upper(2) = 2.37;
  Vector_Constraint_4pos_upper(3) = 0.08;
  Vector_Constraint_4pos_upper(4) = 2.36;
  Vector_Constraint_4pos_upper(5) = 1.31;
  Vector_Constraint_4pos_upper(6) = 2.37;

  Vector_Constraint_4pos_upper(7) = 1.6;
  Vector_Constraint_4pos_upper(8) = -0.01;
  Vector_Constraint_4pos_upper(9) = 2.56;
  Vector_Constraint_4pos_upper(10) = 0.29;
  Vector_Constraint_4pos_upper(11) = 2.55;
  Vector_Constraint_4pos_upper(12) = 1.52;
  Vector_Constraint_4pos_upper(13) = 2.57;
  Vector_Constraint_4pos_upper = Vector_Constraint_4pos_upper - Vector_Current_4pos_left_right;

  Vector_Constraint_4pos_lower(0) = -3.0;
  Vector_Constraint_4pos_lower(1) = 0.30;
  Vector_Constraint_4pos_lower(2) = -2.0;
  Vector_Constraint_4pos_lower(3) = -2.26;
  Vector_Constraint_4pos_lower(4) = -2.36;
  Vector_Constraint_4pos_lower(5) = -1.31;
  Vector_Constraint_4pos_lower(6) = -2.37;

  Vector_Constraint_4pos_lower(7) = -3.35;
  Vector_Constraint_4pos_lower(8) = -3.43;
  Vector_Constraint_4pos_lower(9) = -2.56;
  Vector_Constraint_4pos_lower(10) =-2.48;
  Vector_Constraint_4pos_lower(11) =-2.54;
  Vector_Constraint_4pos_lower(12) =-1.52;
  Vector_Constraint_4pos_lower(13) =-2.56;

  Vector_Constraint_4pos_lower = Vector_Constraint_4pos_lower - Vector_Current_4pos_left_right;

  for(int j=0;j<Number_Joint+Number_Joint;j++)
  {
    Vector_Constraint_4vel_upper(j) = 3.5 *dt ;
    Vector_Constraint_4vel_lower(j) = -3.5 *dt;  
  }

  for(int j=0;j<Number_Joint+Number_Joint;j++)
  {
    if(Vector_Constraint_4pos_upper(j)>=Vector_Constraint_4vel_upper(j))
    {
      Vector_Constraint_4pos_upper(j)=Vector_Constraint_4vel_upper(j);
    }

    if(Vector_Constraint_4pos_lower(j)<=Vector_Constraint_4vel_lower(j))
    {
      Vector_Constraint_4pos_lower(j)=Vector_Constraint_4vel_lower(j);
    }
  }


}

void MPC_OPTIMAL::Constraint_4robot_postion_acceleration_ModularRobot(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower)
{
    Eigen::VectorXd upper_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd upper_acceletion_joint(Number_Joint*horizon); 

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    upper_position_joint(i+j*Number_Joint) = 2.0;
    upper_velocity_joint(i+j*Number_Joint) = 2.5;
    upper_acceletion_joint(i+j*Number_Joint) = 6.4 ;
    }
    upper_position_joint(0+j*Number_Joint) = 2.8;
    upper_position_joint(3+j*Number_Joint) = 2.8;



//    upper_position_joint(4+j*Number_Joint) = 1.74;

      upper_position_joint(4+j*Number_Joint) = 1.5;
    }
    
    Eigen::VectorXd lower_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd lower_acceletion_joint(Number_Joint*horizon); 

    lower_position_joint = -upper_position_joint;
    lower_velocity_joint = -upper_velocity_joint;
    lower_acceletion_joint = -upper_acceletion_joint;

    Vector_Constraint_4pos_upper = upper_position_joint - position_joint_current; 
    Vector_Constraint_4pos_lower = lower_position_joint - position_joint_current; 


    Vector_Constraint_4acc_upper. setZero();
    Vector_Constraint_4acc_lower. setZero();
    for(int i=0;i < horizon ;i++)
    {
    Vector_Constraint_4acc_upper.middleRows(i*Number_Joint, Number_Joint) = upper_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    Vector_Constraint_4acc_lower.middleRows(i*Number_Joint, Number_Joint) = lower_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    }

}


void MPC_OPTIMAL::Constraint_4robot_postion_acceleration_ModularRobot_MPCLearning(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower)
{
    Eigen::VectorXd upper_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd upper_acceletion_joint(Number_Joint*horizon); 

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    upper_position_joint(i+j*Number_Joint) = 2.0;
    upper_velocity_joint(i+j*Number_Joint) = 2.5;
    upper_acceletion_joint(i+j*Number_Joint) = 6.4 ;
    }
    }
    
    Eigen::VectorXd lower_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd lower_acceletion_joint(Number_Joint*horizon); 

    lower_position_joint = -upper_position_joint;
    lower_velocity_joint = -upper_velocity_joint;
    lower_acceletion_joint = -upper_acceletion_joint;

    Vector_Constraint_4pos_upper = upper_position_joint - position_joint_current; 
    Vector_Constraint_4pos_lower = lower_position_joint - position_joint_current; 


    Vector_Constraint_4acc_upper. setZero();
    Vector_Constraint_4acc_lower. setZero();
    for(int i=0;i < horizon ;i++)
    {
    Vector_Constraint_4acc_upper.middleRows(i*Number_Joint, Number_Joint) = upper_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    Vector_Constraint_4acc_lower.middleRows(i*Number_Joint, Number_Joint) = lower_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    }

}








void MPC_OPTIMAL::Constraint_4robot_postion_acceleration_Centaruo(int Number_Joint, int horizon, double dt, Eigen::VectorXd position_joint_current, Eigen::VectorXd velocity_inLastTIme,
                                             Eigen::VectorXd& upper_velocity_joint, Eigen::VectorXd& lower_velocity_joint,
                                             Eigen::VectorXd& Vector_Constraint_4pos_upper, Eigen::VectorXd& Vector_Constraint_4pos_lower,
                                             Eigen::VectorXd& Vector_Constraint_4acc_upper, Eigen::VectorXd& Vector_Constraint_4acc_lower)
{
    Eigen::VectorXd upper_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd upper_acceletion_joint(Number_Joint*horizon); 

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {

    upper_velocity_joint(i+j*Number_Joint) = 3.5;
    upper_acceletion_joint(i+j*Number_Joint) = 6.4; //6.4
    }


     //upper_velocity_joint(2 + j *Number_Joint) = 1.0;
     //upper_acceletion_joint(5+j*Number_Joint) = 0.5*3.14;
     upper_position_joint(0+j*Number_Joint) = 1.2;
     upper_position_joint(1+j*Number_Joint) = 3.40;
     upper_position_joint(2+j*Number_Joint) = 2.37;
     upper_position_joint(3+j*Number_Joint) = 0.08;
     upper_position_joint(4+j*Number_Joint) = 2.36;
     upper_position_joint(5+j*Number_Joint) = 1.31;
     upper_position_joint(6+j*Number_Joint) = 2.37;

    }
    
    Eigen::VectorXd lower_position_joint(Number_Joint*horizon); 
    Eigen::VectorXd lower_acceletion_joint(Number_Joint*horizon); 

    lower_position_joint = -upper_position_joint;
    lower_velocity_joint = -upper_velocity_joint;
    lower_acceletion_joint = -upper_acceletion_joint;
  
      for(int j=0;j<horizon;j++)
    {
     //upper_acceletion_joint(5+j*Number_Joint) = 0.5*3.14;
     lower_position_joint(0+j*Number_Joint) = -3.0;
     lower_position_joint(1+j*Number_Joint) = 0.30;
     lower_position_joint(2+j*Number_Joint) = -2.0;
     lower_position_joint(3+j*Number_Joint) = -2.26;
     lower_position_joint(4+j*Number_Joint) = -2.36;
     lower_position_joint(5+j*Number_Joint) = -1.31;
     lower_position_joint(6+j*Number_Joint) = -2.37;

    }



    Vector_Constraint_4pos_upper = upper_position_joint - position_joint_current; 
    Vector_Constraint_4pos_lower = lower_position_joint - position_joint_current; 

   // std::cout << "Vector_Constraint_4pos_upper = " << std::endl << Vector_Constraint_4pos_upper << std::endl;
   // std::cout << "Vector_Constraint_4pos_lower = " << std::endl << Vector_Constraint_4pos_lower << std::endl;

    Vector_Constraint_4acc_upper. setZero();
    Vector_Constraint_4acc_lower. setZero();
    for(int i=0;i < horizon ;i++)
    {
    Vector_Constraint_4acc_upper.middleRows(i*Number_Joint, Number_Joint) = upper_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    Vector_Constraint_4acc_lower.middleRows(i*Number_Joint, Number_Joint) = lower_acceletion_joint.middleRows(i*Number_Joint, Number_Joint) * (i+1)* dt + velocity_inLastTIme; 
    }

}


void MPC_OPTIMAL::Matrix_Constraint_4acc_pos_acc_ModularRobot(int Number_Joint, int horizon, double dt,
                                                 Eigen::MatrixXd& Matrix_Constraint_4pos_, Eigen::MatrixXd& Matrix_Constraint_4acc_)
{
    
    Matrix_Constraint_4pos_ = Eigen::MatrixXd::Identity(Number_Joint * horizon, Number_Joint * horizon) ;
    for(int i=0;i < horizon ;i++)
    {
      for(int j=0; j<= i;j++)
       {
        Matrix_Constraint_4pos_.block(i*Number_Joint, j*Number_Joint, Number_Joint, Number_Joint) =  Eigen::MatrixXd::Identity(Number_Joint, Number_Joint ) *dt ;
       }  
    }
  // std::cout << "Matrix_Constraint_4pos_= " <<Matrix_Constraint_4pos_ <<std::endl;
    Matrix_Constraint_4acc_ = Eigen::MatrixXd::Identity(Number_Joint * horizon, Number_Joint * horizon) ;

}



void MPC_OPTIMAL::Matrix_Constraint_4acc_pos_acc(int Number_Joint, int horizon, double dt,
                                                 Eigen::MatrixXd& Matrix_Constraint_4pos_, Eigen::MatrixXd& Matrix_Constraint_4acc_)
{
    
    Matrix_Constraint_4pos_ = Eigen::MatrixXd::Identity(Number_Joint * horizon, Number_Joint * horizon) ;
    for(int i=0;i < horizon ;i++)
    {
            for(int j=0; j<= i;j++)
       {
        Matrix_Constraint_4pos_.block(i*Number_Joint, j*Number_Joint, Number_Joint, Number_Joint) =  Eigen::MatrixXd::Identity(Number_Joint, Number_Joint ) *dt;
       } 
    }

    Matrix_Constraint_4acc_ = Eigen::MatrixXd::Identity(Number_Joint * horizon, Number_Joint * horizon) ;

}

void MPC_OPTIMAL::caluculate_Proper(Eigen::VectorXd& axis_distance_on_, Eigen::VectorXd& expection_4Obj_Pos_4Next, 
                       Eigen::VectorXd expection_4Obj_Pos, Eigen::VectorXd expection_4Obj_Vel, double dt)
 {
        Eigen::Vector3d Covarious_4Obj_Pos;
        Covarious_4Obj_Pos<< 0.1,0.1,0.1;
        Eigen::MatrixXd Eigen_Covarious_4Obj_Pos(Covarious_4Obj_Pos.asDiagonal());


        Eigen::Vector3d Covarious_4Obj_Vel;
        Covarious_4Obj_Vel<< 0.1,0.1,0.01;
        Eigen::MatrixXd Eigen_Covarious_4Obj_Vel(Covarious_4Obj_Vel.asDiagonal());

        Eigen::Vector3d Covarious_4Obj_Pos_4Next;

        expection_4Obj_Pos_4Next = expection_4Obj_Pos + expection_4Obj_Vel *dt;
        Covarious_4Obj_Pos_4Next = Covarious_4Obj_Pos + Covarious_4Obj_Vel *dt *dt;

        Eigen::MatrixXd Eigen_Covarious_4Obj_Pos_4Next(Covarious_4Obj_Pos_4Next.asDiagonal());

        axis_distance_on_(0) = 4 * Covarious_4Obj_Pos_4Next(0);
        axis_distance_on_(1) = 4 * Covarious_4Obj_Pos_4Next(1);
        axis_distance_on_(2) = 4 * Covarious_4Obj_Pos_4Next(2);
        //std::cout << axis_distance_on_ <<std::endl;
  }


Eigen::MatrixXd MPC_OPTIMAL::Skew_symmetric_matrix(Eigen::VectorXd Position_Error_ )
{
   Eigen::MatrixXd Skew_symmetric_matrix(3,3);
   Skew_symmetric_matrix <<    0.0,                -Position_Error_[2], Position_Error_[1],
                               Position_Error_[2],  0.0,               -Position_Error_[0], 
                              -Position_Error_[1], Position_Error_[0], 0.0;
   return Skew_symmetric_matrix;
}



Eigen::MatrixXd MPC_OPTIMAL::KroneckerProductSparse(Eigen::MatrixXd I , Eigen::MatrixXd X )
{
   Eigen::MatrixXd P2(I.rows() * X.rows(), I.cols() * X.cols());
   P2.setZero();
   for (int i = 0; i < I.rows(); i++)
    {
       P2.block(i*X.rows(), i*X.cols(), X.rows(), X.cols()) = I(i, i) * X;
    }
   return P2;
}

Eigen::VectorXd MPC_OPTIMAL::UnHatt_matrix(Eigen::MatrixXd Eigen_Skew_symmetric)
{
   Eigen::VectorXd UnHatt_matrix(3);
   UnHatt_matrix << -Eigen_Skew_symmetric(1,2), Eigen_Skew_symmetric(0,2),-Eigen_Skew_symmetric(0,1);
   return UnHatt_matrix;
}

Eigen::MatrixXd MPC_OPTIMAL::ErrorWithOrientation(Eigen::MatrixXd Desire_Orientation, Eigen::MatrixXd Current_Orientation)
{
   Eigen::MatrixXd Error_Orientation;
   Error_Orientation = Desire_Orientation.inverse() * Current_Orientation;
   Error_Orientation = Error_Orientation.log();
   return Error_Orientation;
}

Eigen::VectorXd MPC_OPTIMAL::Vecterization(Eigen::MatrixXd Matrix_Need_Vection)
{

  int Vector_Length;
  Vector_Length = Matrix_Need_Vection.rows() * Matrix_Need_Vection.cols();
  Eigen::VectorXd Result_(Vector_Length);
  for(int i =0; i< Matrix_Need_Vection.rows() ;i++)
  {
  Result_.segment(i * Matrix_Need_Vection.rows(), Matrix_Need_Vection.rows()) = Matrix_Need_Vection.col(i);
  }

   return Result_;
}



void MPC_OPTIMAL::ConstructCost_StateSpace_Orientation(Eigen::MatrixXd Rotation_op, Eigen::VectorXd AangularVelocity_VECTOR_op, Eigen::MatrixXd Rotation_desire, double dt, 
                                                       Eigen::MatrixXd& C_eta, Eigen::MatrixXd& C_w, Eigen::MatrixXd& C_c, Eigen::VectorXd& Error_with_Orientation_vector)
 {

  Eigen::MatrixXd N_4Inverse;
  N_4Inverse = Calculated_inverse_with_N();

  Eigen::MatrixXd AangularVelocity_op = Skew_symmetric_matrix(AangularVelocity_VECTOR_op);

  Eigen::MatrixXd Transform_Skew_Matrix(9,3);
   Transform_Skew_Matrix << 0, 0, 0, 0, 0, 1, 0,-1, 0,
                           0, 0,-1, 0, 0, 0, 1, 0, 0,
                           0, 1, 0,-1, 0, 0, 0, 0, 0; 
  //Eigen::MatrixXd C_eta;   
  //Eigen::MatrixXd C_w;
  //Eigen::MatrixXd C_c;

  Eigen::MatrixXd Identity_33_ = Eigen::MatrixXd::Identity(3,3);  

  Eigen::MatrixXd Rotation_op_AangularVelocity_op;
  Rotation_op_AangularVelocity_op = AangularVelocity_op * Rotation_op; 
  Eigen::MatrixXd Constant_sec_A;
  Constant_sec_A = KroneckerProductSparse(Rotation_op.transpose(),Identity_33_);
  Eigen::MatrixXd Constant_sec_B;
  Constant_sec_B = KroneckerProductSparse(Rotation_op.transpose(),AangularVelocity_op) * Transform_Skew_Matrix;
  Eigen::MatrixXd Constant_sec_C;
  //std :: cout << KroneckerProductSparse(Identity_33_,AangularVelocity_op)  << std::endl << std::endl; 
  //std :: cout << Vecterization(AangularVelocity_op)<< std::endl; 
  Constant_sec_C = Vecterization(AangularVelocity_op) - Constant_sec_A * Transform_Skew_Matrix *AangularVelocity_VECTOR_op;

  //std::cout << Constant_sec_C << std::endl;

  C_c   = dt * N_4Inverse * Constant_sec_A * Constant_sec_C;
  C_w   = dt * N_4Inverse * Constant_sec_A * Constant_sec_A * Transform_Skew_Matrix;
  C_eta = dt * N_4Inverse * Constant_sec_A * Constant_sec_B;
  // Eigen::MatrixXd Error_with_Orientation; 
   Error_Orientation_Vector(Rotation_op, Rotation_desire, Error_with_Orientation_vector);


   //std::cout << "Error_with_Orientation_vector=" << std::endl << Error_with_Orientation_vector << std::endl;

}


void MPC_OPTIMAL::Cost_Minimum_Jerk(int Number_Joint, int horizon, Eigen::VectorXd Current_Joint_Velcity,
                                   Eigen::MatrixXd& H_Matrix_Jerk, Eigen::MatrixXd& g_Matrix_Jerk)
 {

  H_Matrix_Jerk = Eigen::MatrixXd::Identity(horizon*Number_Joint,horizon*Number_Joint);
  
  for(int i=0; i<Number_Joint-1 ;i++)
  {
    H_Matrix_Jerk(i+1,i) = -1;
  }

  g_Matrix_Jerk = Eigen::MatrixXd::Identity(horizon*Number_Joint,1);
  g_Matrix_Jerk.setZero();
  g_Matrix_Jerk.topRows(Number_Joint) = Current_Joint_Velcity;

}


void MPC_OPTIMAL::Error_Orientation_Vector( Eigen::MatrixXd Rotation_op, Eigen::MatrixXd Rotation_desire,Eigen::VectorXd& Error_with_Orientation_vector)
{

  Eigen:: MatrixXd Error_with_Orientation; 
  Error_with_Orientation = Rotation_op * Rotation_desire.transpose();
  Error_with_Orientation = Error_with_Orientation.log();
  Error_with_Orientation_vector = UnHatt_matrix(Error_with_Orientation);
  // std::cout <<"Error_with_Orientation_vector =" << Error_with_Orientation_vector << std::endl;

}


Eigen::MatrixXd MPC_OPTIMAL::Calulated_pseudo_inverse_left(Eigen::MatrixXd Pesudo_matrix)
{
   Eigen::MatrixXd _pseudo_inverse_left;
   _pseudo_inverse_left = (Pesudo_matrix.transpose() * Pesudo_matrix).inverse()*Pesudo_matrix.transpose();
   return _pseudo_inverse_left;
}

Eigen::MatrixXd MPC_OPTIMAL::Calculated_inverse_with_N()
{
Eigen::MatrixXd Transform_Skew_Matrix(9,3);
Transform_Skew_Matrix << 0, 0, 0, 0, 0, 1, 0,-1, 0,
                         0, 0,-1, 0, 0, 0, 1, 0, 0,
                         0, 1, 0,-1, 0, 0, 0, 0, 0; 
 Eigen::MatrixXd Result_4Inverse = Calulated_pseudo_inverse_left(Transform_Skew_Matrix);
 //std::cout << "Calculated_inverse_with_N" << std::endl <<  Result_4Inverse * Transform_Skew_Matrix << std::endl;
 return Result_4Inverse;
 }





void MPC_OPTIMAL::CenControl_Pub_Desire_right(ros::Publisher _joint_cmd_pub, Eigen::VectorXd position)
{
    xbot_msgs::JointCommand j_arm2_1;
    xbot_msgs::JointCommand j_arm2_2;
    xbot_msgs::JointCommand j_arm2_3;
    xbot_msgs::JointCommand j_arm2_4;
    xbot_msgs::JointCommand j_arm2_5;
    xbot_msgs::JointCommand j_arm2_6;
    xbot_msgs::JointCommand j_arm2_7;

    j_arm2_1.name.push_back("j_arm2_1");
    j_arm2_1.ctrl_mode.push_back(9);
    j_arm2_1.stiffness.push_back(1000);
    //head_camera_cmd.position.push_back(0.62);
    j_arm2_2.name.push_back("j_arm2_2");
    j_arm2_2.ctrl_mode.push_back(9);
    j_arm2_2.stiffness.push_back(1000);

    j_arm2_3.name.push_back("j_arm2_3");
    j_arm2_3.ctrl_mode.push_back(9);
    j_arm2_3.stiffness.push_back(1000);

    j_arm2_4.name.push_back("j_arm2_4");
    j_arm2_4.ctrl_mode.push_back(9);
    j_arm2_4.stiffness.push_back(1000);

    j_arm2_5.name.push_back("j_arm2_5");
    j_arm2_5.ctrl_mode.push_back(9);
    j_arm2_5.stiffness.push_back(1000);

    j_arm2_6.name.push_back("j_arm2_6");
    j_arm2_6.ctrl_mode.push_back(9);
    j_arm2_6.stiffness.push_back(1000);

    j_arm2_7.name.push_back("j_arm2_7");
    j_arm2_7.ctrl_mode.push_back(9);
    j_arm2_7.stiffness.push_back(1000);

    j_arm2_1.position.push_back(position(0));
    j_arm2_2.position.push_back(position(1));
    j_arm2_3.position.push_back(position(2));
    j_arm2_4.position.push_back(position(3));
    j_arm2_5.position.push_back(position(4));
    j_arm2_6.position.push_back(position(5));
    j_arm2_7.position.push_back(position(6));

    _joint_cmd_pub.publish(j_arm2_1);
    _joint_cmd_pub.publish(j_arm2_2);
    _joint_cmd_pub.publish(j_arm2_3);
    _joint_cmd_pub.publish(j_arm2_4);
    _joint_cmd_pub.publish(j_arm2_5);
    _joint_cmd_pub.publish(j_arm2_6);
    _joint_cmd_pub.publish(j_arm2_7);
}


void MPC_OPTIMAL::CenControl_Pub_Desire_left(ros::Publisher _joint_cmd_pub, Eigen::VectorXd position)
{
    xbot_msgs::JointCommand j_arm1_1;
    xbot_msgs::JointCommand j_arm1_2;
    xbot_msgs::JointCommand j_arm1_3;
    xbot_msgs::JointCommand j_arm1_4;
    xbot_msgs::JointCommand j_arm1_5;
    xbot_msgs::JointCommand j_arm1_6;
    xbot_msgs::JointCommand j_arm1_7;

    j_arm1_1.name.push_back("j_arm1_1");
    j_arm1_1.ctrl_mode.push_back(9);
    j_arm1_1.stiffness.push_back(1000);
    //head_camera_cmd.position.push_back(0.62);
    j_arm1_2.name.push_back("j_arm1_2");
    j_arm1_2.ctrl_mode.push_back(9);
    j_arm1_2.stiffness.push_back(1000);

    j_arm1_3.name.push_back("j_arm1_3");
    j_arm1_3.ctrl_mode.push_back(9);
    j_arm1_3.stiffness.push_back(1000);

    j_arm1_4.name.push_back("j_arm1_4");
    j_arm1_4.ctrl_mode.push_back(9);
    j_arm1_4.stiffness.push_back(1000);

    j_arm1_5.name.push_back("j_arm1_5");
    j_arm1_5.ctrl_mode.push_back(9);
    j_arm1_5.stiffness.push_back(1000);

    j_arm1_6.name.push_back("j_arm1_6");
    j_arm1_6.ctrl_mode.push_back(9);
    j_arm1_6.stiffness.push_back(1000);

    j_arm1_7.name.push_back("j_arm1_7");
    j_arm1_7.ctrl_mode.push_back(9);
    j_arm1_7.stiffness.push_back(1000);

    j_arm1_1.position.push_back(position(0));
    j_arm1_2.position.push_back(position(1));
    j_arm1_3.position.push_back(position(2));
    j_arm1_4.position.push_back(position(3));
    j_arm1_5.position.push_back(position(4));
    j_arm1_6.position.push_back(position(5));
    j_arm1_7.position.push_back(position(6));

    _joint_cmd_pub.publish(j_arm1_1);
    _joint_cmd_pub.publish(j_arm1_2);
    _joint_cmd_pub.publish(j_arm1_3);
    _joint_cmd_pub.publish(j_arm1_4);
    _joint_cmd_pub.publish(j_arm1_5);
    _joint_cmd_pub.publish(j_arm1_6);
    _joint_cmd_pub.publish(j_arm1_7);
}



void MPC_OPTIMAL::Second_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    Eigen::MatrixXd Jacobian_input, Eigen::MatrixXd Desire_velocity, 
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::VectorXd& result_jointVel, int orientation)
{

    // std::cout << Jacobian_4Robot_End * Joint_velocity_ <<std::endl;
    Eigen::MatrixXd Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Eigen::MatrixXd Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);

   
    Eigen::VectorXd velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    Eigen::VectorXd angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);

		Rotation_op = Rotation_current;


    Eigen::VectorXd position_joint_current(Number_Joint*horizon);
    


    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }

    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 
   /* if(orientation)
  {
    ConstructCost_Orientation_ModularRobot(Number_Joint, horizon, dt , 
                              Rotation_op , angvelocity_Jacobian_end_, 
                              Rotation_desire, Jacobian_4Robot_End_4angvelocity , Joint_velocity_,
                              H_, g_, Slack_Vector.topRows(3)); 
  }

  else
    {
     costFunction_VelocityandPos_ModularRobot(Number_Joint, horizon, dt ,cartpos,
                                              velocity_Jacobian_end_ , Joint_velocity_, 
                                              Pos_Vel_disire_, 
                                              Jacobian_4Robot_End_4velocity , JacobianDot_4Robot_End_,  
                                              H_, g_, t, Slack_Vector.bottomRows(3));
    } */

   costFunction_VelocityandPos_Orientation_ModularRobot( Number_Joint, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_current, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t , Slack_Vector);    

   /* costFunction_VelocityandPos_Orientation_ModularRobotCompare(Number_Joint, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_current, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t , Slack_Vector); 
   */
//////////////////////////////////////////////////////////////////////////////////////////

    // std::cout << "robot" <<std::endl;
    
    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint * horizon); 
    Eigen:: VectorXd Vector_Constraint_4acc_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4acc_lower (Number_Joint * horizon);  

    Eigen:: VectorXd upper_velocity_joint(Number_Joint * horizon); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint * horizon); 
    Constraint_4robot_postion_acceleration_ModularRobot(Number_Joint, horizon, dt, position_joint_current, Joint_velocity_, 
                                             upper_velocity_joint , lower_velocity_joint,
                                             Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                             Vector_Constraint_4acc_upper, Vector_Constraint_4acc_lower);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    Eigen:: MatrixXd Matrix_Constraint_4pos_ (Number_Joint * horizon, Number_Joint * horizon) ;
    Eigen:: MatrixXd Matrix_Constraint_4acc_(Number_Joint * horizon,Number_Joint * horizon) ;
    Matrix_Constraint_4acc_pos_acc_ModularRobot(Number_Joint, horizon, dt, Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon + Jacobian_input.rows(), Number_Joint * horizon);  
    Eigen_4Constraint_A.setZero();
    int NumberConstraint = Number_Joint * horizon;

    Eigen_4Constraint_A.topRows(Matrix_Constraint_4pos_.rows()) = Matrix_Constraint_4pos_;
    Eigen_4Constraint_A.block(NumberConstraint,0,Jacobian_input.rows(),Number_Joint) = Jacobian_input;

    // std::cout << "Eigen_4Constraint_A.block(NumberConstraint,0,Jacobian_input.rows(),Number_Joint) =" << std::endl << Eigen_4Constraint_A.block(NumberConstraint,0,Jacobian_input.rows(),Number_Joint) <<std::endl;
    // std::cout << "Jacobian_input =" << std::endl << Jacobian_input <<std::endl;
    // std::cout << "Eigen_4Constraint_A =" << std::endl << Eigen_4Constraint_A <<std::endl;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon + Jacobian_input.rows());  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon + Jacobian_input.rows());  
    Vector_Constraint_Aconstraint_lower.setZero();
    Vector_Constraint_Aconstraint_upper.setZero();

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 

    Vector_Constraint_Aconstraint_lower.bottomRows(Jacobian_input.rows()) =  Jacobian_input *  Desire_velocity; 
    //Jacobian_input * Desire_velocity; 

    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;

    Vector_Constraint_Aconstraint_upper.bottomRows(Jacobian_input.rows()) =  Jacobian_input * Desire_velocity;
    // Jacobian_input * Desire_velocity;

////////////////////////////////////////
	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
    matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());


  	qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
    matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);



    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	   matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_Aconstraint_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_Aconstraint_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);



	QProblem example( horizon*Number_Joint, horizon*Number_Joint + Jacobian_input.rows());

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 1000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[Number_Joint*horizon + Jacobian_input.rows()];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );


	printf( "\nxOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal());
     

    for(int i=0;i<Number_Joint;i++)
    {
    result_jointVel(i) = xOpt[i];

    }



}




void MPC_OPTIMAL::MPC_Learning(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::MatrixXd& Desire_Matrix, Eigen::VectorXd& First_Result, int orientation)
  {

    Eigen::MatrixXd Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Eigen::MatrixXd Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);

    Eigen::VectorXd velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    Eigen::VectorXd angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    // Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);


    Eigen::VectorXd position_joint_current(Number_Joint*horizon);

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }
    
   
    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 
   int number_slack = 0;
   int cols_number = 0;
   for(int i=0;i<6;i++)
   {
     if(Slack_Vector(i)==1)
     {
       number_slack = number_slack+1;
       // Desire_Matrix.row(cols_number) = Jacobian_4Robot_End.row(i);
     }
 
   }
   
   Desire_Matrix = Eigen::MatrixXd::Zero(number_slack,Jacobian_4Robot_End.cols());
     for(int i=0;i<6;i++)
   {
     if(Slack_Vector(i)==1)
     {
       // number_slack = number_slack+1;
        Desire_Matrix.row(cols_number) = Jacobian_4Robot_End.row(i);
        cols_number = cols_number +1;
     }
       
   }

   costFunction_VelocityandPos_Orientation_ModularRobot_MPClearning(Number_Joint, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_current, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t , Slack_Vector);    
   

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }

    //collision_detection.joint_states_input_ = Desire_position;

    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint * horizon); 
    Eigen:: VectorXd Vector_Constraint_4acc_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4acc_lower (Number_Joint * horizon);  

    Eigen:: VectorXd upper_velocity_joint(Number_Joint * horizon); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint * horizon); 
    Constraint_4robot_postion_acceleration_ModularRobot_MPCLearning(Number_Joint, horizon, dt, position_joint_current, Joint_velocity_, 
                                             upper_velocity_joint , lower_velocity_joint,
                                             Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                             Vector_Constraint_4acc_upper, Vector_Constraint_4acc_lower);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    Eigen:: MatrixXd Matrix_Constraint_4pos_ (Number_Joint * horizon, Number_Joint * horizon) ;
    Eigen:: MatrixXd Matrix_Constraint_4acc_(Number_Joint * horizon,Number_Joint * horizon) ;
    Matrix_Constraint_4acc_pos_acc_ModularRobot(Number_Joint, horizon, dt, Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   //std::cout << Matrix_Constraint_4pos_ <<std::endl;


    Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon , Number_Joint * horizon);  
    int NumberConstraint = Number_Joint * horizon;
    Eigen_4Constraint_A.setZero();
    Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon );  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon );  

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 
   // Vector_Constraint_Aconstraint_lower.bottomRows(NumberConstraint) = Vector_Constraint_4acc_lower;

    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;

////////////////////////////////////////
	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());

	  qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
  	matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);

    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_Aconstraint_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_Aconstraint_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);




	QProblem example( horizon*Number_Joint, horizon*Number_Joint );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[(Number_Joint)*horizon];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );

	printf( "\n xOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );


    for(int i=0; i<Number_Joint ; i ++)
    {
      First_Result(i) = xOpt[i];

    }




}



void MPC_OPTIMAL::First_Hieracy(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t, Eigen::VectorXd Slack_Vector,
                    Eigen::MatrixXd& Desire_Matrix, Eigen::VectorXd& First_Result, int orientation)
  {

    Eigen::MatrixXd Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Eigen::MatrixXd Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);

    Eigen::VectorXd velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    Eigen::VectorXd angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    // Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);


    Eigen::VectorXd position_joint_current(Number_Joint*horizon);

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }
    
   
    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 
   int number_slack = 0;
   int cols_number = 0;
   for(int i=0;i<6;i++)
   {
     if(Slack_Vector(i)==1)
     {
       number_slack = number_slack+1;
       // Desire_Matrix.row(cols_number) = Jacobian_4Robot_End.row(i);
     }
 
   }
   
   Desire_Matrix = Eigen::MatrixXd::Zero(number_slack,Jacobian_4Robot_End.cols());
     for(int i=0;i<6;i++)
   {
     if(Slack_Vector(i)==1)
     {
       // number_slack = number_slack+1;
        Desire_Matrix.row(cols_number) = Jacobian_4Robot_End.row(i);
        cols_number = cols_number +1;
     }
       
   }

   costFunction_VelocityandPos_Orientation_ModularRobot(Number_Joint, horizon, dt ,cartpos,
                                                        velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                                        Joint_velocity_, Rotation_current, 
                                                        Pos_Vel_disire_, Rotation_desire,
                                                        Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                                        H_, g_ , t , Slack_Vector);    
   

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    }

    //collision_detection.joint_states_input_ = Desire_position;

    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint * horizon); 
    Eigen:: VectorXd Vector_Constraint_4acc_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4acc_lower (Number_Joint * horizon);  

    Eigen:: VectorXd upper_velocity_joint(Number_Joint * horizon); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint * horizon); 
    Constraint_4robot_postion_acceleration_ModularRobot(Number_Joint, horizon, dt, position_joint_current, Joint_velocity_, 
                                             upper_velocity_joint , lower_velocity_joint,
                                             Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                             Vector_Constraint_4acc_upper, Vector_Constraint_4acc_lower);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    Eigen:: MatrixXd Matrix_Constraint_4pos_ (Number_Joint * horizon, Number_Joint * horizon) ;
    Eigen:: MatrixXd Matrix_Constraint_4acc_(Number_Joint * horizon,Number_Joint * horizon) ;
    Matrix_Constraint_4acc_pos_acc_ModularRobot(Number_Joint, horizon, dt, Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   //std::cout << Matrix_Constraint_4pos_ <<std::endl;


    Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon , Number_Joint * horizon);  
    int NumberConstraint = Number_Joint * horizon;
    Eigen_4Constraint_A.setZero();
    Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon );  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon );  

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 
   // Vector_Constraint_Aconstraint_lower.bottomRows(NumberConstraint) = Vector_Constraint_4acc_lower;

    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;
   // Vector_Constraint_Aconstraint_upper.bottomRows(NumberConstraint) = Vector_Constraint_4acc_upper;

 // std::cout << "Eigen_4Constraint_A" << std::endl <<Eigen_4Constraint_A<< std::endl;
 // std::cout << "Vector_Constraint_4pos_upper" << std::endl <<Vector_Constraint_4pos_upper<< std::endl;

////////////////////////////////////////
	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());

	  qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
  	matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);

    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_Aconstraint_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_Aconstraint_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);




	QProblem example( horizon*Number_Joint, horizon*Number_Joint );

	Options options;
	example.setOptions( options );

	/* Solve first QP. */
	int_t nWSR = 10000;
	example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, 0);

	/* Get and print solution of first QP. */
	real_t xOpt[Number_Joint*horizon];
	real_t yOpt[(Number_Joint)*horizon];
	example.getPrimalSolution( xOpt );
	//example.getDualSolution( yOpt );

	printf( "\n xOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );


    for(int i=0; i<Number_Joint ; i ++)
    {
      First_Result(i) = xOpt[i];

    }




}


void MPC_OPTIMAL::optimal_online(Eigen::MatrixXd Jacobian_4Robot_End, Eigen::MatrixXd JacobianDot_4Robot_End_,  
                    KDL::Frame cartpos, Eigen::VectorXd Pos_Vel_disire_,
                    Eigen::MatrixXd Rotation_desire, Eigen::MatrixXd Rotation_current,
                    Eigen::VectorXd Joint_velocity_, Eigen::VectorXd Joint_Position_current,
                    double dt, int horizon, int Number_Joint, double t,
                    Eigen::VectorXd& First_Result)
  {

    Eigen::MatrixXd Jacobian_4Robot_End_4velocity = Jacobian_4Robot_End.topRows(3);
    Eigen::MatrixXd Jacobian_4Robot_End_4angvelocity = Jacobian_4Robot_End.bottomRows(3);

    Eigen::VectorXd velocity_Jacobian_end_ = Jacobian_4Robot_End_4velocity * Joint_velocity_;
    Eigen::VectorXd angvelocity_Jacobian_end_ = Jacobian_4Robot_End_4angvelocity * Joint_velocity_;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen::MatrixXd H_;
    Eigen::MatrixXd g_;
    // Eigen::MatrixXd Rotation_op(3,3);
    Eigen::MatrixXd Error_Rotation_op(3,3);


    Eigen::VectorXd position_joint_current(Number_Joint*horizon);

    for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);
    }
    }
    
   
    //MPC_OPTIMAL_.costFunction_Velocity(number_joint, horizon, dt ,cartpos, 
    //                      Position_disire_, Jacobian_4Robot_End_4velocity ,H_, g_); 
   int number_slack = 0;

   
  // Desire_Matrix = Eigen::MatrixXd::Zero(number_slack,Jacobian_4Robot_End.cols());


      costFunction_VelocityandPos_Orientation(Number_Joint, horizon, dt ,cartpos,
                                           velocity_Jacobian_end_ ,  angvelocity_Jacobian_end_,
                                           Joint_velocity_, Rotation_current, 
                                           Pos_Vel_disire_, Rotation_desire,
                                           Jacobian_4Robot_End , JacobianDot_4Robot_End_,  
                                           H_, g_ , t);  
   
     /* costFunction_Velocity(Number_Joint, horizon, dt, cartpos ,
                           Pos_Vel_disire_.topRows(3*horizon) ,Jacobian_4Robot_End.topRows(3) ,
                           H_, g_); 
      */
////////////////////////////////////////////////////////////////////////////////////////////

    // Desire_Matrix = Jacobian_4Robot_End_4angvelocity;

   /* for(int j=0;j<horizon;j++)
    {
    for (int i=0;i<Number_Joint;i++)
    {
    position_joint_current(i+j*Number_Joint) = Joint_Position_current(i);

    }
    } */

    //collision_detection.joint_states_input_ = Desire_position;

    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint * horizon); 
    Eigen:: VectorXd Vector_Constraint_4acc_upper (Number_Joint * horizon);  
    Eigen:: VectorXd Vector_Constraint_4acc_lower (Number_Joint * horizon);  

    Eigen:: VectorXd upper_velocity_joint(Number_Joint * horizon); 
    Eigen:: VectorXd lower_velocity_joint(Number_Joint * horizon); 
    Constraint_4robot_postion_acceleration_Centaruo(Number_Joint, horizon, dt, position_joint_current, Joint_velocity_, 
                                             upper_velocity_joint , lower_velocity_joint,
                                             Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                             Vector_Constraint_4acc_upper, Vector_Constraint_4acc_lower);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    Eigen:: MatrixXd Matrix_Constraint_4pos_ (Number_Joint * horizon, Number_Joint * horizon) ;
    Eigen:: MatrixXd Matrix_Constraint_4acc_(Number_Joint * horizon, Number_Joint * horizon) ;
    Matrix_Constraint_4acc_pos_acc(Number_Joint, horizon, dt, Matrix_Constraint_4pos_, Matrix_Constraint_4acc_);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   //std::cout << Matrix_Constraint_4pos_ <<std::endl;


   // Eigen:: MatrixXd Eigen_4Constraint_A (Number_Joint* horizon , Number_Joint * horizon);  
    int NumberConstraint = Number_Joint * horizon;
   // Eigen_4Constraint_A.setZero();
   // Eigen_4Constraint_A.topRows(NumberConstraint) = Matrix_Constraint_4pos_;
    
    Eigen:: MatrixXd  Eigen_4Constraint_A = Eigen::MatrixXd::Identity(Number_Joint* horizon , Number_Joint * horizon);  


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint * horizon );  
    Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint * horizon );  

    Vector_Constraint_Aconstraint_lower.topRows(NumberConstraint) = Vector_Constraint_4pos_lower; 
    Vector_Constraint_Aconstraint_upper.topRows(NumberConstraint) = Vector_Constraint_4pos_upper;
    

    for(int i=0;i<NumberConstraint;i++)
    {
      Vector_Constraint_Aconstraint_upper(i) =  100;
      Vector_Constraint_Aconstraint_lower(i) = -100;
    }
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

    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());

    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = lower_velocity_joint;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = upper_velocity_joint;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_4pos_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_4pos_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);

   // std::cout << "Matrix_Constraint_4pos_ = " << std::endl << Matrix_Constraint_4pos_ << std::endl;
   // std::cout << "Vector_Constraint_Aconstraint_lower_matrix = " << std::endl << Vector_Constraint_Aconstraint_lower_matrix << std::endl;
   // std::cout << "Vector_Constraint_Aconstraint_upper_matrix = " << std::endl << Vector_Constraint_Aconstraint_upper_matrix << std::endl;

		// QProblem example( horizon*Number_Joint, horizon*Number_Joint );

	  QProblemB example( horizon*Number_Joint );

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
	 real_t xOpt[Number_Joint*horizon];
	 real_t yOpt[Number_Joint*horizon];
	 example.getPrimalSolution( xOpt );
	 example.getDualSolution( yOpt );

   cost_int_ = example.getObjVal();

	 printf( "\n xOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n"  , 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );
	 printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

    for(int i=0; i<Number_Joint * horizon ; i ++)
    {
      First_Result(i) = xOpt[i];
      // std::cout << "yOpt[] = " << yOpt[i] << std::endl;
    }


}

void MPC_OPTIMAL::QP_online_DualArm(int Nearest_Link_left, int Nearest_Link_right, 
                                    std::string left_Arm_Link,  std::string right_Arm_Link,
                                    Eigen::VectorXd Position_left, Eigen::VectorXd Position_right,
                                    Eigen::MatrixXd Jacobian_left, Eigen::MatrixXd Jacobian_right,
                                    Eigen::VectorXd Joint_Position_current_left, Eigen::VectorXd Joint_Position_current_right, 
                                    Eigen::VectorXd Joint_Velocity_current_left, Eigen::VectorXd Joint_Velocity_current_right, 
                                    Eigen::Vector3d nearest_points_left, Eigen::Vector3d nearest_points_right,
                                    Eigen::VectorXd position_desired_left, Eigen::VectorXd position_desired_right,
                                    KDL::Tree _tree, double distance,
                                    Eigen::VectorXd& Changing_Position)
                    
{
    Eigen::MatrixXd Jacobian_4Robot_End_4velocity_left = Jacobian_left.topRows(3);
    Eigen::VectorXd velocity_Jacobian_end_left = Jacobian_4Robot_End_4velocity_left * Joint_Velocity_current_left;

    Eigen::MatrixXd Jacobian_4Robot_End_4velocity_right = Jacobian_right.topRows(3);
    Eigen::VectorXd velocity_Jacobian_end_right = Jacobian_4Robot_End_4velocity_right * Joint_Velocity_current_right;

    Eigen::MatrixXd H_Center = Eigen::MatrixXd::Zero(Jacobian_4Robot_End_4velocity_left.rows() + Jacobian_4Robot_End_4velocity_right.rows(),
                                               Jacobian_4Robot_End_4velocity_left.cols() + Jacobian_4Robot_End_4velocity_right.cols());

    H_Center.block(0, 0, Jacobian_4Robot_End_4velocity_left.rows(), Jacobian_4Robot_End_4velocity_left.cols()) = Jacobian_4Robot_End_4velocity_left;
    H_Center.block(Jacobian_4Robot_End_4velocity_left.rows(), Jacobian_4Robot_End_4velocity_left.cols(), 
                   Jacobian_4Robot_End_4velocity_right.rows(), Jacobian_4Robot_End_4velocity_right.cols()) = Jacobian_4Robot_End_4velocity_right;
   
    Eigen::MatrixXd H_Zero;
    H_Zero = Eigen::MatrixXd::Identity(14,14); 

    Eigen::MatrixXd H_State = Eigen::MatrixXd::Identity(6,6); 
    Eigen::MatrixXd H_;
    // H_Center.setZero();
    
  //  if(distance <= 0.05)
   {

        H_State(0,0) = 0.0;
        H_State(1,1) = 0.0;
        H_State(2,2) = 0;

        H_State(3,3) = 0.0;
        H_State(4,4) = 0.0;
        H_State(5,5) = 0.0;

   //  H_Zero = H_Zero ;

   
   } 

    H_ = H_Center.transpose() * H_State * H_Center + 0.001*H_Zero;

 
    Eigen::VectorXd g_Center;
    Eigen::VectorXd Diff_DesAndCur(3+3);
    int Number_Joint = 7;
    double dt = 0.01;
    Diff_DesAndCur.setZero();
    // Diff_DesAndCur.topRows(3) = velocity_Jacobian_end_left * dt - position_desired_left;
    // Diff_DesAndCur.bottomRows(3) = velocity_Jacobian_end_right * dt - position_desired_right;
    Diff_DesAndCur.topRows(3) = Position_left + velocity_Jacobian_end_left * dt * 0  - position_desired_left;
    Diff_DesAndCur.bottomRows(3) = Position_right + velocity_Jacobian_end_right * dt * 0  - position_desired_right;
    // Diff_DesAndCur.setZero();

    Eigen::MatrixXd g_;
    g_ = 2 * H_Center.transpose() * H_State * Diff_DesAndCur;

    // std::cout << "g_ =" << g_ << std::endl; 
    // std::cout << "H_Center.transpose() * H_State =" << H_Center.transpose() * H_State << std::endl; 
    // std::cout << "H_State * H_Center =" << H_State * H_Center<< std::endl;   

    double Max_distance_;
    Eigen::MatrixXd Distance_constraint;
    Distance_constraint = Grad_Taylor_Relax_dualarm(Nearest_Link_left, Nearest_Link_right, 
                                                    left_Arm_Link,     right_Arm_Link,
                                                    Jacobian_left, Jacobian_right,
                                                    Joint_Position_current_left, Joint_Position_current_right, 
                                                    nearest_points_left, nearest_points_right,
                                                     _tree, distance,
                                                     Max_distance_);
    
     
    
    
    // Eigen:: MatrixXd Eigen_4Constraint_A (1, Number_Joint + Number_Joint);
     // Eigen:: MatrixXd Eigen_4Constraint_A (3, Number_Joint + Number_Joint);
    // Eigen:: MatrixXd Eigen_4Constraint_A (5, Number_Joint + Number_Joint);  
    Eigen:: MatrixXd Eigen_4Constraint_A (7, Number_Joint + Number_Joint);
    Eigen_4Constraint_A.setZero();

    // std::cout << "Eigen_4Constraint_A =" << Eigen_4Constraint_A << std::endl;
    // Eigen_4Constraint_A.setZero();
    // Eigen_4Constraint_A.topRows(Number_Joint+Number_Joint) = Eigen::MatrixXd::Identity(Number_Joint+Number_Joint, Number_Joint+Number_Joint);
    
     Eigen_4Constraint_A.topRows(1) = Distance_constraint;  
    /* Eigen_4Constraint_A.block(2, 1, 
                              2, Jacobian_4Robot_End_4velocity_left.cols()) = Jacobian_4Robot_End_4velocity_left.topRows(2);
    */ 

     // Eigen_4Constraint_A.middleRows(1,1) = H_Center.middleRows(2,1);
     // Eigen_4Constraint_A.bottomRows(1) = H_Center.middleRows(5,1);

    // Eigen_4Constraint_A.middleRows(1,2) = H_Center.middleRows(1,2);
    // Eigen_4Constraint_A.bottomRows(2) = H_Center.middleRows(3,2);

     
    Eigen_4Constraint_A.bottomRows(6) = H_Center.bottomRows(6);

    // std::cout << "Eigen_4Constraint_A= " << Eigen_4Constraint_A << std::endl;
    // Eigen_4Constraint_A.setZero();
    // std::cout << "Eigen_4Constraint_A =" << Distance_constraint << std::endl;
/////////////////////////////////////////////////////////////////////
    Eigen:: VectorXd Vector_Constraint_4pos_upper (Number_Joint  + Number_Joint);  
    Eigen:: VectorXd Vector_Constraint_4pos_lower (Number_Joint  + Number_Joint); 
    Eigen:: VectorXd Vector_Constraint_4vel_upper (Number_Joint  + Number_Joint);  
    Eigen:: VectorXd Vector_Constraint_4vel_lower (Number_Joint  + Number_Joint);  
    
    Vector_Constraint_4pos_lower.setZero();
    Vector_Constraint_4pos_upper.setZero();
    Vector_Constraint_4vel_upper.setZero();
    Vector_Constraint_4vel_lower.setZero();

    Eigen::VectorXd Vector_Current_4pos_left_right(Number_Joint  + Number_Joint);
    Vector_Current_4pos_left_right.topRows(Number_Joint) = Joint_Position_current_left;
    Vector_Current_4pos_left_right.bottomRows(Number_Joint) = Joint_Position_current_right;


    Constraint_dualarm_posandvel(Vector_Current_4pos_left_right,
                                 Vector_Constraint_4pos_upper, Vector_Constraint_4pos_lower,
                                 Vector_Constraint_4vel_upper, Vector_Constraint_4vel_lower);
  
    for(int i=0;i<Number_Joint  + Number_Joint;i++)
    {
     // Vector_Constraint_4pos_lower(i) = -0.01;
     // Vector_Constraint_4pos_upper(i) =  0.01;

    }
    
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (5);  
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (5); 
    
     // Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (3);  
     // Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (3); 
    
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (Number_Joint  + Number_Joint);  
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (Number_Joint  + Number_Joint);  
    // std::cout << "Vector_Constraint_4vel_lower =" << Vector_Constraint_4vel_lower << std::endl;
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (1);  
    // Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (1);  

     // Vector_Constraint_Aconstraint_lower.setZero();
     // Vector_Constraint_Aconstraint_upper.setZero();
   // std::cout << "Vector_Constraint_Aconstraint_lower =" << Vector_Constraint_Aconstraint_lower << std::endl;
    // Max_distance_ = -1000;
      // Vector_Constraint_Aconstraint_lower(0) =  Max_distance_;
    //Vector_Constraint_Aconstraint_lower(0) =  -10000; 
      //Vector_Constraint_Aconstraint_upper(0) =  100000;


   /* Vector_Constraint_Aconstraint_lower(1) =  -0.00- Diff_DesAndCur(2);
    Vector_Constraint_Aconstraint_upper(1) =  0.00 - Diff_DesAndCur(2);

    Vector_Constraint_Aconstraint_lower(2) =  -0.00 - Diff_DesAndCur(5);
    Vector_Constraint_Aconstraint_upper(2) =  0.00 - Diff_DesAndCur(5);  */


  /*  Vector_Constraint_Aconstraint_lower(0) =  -0.45 - Diff_DesAndCur(0);
    Vector_Constraint_Aconstraint_upper(0) =  0.45 - Diff_DesAndCur(0);

    Vector_Constraint_Aconstraint_lower(1) =  -0.45 - Diff_DesAndCur(1);
    Vector_Constraint_Aconstraint_upper(1) =  0.45 - Diff_DesAndCur(1);

    Vector_Constraint_Aconstraint_lower(3) =  -0.45 - Diff_DesAndCur(3);
    Vector_Constraint_Aconstraint_upper(3) =  0.45  - Diff_DesAndCur(3);

    Vector_Constraint_Aconstraint_lower(4) =  -0.45 - Diff_DesAndCur(4);
    Vector_Constraint_Aconstraint_upper(4) =  0.45 - Diff_DesAndCur(4);; 
  */

    // std::cout<<"Max_distance_" << Max_distance_ <<std::endl;
    // std::cout << "Vector_Constraint_Aconstraint_upper =" << Vector_Constraint_Aconstraint_upper << std::endl;

     Eigen:: VectorXd Vector_Constraint_Aconstraint_lower (7);  
     Eigen:: VectorXd Vector_Constraint_Aconstraint_upper (7); 


    Vector_Constraint_Aconstraint_lower.setZero();
    Vector_Constraint_Aconstraint_upper.setZero();

    Vector_Constraint_Aconstraint_lower(0) =  Max_distance_;
    Vector_Constraint_Aconstraint_upper(0) =  100000;


    Vector_Constraint_Aconstraint_lower(1) =  -0.2- Diff_DesAndCur(0);
    Vector_Constraint_Aconstraint_upper(1) =  0.2- Diff_DesAndCur(0);

    Vector_Constraint_Aconstraint_lower(2) =  -0.2- Diff_DesAndCur(1);
    Vector_Constraint_Aconstraint_upper(2) =  0.2 - Diff_DesAndCur(1);

    Vector_Constraint_Aconstraint_lower(3) =  -0.00- Diff_DesAndCur(2);
    Vector_Constraint_Aconstraint_upper(3) =  0.00 - Diff_DesAndCur(2);

    Vector_Constraint_Aconstraint_lower(4) =  -0.2- Diff_DesAndCur(3);
    Vector_Constraint_Aconstraint_upper(4) =  0.2 - Diff_DesAndCur(3);

    Vector_Constraint_Aconstraint_lower(5) =  -0.2- Diff_DesAndCur(4);
    Vector_Constraint_Aconstraint_upper(5) =  0.2 - Diff_DesAndCur(4);

    Vector_Constraint_Aconstraint_lower(6) =  -0.00 - Diff_DesAndCur(5);
    Vector_Constraint_Aconstraint_upper(6) =  0.00 - Diff_DesAndCur(5); 


	USING_NAMESPACE_QPOASES

    qpOASES::real_t* H_qpoases;
    H_qpoases = (qpOASES::real_t*)malloc(H_.rows()* H_.cols()*sizeof(qpOASES::real_t)); //rows lie
    matrix_to_real(H_qpoases, H_, H_.rows(), H_.cols());

  	qpOASES::real_t* g_qpoases;
    Eigen::MatrixXd g_matrix = g_;
    g_qpoases = (qpOASES::real_t*)malloc(g_matrix.rows()*sizeof(qpOASES::real_t));
    matrix_to_real(g_qpoases, g_matrix, g_matrix.rows(), 1);    

    qpOASES::real_t* A_qpoases;
    A_qpoases = (qpOASES::real_t*)malloc(Eigen_4Constraint_A.rows()*Eigen_4Constraint_A.cols()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(A_qpoases, Eigen_4Constraint_A, Eigen_4Constraint_A.rows(), Eigen_4Constraint_A.cols());
    
    qpOASES::real_t* lb_qpoases;
    Eigen::MatrixXd lower_velocity_joint_matrix = Vector_Constraint_4pos_lower;
    //cout << lower_velocity_joint_matrix << endl;
    lb_qpoases = (qpOASES::real_t*)malloc(lower_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	   matrix_to_real(lb_qpoases, lower_velocity_joint_matrix, lower_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* ub_qpoases;
    Eigen::MatrixXd upper_velocity_joint_matrix = Vector_Constraint_4pos_upper;
    ub_qpoases = (qpOASES::real_t*)malloc(upper_velocity_joint_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ub_qpoases, upper_velocity_joint_matrix, upper_velocity_joint_matrix.rows() , 1);

    qpOASES::real_t* lbA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_lower_matrix = Vector_Constraint_Aconstraint_lower;
    lbA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_lower_matrix.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(lbA_qpoases, Vector_Constraint_Aconstraint_lower_matrix, Vector_Constraint_Aconstraint_lower_matrix.rows() , 1);

    qpOASES::real_t* ubA_qpoases;
    Eigen::MatrixXd Vector_Constraint_Aconstraint_upper_matrix = Vector_Constraint_Aconstraint_upper;
    ubA_qpoases = (qpOASES::real_t*)malloc(Vector_Constraint_Aconstraint_upper.rows()*sizeof(qpOASES::real_t)); //rows lie
	  matrix_to_real(ubA_qpoases, Vector_Constraint_Aconstraint_upper_matrix, Vector_Constraint_Aconstraint_upper.rows() , 1);


   // QProblem example( Number_Joint+Number_Joint, 1);
   // QProblem example( Number_Joint+Number_Joint, 3);   
   // QProblem example( Number_Joint+Number_Joint, 5);
   QProblem example( Number_Joint+Number_Joint, 7);   
    Options options;
    example.setOptions( options );
    real_t cputime = 1.0;
    /* Solve first QP. */
    int_t nWSR = 100000;
    example.init( H_qpoases,g_qpoases,A_qpoases,lb_qpoases,ub_qpoases,lbA_qpoases,ubA_qpoases, nWSR, &cputime);

    /* Get and print solution of first QP. */
    real_t xOpt[Number_Joint+Number_Joint];
    real_t yOpt[1];
    example.getPrimalSolution( xOpt );
    //example.getDualSolution( yOpt );

   if(example.getObjVal() == INFTY)
   {

    // return 0;
   }


	printf( "\nxOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal());
	printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 
  std::cout << "changing" << std::endl;

  Eigen::VectorXd Middele(Number_Joint+Number_Joint);
  Middele.setZero();
  Changing_Position = Middele;
  for(int i=0; i< Number_Joint+Number_Joint;i++)
  {
    Changing_Position(i) = xOpt[i];
  }



}

Eigen::MatrixXd MPC_OPTIMAL::Grad_Taylor_Relax_dualarm(int Nearest_Link_left, int Nearest_Link_right, 
                                                      std::string left_Arm_Link,  std::string right_Arm_Link,
                                                      Eigen::MatrixXd Jacobian_left, Eigen::MatrixXd Jacobian_right,
                                                      Eigen::VectorXd Joint_Position_current_left, Eigen::VectorXd Joint_Position_current_right, 
                                                      Eigen::Vector3d nearest_points_left, Eigen::Vector3d nearest_points_right,
                                                      KDL::Tree _tree, double distance,
                                                      double& Max_distance_)

{


   Eigen::MatrixXd Jacobian_Nearest_left;
   Jacobian_Nearest_left = Get_Jacobian_4Point(Nearest_Link_left, left_Arm_Link, Joint_Position_current_left, nearest_points_left, _tree);
   Eigen::MatrixXd Jacobian_Nearest_right;
   Jacobian_Nearest_right = Get_Jacobian_4Point(Nearest_Link_right, right_Arm_Link, Joint_Position_current_right, nearest_points_right, _tree);
   
   Eigen::MatrixXd Middle_matrix_left = Jacobian_left.topRows(3);
   Middle_matrix_left.setZero();
   Eigen::MatrixXd Middle_matrix_right = Jacobian_right.topRows(3);
   Middle_matrix_right.setZero();


   Middle_matrix_left.leftCols(Jacobian_Nearest_left.cols())= Jacobian_Nearest_left.block(0,0, 3, Jacobian_Nearest_left.cols());
   Middle_matrix_right.leftCols(Jacobian_Nearest_right.cols())= Jacobian_Nearest_right.block(0,0, 3, Jacobian_Nearest_right.cols());

   Eigen::MatrixXd Middle_matrix(Middle_matrix_left.rows(), Middle_matrix_right.cols() + Middle_matrix_left.cols());
   Middle_matrix.rightCols(Middle_matrix_right.cols()) = -Middle_matrix_right;
   Middle_matrix.leftCols(Middle_matrix_left.cols()) = Middle_matrix_left;

   Eigen::MatrixXd Vector_Error_close = nearest_points_left - nearest_points_right;
   
  
   Eigen::MatrixXd Matrix_distance_A;
   Matrix_distance_A = Vector_Error_close.transpose() * Middle_matrix;
   
   // std::cout << "Matrix_distance_A=" << Matrix_distance_A << std::endl;

   Max_distance_ = 0.015;
   Max_distance_ = Max_distance_ - distance;

   return Matrix_distance_A;
}

void MPC_OPTIMAL::Relatvie_Orientation(Eigen::VectorXd current_dir_frameOri, Eigen::MatrixXd& Rotation_matrix)
{
    Eigen::VectorXd current_dir_frameOri_norm = current_dir_frameOri.normalized();
    Eigen::VectorXd axis_function(3);
    Eigen::Vector3d Z_(0,0,1);
    
    axis_function.setZero();
    
    if(current_dir_frameOri_norm(0)==0)
    {
      axis_function(0)=1;
    }
    
    else
    {
      axis_function(0)=1;
      axis_function(1)=-current_dir_frameOri_norm(0)/current_dir_frameOri_norm(1);
    }

    double relative_crodd_product;
    relative_crodd_product = current_dir_frameOri_norm.dot(Z_);
    double theita_;
    theita_ = acos(relative_crodd_product);
    
    std::cout << relative_crodd_product << "=" << theita_ << std::endl;

    Rotation_matrix = Tramsform_axis_2rotationMarix(axis_function(0), axis_function(1), axis_function(2) , theita_);

}

Eigen::Matrix3d MPC_OPTIMAL::Tramsform_axis_2rotationMarix(double x, double y, double z, double angle)
{
Eigen::Vector3d vector(x, y, z);//旋转轴
//double angle;//旋转角度
Eigen::AngleAxisd rotationVector(angle, vector.normalized());
Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();
rotationMatrix = rotationVector.toRotationMatrix();//所求旋转矩阵
return rotationMatrix;
}


double MPC_OPTIMAL::cost_function(double distance_cost, double MPC_cost)
{
double cost_ = 0;
double weight = 20;
cost_ = distance_cost - MPC_cost;
return cost_;
}
















void MPC_OPTIMAL:: costFunction_Velocity_IK_solve(int Number_Joint,  Eigen::VectorXd current_end_position ,
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


void MPC_OPTIMAL::Constraint_4modular_IK(int Number_Joint, Eigen::VectorXd position_joint_current, 
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



void MPC_OPTIMAL::IK_solution(Eigen::MatrixXd Jacobian_4Robot_End, 
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

   cost_int_ = example.getObjVal();

	 printf( "\n xOpt = [ %e, %e , %e, %e, %e, %e];    objVal = %e\n\n"  , 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],example.getObjVal() );
	 printf( "CPU time:  %.3f microseconds\n\n", cputime*1.0e6 ); 

   for(int i =0 ; i < Number_Joint; i++)
      First_Result(i) = xOpt[i];
   // std::cout << First_Result << std::endl;  
      // std::cout << "yOpt[] = " << yOpt[i] << std::endl


}



Eigen::VectorXd MPC_OPTIMAL::solvIK(Eigen::VectorXd Position_disire_, std::string path_urdf ,Eigen::VectorXd& First_Result)
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
    

    std::cout << cartpos << std::endl;
    std::cout << Position_disire_(0) - cartpos.p(0) << std::endl;
    std::cout << Position_disire_(1) - cartpos.p(1) << std::endl;
    std::cout << Position_disire_(2)-  cartpos.p(2) << std::endl;    
    
    return Joint_Position_current;
}