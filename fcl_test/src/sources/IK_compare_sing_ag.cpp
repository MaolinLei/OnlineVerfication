
#include "ros/ros.h"
#include "math.h"
#include "ros/time.h"

#include <Eigen/Dense>
#include <cartesian_interface/ros/RosClient.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h> 
#include <cartesian_interface/CartesianInterfaceImpl.h>

#include "geometry_msgs/PoseStamped.h"

//traj_generate
#include <sensor_msgs/JointState.h>
#include <traj.h>


#include <iostream>
#include <fstream>

#include <thread>

using namespace XBot::Cartesian;


Eigen::VectorXd Joint_state(5);
Eigen::VectorXd Joint_velcity(5);

std::ofstream velcity_files;   
std::ofstream position_files;   

Eigen::Vector3d current_position;  


void Topic_Subscrib(const sensor_msgs::JointStateConstPtr& msg)
{  
  
       Joint_state.setZero();
       Joint_velcity.setZero();
        // msg.header.stamp = ros::Time::now();    // Assign time
        // msg.position[0] = angle;
	   // msg.header.stamp = ros::Time::now();
       for(int i=0;i<5;i++)
       {
       Joint_state(i) = msg->position[i];
       }

       for(int i=0;i<5;i++)
       {
       Joint_velcity(i) = msg->velocity[i];
       }      

/*        if (velcity_files.is_open()) 
      {
            velcity_files << Joint_state(0) <<"\t"<<  Joint_state(1) <<"\t" << Joint_state(2) 
                           <<"\t" << Joint_state(3) <<"\t" << Joint_state(4)<<std::endl;
            }

        if (position_files.is_open()) 
      {
            position_files << Joint_velcity(0) <<"\t"<<  Joint_velcity(1) <<"\t" << Joint_velcity(2) 
                          <<"\t" << Joint_velcity(3) <<"\t" << Joint_velcity(4)<<std::endl;

               
            }
  */    // std::cout  << Joint_state << std::endl << std::endl;
      // std::cout  << Joint_velcity << std::endl << std::endl;
}



void Topic_Subscrib_robotend(const geometry_msgs::PoseStamped& msg)
{  
  
       current_position.setZero();
        // msg.header.stamp = ros::Time::now();    // Assign time
        // msg.position[0] = angle;
	   // msg.header.stamp = ros::Time::now();
       current_position(0) = msg.pose.position.x;
       current_position(1) = msg.pose.position.y;
       current_position(2) = msg.pose.position.z;     


       std::cout <<  current_position <<std::endl; 
}




int main(int argv, char **argc) 
{

    ros::init(argv, argc, "IK_compare");
    ros::NodeHandle n;
 

    ros::Subscriber topic_sub = n.subscribe("/cartesian/solution", 100, Topic_Subscrib);
    ros::Subscriber topic_sub_pos = n.subscribe("/geometry_msgs/PoseStamped", 100, Topic_Subscrib_robotend);


    RosClient cli;
    auto task = cli.getTask("TCP_gripper_A_ori");

    // ..use task
    task->setLambda(0.5);

    // ..convert to Cartesian task
    auto task_cartesian = std::dynamic_pointer_cast<CartesianTask>(task);

    double dt = 0.01;
    ros::Rate rate(1./dt);

    // if conversion was successful...

   
    if(task_cartesian)
    {
     
        Eigen::Affine3d Ttgt;


        ros::NodeHandle n;
        ros::NodeHandle n_;
        traj traj;
        traj.represent_base = "ci/base_link";
        traj.dt = 0.01;
        traj._Acc = 0.1;
        traj._Vel = 0.5;
        traj._dev_order = 4.0;
        traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
        traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
        traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   


        double time = 5.0;

            // fill Ttgt...
        task_cartesian->getPoseReference(Ttgt);


        Eigen::Vector3d p;
        Eigen::Vector3d v;
        v.setZero();
        std::vector<Eigen::Vector3d> p_list;
        p[0] =  Ttgt.translation().x() += 0.0;
        p[1] =  Ttgt.translation().y() += 0.0;
    // p[1] = cartpos.p(1);
        p[2] = Ttgt.translation().z() += 0.0;

        Ttgt.translation().x() += 0.4;

        p_list.push_back(p);

        p[0] =  Ttgt.translation().x();
        p[1] =  Ttgt.translation().y() ;
    // p[1] = cartpos.p(1);
        p[2] =  Ttgt.translation().z();


        p_list.push_back(p); 

        bool success = traj.trajGeneration(p_list,v);

        Eigen::VectorXd Online_Point((1000+1)*3);
        Online_Point.setZero();


            // command reaching motion
        task_cartesian->setPoseTarget(Ttgt, time);

        velcity_files.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/IK_compar_ikbased/IKcompare_HighPos/JointVeclotiy.txt");
        position_files.open("/home/mlei/catkin_ws/src/mpc/fcl_test/src/sources/IK_compar_ikbased/IKcompare_HighPos/JointPosition.txt");
            // sleep time
        const double sleep_dt = 0.01;

        // wait until motion started
        while(task_cartesian->getTaskState() == State::Online)
        {
            cli.update(0, 0);

            ros::spin();
         //   rate.sleep();
             ros::Duration(sleep_dt).sleep();
        }

        std::cout << "Motion started" << std::endl;

        // wait until motion completed
        while(task_cartesian->getTaskState() == State::Reaching)
        {
            cli.update(0, 0);

         /*   if (velcity_files.is_open()) {
                velcity_files << Joint_state(0) <<"\t"<<  Joint_state(1) <<"\t" << Joint_state(2) 
                               <<"\t" << Joint_state(3) <<"\t" << Joint_state(4)<<std::endl;
                }

            if (position_files.is_open()) {
                position_files << Joint_velcity(0) <<"\t"<<  Joint_velcity(1) <<"\t" << Joint_velcity(2) 
                               <<"\t" << Joint_velcity(3) <<"\t" << Joint_velcity(4)<<std::endl;

               
            }
         */     
            ros::spin();     
            ros::Duration(sleep_dt).sleep();

          //  rate.sleep();
        }

        std::cout << "Motion completed" << std::endl;
    }

 
}

// int main(int argc, char ** argv)
// {
//     RosClient cli;
//     ros::NodeHandle n;
//     ros::NodeHandle n_;
//     auto tasks = cli.getTaskList();

//     for(auto t : tasks) std::cout << t << "\n";

//    // std::cout << "Motion completed" << std::endl;   
//     auto task = cli.getTask("TCP_gripper_A_ori");
    
     
//     traj traj;
//     traj.represent_base = "ci/base_link";
//     traj.dt = 0.01;
//     traj._Acc = 0.1;
//     traj._Vel = 0.5;
//     traj._dev_order = 4.0;
//     traj.tra_visual_ = n.advertise<visualization_msgs::Marker>("trajectory",1,true);
//     traj._traj_vis_online = n.advertise<visualization_msgs::Marker>("trajectory_Online",1,true);   
//     traj.odom_pub = n.advertise<geometry_msgs::PoseArray>("poseStampedDsr",1,true);   



//      // ..use task
//      task->setLambda(0.5);

//      // ..convert to Cartesian task
//      auto task_cartesian = std::dynamic_pointer_cast<CartesianTask>(task);
     
//     Eigen::Affine3d Ttgt;
//     task_cartesian->getPoseReference(Ttgt);

//     Eigen::Vector3d p;
//     Eigen::Vector3d v;
//     v.setZero();
//     std::vector<Eigen::Vector3d> p_list;
//     p[0] =  Ttgt.translation().x() += 0.0;
//     p[1] =  Ttgt.translation().y() += 0.0;
//    // p[1] = cartpos.p(1);
//     p[2] = Ttgt.translation().z() += 0.0;

//     p_list.push_back(p);

//     p[0] =  Ttgt.translation().x() += 0.2;
//     p[1] =  Ttgt.translation().y() += 0.0;
//    // p[1] = cartpos.p(1);
//     p[2] = Ttgt.translation().z() += 0.0;


//     p_list.push_back(p); 

//     bool success = traj.trajGeneration(p_list,v);

    
//     int loop=0;

//     Eigen::VectorXd Online_Point((1000+1)*3);
//     Online_Point.setZero();
//     // if conversion was successful...



//     if(task_cartesian)
//     {  // fill Ttgt...
//         task_cartesian->getPoseReference(Ttgt);
//         Ttgt.translation().x() += 0.2;
//         double time = 5.0;
//         // command reaching motion
//         task_cartesian->setPoseTarget(Ttgt, time);

//         // sleep time
//         const double sleep_dt = 0.001;
//        // traj.Online_Point_ = Online_Point;

 

//         // wait until motion started
//         while(task_cartesian->getTaskState() == State::Online)
//         {
//             cli.update(0, 0);
//             ros::Duration(sleep_dt).sleep();

//             task_cartesian->getPoseReference(Ttgt);
//             //  for(int i = 0; i<3 ;i++)
//             {


//             //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
//             // cout << "Error_ = " << Error_.norm() << endl;
//             }            
//         }
   
//         std::cout << "Motion started" << std::endl;

//         // wait until motion completed
//         while(task_cartesian->getTaskState() == State::Reaching)
//         {
//             // Eigen::Affine3d T;
//             // task_cartesian->getCurrentPose(T);

//             std::cout << Ttgt.matrix() << std::endl;
//             task_cartesian->getPoseReference(Ttgt);
//             //  for(int i = 0; i<3 ;i++)
//             {
//             // Error_(i) = cartpos.p(i) - Position_disire_(i);
//             Online_Point(loop*3+0)=Ttgt.translation().x();
//             Online_Point(loop*3+1)=Ttgt.translation().y();
//             Online_Point(loop*3+2)=Ttgt.translation().z();
     
//             traj.visTrajectory_online(loop);
//             traj.Online_Point_ = Online_Point;
//             loop++; 
//             //Error_(i) = Desire_Rotation_VECTOR(i) - Rotation_op_VECTOR(i);
//             // cout << "Error_ = " << Error_.norm() << endl;
//             }    

//             cli.update(0, 0);
//             ros::Duration(sleep_dt).sleep();
//         }

//         loop++;
//         std::cout << "Motion completed" << std::endl;
//      }



// }




// using namespace XBot::Cartesian;

// int main(int argc, char **argv)
// {
//     /* Part 0: contructing the solver object */
//     ros::init(argc, argv, "MPC_modular_"); //Say to ROS the name of the node and the parameters

//     ros::NodeHandle nh_("xbotcore");

//     auto xbot_cfg = XBot::ConfigOptionsFromParamServer(nh_);



//     // an option structure which is needed to make a model

//     // XBot::ConfigOptions xbot_cfg;

//     // set the urdf and srdf path..
//     xbot_cfg.set_urdf_path("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf");
//     xbot_cfg.set_srdf_path("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/srdf/ModularBot.srdf");

//     // the following call is needed to generate some default joint IDs
//     xbot_cfg.generate_jidmap();

//     // some additional parameters..
//     xbot_cfg.set_parameter("is_model_floating_base", false);
//     xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
//     auto default_ctrl_mode = XBot::ControlMode::Position();
//     auto robot_control = XBot::RobotInterface::getRobot(xbot_cfg);

//     robot_control->setControlMode(default_ctrl_mode);
//    // auto default_ctrl_mode = XBot::ControlMode::Position();

//     // and we can make the model class
//     auto model = XBot::ModelInterface::getModel(xbot_cfg);
//     // robot->setControlMode(default_ctrl_mode);

//     Eigen::VectorXd qhome;
//     Eigen::VectorXd qhome_init(5);

//     qhome_init.setZero();
//     qhome = qhome_init;
//     // qhome_init(0) = 0.0;
//     // qhome_init(1) = 0.48;
//     // qhome_init(2) = 0.96;
//     // qhome_init(3) = 0.0;
//     // qhome_init(4) = 0.84;
//     model->getRobotState("home", qhome);
//     model->setJointPosition(qhome);
//     model->update();

//     // std::string path_to_config_file = XBot::Utils::getXBotConfig();
//     // XBot::RobotInterface::Ptr robot = XBot::RobotInterface::getRobot ( path_to_config_file ); 
//     // path_to_config_file = "/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/config/ModularBot.yaml";
//     // robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object
//     // std::cout << robot <<std::endl;
//     // std::cout << robot <<std::endl;
    
//     const double dt = 0.02;
//     auto ctx = std::make_shared<Context>(
//                 std::make_shared<Parameters>(dt),
//                 model
//             );

//     // load the ik problem given a yaml file
//     auto ik_pb_yaml = YAML::LoadFile("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/cartesio/ModularBot_cartesio_IK_config.yaml");
//     ProblemDescription ik_pb(ik_pb_yaml, ctx);

//     // we are finally ready to make the CartesIO solver "OpenSot"
//     auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
//                                                        ik_pb, ctx
//                                                        );


//     int current_state = 0; // hand-crafted finite state machine!
//     double time = 0;
//     Eigen::VectorXd q, qdot, qddot;

//     auto larm = solver->getTask("TCP_gripper_A_ori");


//     auto larm_cartesian = std::dynamic_pointer_cast<CartesianTask>(larm);
//     Eigen::Affine3d Tref;
//     larm_cartesian->getPoseReference(Tref);
//     std::cout << "Task current reference is \n" << Tref.matrix()  << std::endl;

//     std::cout << "Press ENTER to continue.. \n";
//     std::cin.ignore();

//     larm_cartesian->getPoseReference(Tref);

// while(true)
// {
//     // if(current_state == 0) // here we command a reaching motion
//     // {
//         std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;
//        // larm_cartesian->getPoseReference(Tref);
//        // larm_cartesian->getPoseReference(Tref);
//         Tref.translation()[0] += 0.001;
//         double target_time = 0.01;
//         larm_cartesian->setPoseTarget(Tref, target_time);

//         current_state++;
//     // }

//     // if(current_state == 1) // here we check that the reaching started
//     // {
//     //     if(larm_cartesian->getTaskState() == State::Reaching)
//     //     {
//     //         std::cout << "Motion started!" << std::endl;
//     //         current_state++;
//     //     }
//     // }

//     // if(current_state == 2) // here we wait for it to be completed
//     // {
//     //     if(larm_cartesian->getTaskState() == State::Online)
//     //     {
//     //         Eigen::Affine3d T;
//     //         larm_cartesian->getCurrentPose(T);

//     //         std::cout << "Motion completed, final error is " <<
//     //                       (T.inverse()*Tref).translation().norm() << std::endl;

//     //         current_state++;
//     //     }
//     // }

//     // if(current_state == 3) // here we wait the robot to come to a stop
//     // {
//     //     std::cout << "qdot norm is " << qdot.norm() << std::endl;
//     //     if(qdot.norm() < 1e-3)
//     //     {
//     //         std::cout << "Robot came to a stop, press ENTER to exit.. \n";
//     //         std::cin.ignore();
//     //         current_state++;
//     //     }

//     // }

//    // if(current_state == 4) break;

//     // update and integrate model state
//     solver->update(time, dt);



//     model->getJointPosition(q);
//     model->getJointVelocity(qdot);
//     model->getJointAcceleration(qddot);

//     q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
//     qdot += dt * qddot;

//     model->setJointPosition(q);
//     model->setJointVelocity(qdot);
//     model->update();

//     robot_control->setPositionReference(q);
//     robot_control->setVelocityReference(qdot);       
//         // robot_control->setEffortReference(tau);
//     robot_control->move();

//     std::this_thread::sleep_for(std::chrono::duration<double>(dt));
//     time += dt;
// }


//  //   std::cout<< "rbdl finish" << std::endl;
//     // initialize to a homing configuration
// //     Eigen::VectorXd qhome,qhomer;
// //     Eigen::VectorXd qhome_init(5);
// //     qhome_init.setZero();
// //     qhome_init(0) = 0.0;
// //     qhome_init(1) = 0.48;
// //     qhome_init(2) = 0.96;
// //     qhome_init(3) = 0.0;
// //     qhome_init(4) = 0.84;
// //    // model->getRobotState("home", qhome);
// //     qhome = qhome_init;
// //     model->setJointPosition(qhome);
// //     model->update();
    

// //     // before constructing the problem description, let us build a
// //     // context object which stores some information, such as
// //     // the control period
// //     const double dt = 0.01;
// //     auto ctx = std::make_shared<Context>(
// //                 std::make_shared<Parameters>(dt),
// //                 model
// //             );

// //     // load the ik problem given a yaml file
// //     auto ik_pb_yaml = YAML::LoadFile("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/cartesio/ModularBot_cartesio_IK_config.yamlf");
// //     ProblemDescription ik_pb(ik_pb_yaml, ctx);

// //     // we are finally ready to make the CartesIO solver "OpenSot"
// //     auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
// //                                                        ik_pb, ctx
// //                                                        );


// //     std::cout << "Solver constructed successfully \n";
// //     std::cout << "Press ENTER to continue.. \n";
// //     std::cin.ignore();


// //     /* Part 1: inspecting the defined tasks */

// //     // inspect list of defined tasks
// //     std::cout << "Defined tasks are: " << std::endl;

// //     for(auto tname : solver->getTaskList())
// //     {
// //         std::cout << " - " << tname << "\n";
// //     }

// //     std::cout << "Press ENTER to continue.. \n";
// //     std::cin.ignore();

// //     // inspect properties of "left_hand" task
// //     std::string task_name = "TCP_gripper_A";
// //     auto larm = solver->getTask(task_name);

// //     std::cout << "Task name is "             << larm->getName() << std::endl;
// //     std::cout << "Task type is "             << larm->getType() << std::endl;
// //     std::cout << "Task activation state is " << EnumToString(larm->getActivationState()) << std::endl;
// //     std::cout << "Task size is "             << larm->getSize() << std::endl;
// //     std::cout << "Task lambda is "           << larm->getLambda() << std::endl;
// //     std::cout << "Task weight is: \n "       << larm->getWeight() << std::endl;

// //     std::cout << "Press ENTER to continue.. \n";
// //     std::cin.ignore();

// //     // check that "left_hand" is actually a Cartesian type task
// //     auto larm_cartesian = std::dynamic_pointer_cast<CartesianTask>(larm);
// //     if(!larm_cartesian)
// //     {
// //         throw std::runtime_error("Unexpected task type!");
// //     }

// //     // inspect cartesian properties..
// //     std::cout << "Task distal link is "      << larm_cartesian->getDistalLink() << std::endl;
// //     std::cout << "Task base link is "        << larm_cartesian->getBaseLink() << std::endl;
// //     std::cout << "Task control mode is "     << EnumToString(larm_cartesian->getControlMode()) << std::endl;
// //     std::cout << "Task state is "            << EnumToString(larm_cartesian->getTaskState()) << std::endl;
// //     Eigen::Affine3d Tref;
// //     larm_cartesian->getPoseReference(Tref);
// //     std::cout << "Task current reference is \n" << Tref.matrix()  << std::endl;

// //     std::cout << "Press ENTER to continue.. \n";
// //     std::cin.ignore();


// //     /* Part 2: control loop */

// //     int current_state = 0; // hand-crafted finite state machine!
// //     double time = 0;
// //     Eigen::VectorXd q, qdot, qddot;
// //     Eigen::VectorXd qr, qdotr;
// //     while(true)
// //     {
// //         if(current_state == 0) // here we command a reaching motion
// //         {
// //             std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;

// //             larm_cartesian->getPoseReference(Tref);
// //             Tref.translation()[0] += 0.3;
// //             double target_time = 3.0;
// //             larm_cartesian->setPoseTarget(Tref, target_time);

// //             current_state++;
// //         }

// //         if(current_state == 1) // here we check that the reaching started
// //         {
// //             if(larm_cartesian->getTaskState() == State::Reaching)
// //             {
// //                 std::cout << "Motion started!" << std::endl;
// //                 current_state++;
// //             }
// //         }

// //         if(current_state == 2) // here we wait for it to be completed
// //         {
// //             if(larm_cartesian->getTaskState() == State::Online)
// //             {
// //                 Eigen::Affine3d T;
// //                 larm_cartesian->getCurrentPose(T);

// //                 std::cout << "Motion completed, final error is " <<
// //                               (T.inverse()*Tref).translation().norm() << std::endl;

// //                 current_state++;
// //             }
// //         }

// //         if(current_state == 3) // here we wait the robot to come to a stop
// //         {
// //             std::cout << "qdot norm is " << qdot.norm() << std::endl;
// //             if(qdot.norm() < 1e-3)
// //             {
// //                 std::cout << "Robot came to a stop, press ENTER to exit.. \n";
// //                 std::cin.ignore();
// //                 current_state++;
// //             }

// //         }

// //         if(current_state == 4) break;

// //         // update and integrate model state
// //         solver->update(time, dt);

// //         model->getJointPosition(q);
// //         model->getJointVelocity(qdot);
// //         model->getJointAcceleration(qddot);

// //         q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
// //         qdot += dt * qddot;

// //         model->setJointPosition(q);
// //         model->setJointVelocity(qdot);
// //         model->update();
        
// //         robot->setReferenceFrom(*model);
// //         robot->getPositionReference(qr);
// //         robot->getVelocityReference(qdotr);
        
// //         robot->setPositionReference(qr);
// //         robot->setVelocityReference(qdotr);
// //         robot->move();
        
        


// //         // roughly loop at 100 Hz
// //         std::this_thread::sleep_for(std::chrono::duration<double>(dt));
// //         time += dt;
// //     }


// } 