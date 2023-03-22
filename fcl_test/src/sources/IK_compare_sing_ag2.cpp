#include <Eigen/Dense>
#include <cartesian_interface/ros/RosClient.h>

#include <XBotInterface/RobotInterface.h>
#include <RobotInterfaceROS/ConfigFromParam.h> 
#include <cartesian_interface/CartesianInterfaceImpl.h>
//traj_generate

#include <traj.h>

#include <thread>

using namespace XBot::Cartesian;

// using namespace XBot::Cartesian;

int main(int argc, char **argv)
{
    /* Part 0: contructing the solver object */
    ros::init(argc, argv, "MPC_modular_"); //Say to ROS the name of the node and the parameters

    ros::NodeHandle nh_("xbotcore");

    auto xbot_cfg = XBot::ConfigOptionsFromParamServer(nh_);



    // an option structure which is needed to make a model

    // XBot::ConfigOptions xbot_cfg;

    // set the urdf and srdf path..
    xbot_cfg.set_urdf_path("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/urdf/ModularBot.urdf");
    xbot_cfg.set_srdf_path("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/srdf/ModularBot.srdf");

    // the following call is needed to generate some default joint IDs
    xbot_cfg.generate_jidmap();

    // some additional parameters..
    xbot_cfg.set_parameter("is_model_floating_base", false);
    xbot_cfg.set_parameter<std::string>("model_type", "RBDL");
    auto default_ctrl_mode = XBot::ControlMode::Position();
    auto robot_control = XBot::RobotInterface::getRobot(xbot_cfg);

    robot_control->setControlMode(default_ctrl_mode);
   // auto default_ctrl_mode = XBot::ControlMode::Position();

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(xbot_cfg);
    // robot->setControlMode(default_ctrl_mode);

    Eigen::VectorXd qhome;
    Eigen::VectorXd qhome_init(5);

    qhome_init.setZero();
    qhome = qhome_init;
    // qhome_init(0) = 0.0;
    // qhome_init(1) = 0.48;
    // qhome_init(2) = 0.96;
    // qhome_init(3) = 0.0;
    // qhome_init(4) = 0.84;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();

    // std::string path_to_config_file = XBot::Utils::getXBotConfig();
    // XBot::RobotInterface::Ptr robot = XBot::RobotInterface::getRobot ( path_to_config_file ); 
    // path_to_config_file = "/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/config/ModularBot.yaml";
    // robot = XBot::RobotInterface::getRobot ( path_to_config_file ); // returns a shared pointer to a robot object
    // std::cout << robot <<std::endl;
    // std::cout << robot <<std::endl;
    
    const double dt = 0.02;
    auto ctx = std::make_shared<Context>(
                std::make_shared<Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    auto ik_pb_yaml = YAML::LoadFile("/home/oem/catkin_ws/src/modularbots_fraunhofer/ModularBot_5DOF/cartesio/ModularBot_cartesio_IK_config.yaml");
    ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );


    int current_state = 0; // hand-crafted finite state machine!
    double time = 0;
    Eigen::VectorXd q, qdot, qddot;

    auto larm = solver->getTask("TCP_gripper_A_ori");


    auto larm_cartesian = std::dynamic_pointer_cast<CartesianTask>(larm);
    Eigen::Affine3d Tref;
    larm_cartesian->getPoseReference(Tref);
    std::cout << "Task current reference is \n" << Tref.matrix()  << std::endl;

    std::cout << "Press ENTER to continue.. \n";
    std::cin.ignore();

    larm_cartesian->getPoseReference(Tref);

while(true)
{
    if(current_state == 0) // here we command a reaching motion
    {
        std::cout << "Commanding left hand forward 0.3m in 3.0 secs" << std::endl;

        larm_cartesian->getPoseReference(Tref);
        Tref.translation()[0] += 0.3;
        double target_time = 3.0;
        larm_cartesian->setPoseTarget(Tref, target_time);

        current_state++;
    }

    if(current_state == 1) // here we check that the reaching started
    {
        if(larm_cartesian->getTaskState() == State::Reaching)
        {
            std::cout << "Motion started!" << std::endl;
            current_state++;
        }
    }

    if(current_state == 2) // here we wait for it to be completed
    {
        if(larm_cartesian->getTaskState() == State::Online)
        {
            Eigen::Affine3d T;
            larm_cartesian->getCurrentPose(T);

            std::cout << "Motion completed, final error is " <<
                          (T.inverse()*Tref).translation().norm() << std::endl;

            current_state++;
        }
    }

    if(current_state == 3) // here we wait the robot to come to a stop
    {
        std::cout << "qdot norm is " << qdot.norm() << std::endl;
        if(qdot.norm() < 1e-3)
        {
            std::cout << "Robot came to a stop, press ENTER to exit.. \n";
            std::cin.ignore();
            current_state++;
        }

    }

    if(current_state == 4) break;

    // update and integrate model state
    solver->update(time, dt);



    model->getJointPosition(q);
    model->getJointVelocity(qdot);
    model->getJointAcceleration(qddot);

    q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
    qdot += dt * qddot;

    model->setJointPosition(q);
    model->setJointVelocity(qdot);
    model->update();

    robot_control->setPositionReference(q);
    robot_control->setVelocityReference(qdot);       
        // robot_control->setEffortReference(tau);
    robot_control->move();

    std::this_thread::sleep_for(std::chrono::duration<double>(dt));
    time += dt;
}




} 