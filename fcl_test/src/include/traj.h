#ifndef _TRAJ_H_
#define _TRAJ_H_
#include <fstream>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <stdio.h>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Sparse>
//#include <qpOASES.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"

#include <unsupported/Eigen/MatrixFunctions>

#include </home/mlei/catkin_ws/traj_generate/src/src/include/trajectory_generator_waypoint.h>

#include <Eigen/Geometry> 
class traj{

private:

TrajectoryGeneratorWaypoint *_trajGene;

public:

traj();
~traj();

Eigen::VectorXd Online_Point_;

ros::Publisher tra_visual_;
ros::Publisher _traj_vis_online;
ros::Publisher val_visual_;
ros::Publisher sphere_visual_;
ros::Publisher sphere_visual_Random;
ros::Publisher odom_pub;
//ros::Publisher posePub;

Eigen::MatrixXd _polyCoeff;
Eigen::VectorXd _polyTime;

Eigen::MatrixXd init_Orientation_Traj;
Eigen::MatrixXd Rotation_Error_Traj;

double time_duration;
double dt_orientation;

Eigen::Vector3d start_vel;


std::string represent_base;

double dt;
int _dev_order;
double altitude;
double _Vel;
double _Acc;

Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path);

Eigen::Vector3d getTrajPos(double t_cur);

Eigen::Vector3d getVel(double t_cur);

double getTrajTime( Eigen::Vector3d pos_);

void visTrajectory(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);

void visTrajectoryandOrientation(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time);

void visTrajectory_online(int number_);

void visEllipise(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time );

void visEllipise_Random(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time );

void trajOptimization(Eigen::MatrixXd path, Eigen::Vector3d vel);

bool trajGeneration(std::vector<Eigen::Vector3d> p, Eigen::Vector3d v);

void Orientation_Trajectory_Generation(double t_segment , Eigen::MatrixXd& Desire_Oritation);



};
#endif

