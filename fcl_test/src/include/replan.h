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

//#include </home/oem/catkin_ws/traj_generate/src/src/include/trajectory_generator_waypoint.h>

#include <Eigen/Geometry> 
class replan{



public:

replan();
~replan();



void online_optimal(double t_segment , Eigen::MatrixXd& Desire_Oritation);



};
#endif

