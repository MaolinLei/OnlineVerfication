#include "ros/ros.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <qpOASES.hpp>
#include "fcl/config.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

#include <iostream>
#include <vector>
#include <algorithm>

using namespace std;
using namespace Eigen;

Eigen::MatrixXd _polyCoeff;
Eigen::VectorXd _polyTime;

int _dev_order, _min_order;



using namespace std;
using namespace Eigen;

int main()
{

double dt = 0.01; 

Vector3d expection_4Obj_Pos; 
VectorXd Covarious_4Obj_Pos;
Covarious_4Obj_Pos<< 0,0,1;
MatrixXd Eigen_Covarious_4Obj_Pos(Covarious_4Obj_Pos.asDiagonal());


Vector3d expection_4Obj_Vel; 
VectorXd Covarious_4Obj_Vel;
MatrixXd Eigen_Covarious_4Obj_Vel(Covarious_4Obj_Vel.asDiagonal());


Vector3d expection_4Obj_Pos_4Next; 
VectorXd Covarious_4Obj_Pos_4Next;

expection_4Obj_Pos_4Next = expection_4Obj_Pos + expection_4Obj_Vel *dt;
expection_4Obj_Pos_4Next = Covarious_4Obj_Pos + Covarious_4Obj_Vel *dt *dt;

MatrixXd Eigen_Covarious_4Obj_Pos_4Next(expection_4Obj_Pos_4Next.asDiagonal());

Vector3d axis_distance_on_;
Vector3d Center_4Ellipisoid;
axis_distance_on_(1) = expection_4Obj_Pos_4Next(1);
axis_distance_on_(2) = expection_4Obj_Pos_4Next(2);
axis_distance_on_(3) = expection_4Obj_Pos_4Next(3);

Center_4Ellipisoid = expection_4Obj_Pos_4Next;

cout << Eigen_Covarious_4Obj_Pos<< endl;

return 0;

}
