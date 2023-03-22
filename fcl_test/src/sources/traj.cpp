#include "traj.h"

using namespace std;
using namespace Eigen;


traj::traj()
{

}

traj::~traj()
{

}

Eigen::VectorXd traj::timeAllocation(Eigen::MatrixXd Path)
{
    Eigen::VectorXd time(Path.rows() - 1);

    int num = Path.rows() - 1;

    for(int i = 0; i < num; i++)
    {
        geometry_msgs::Point p_0;
        p_0.x = Path(i, 0);
        p_0.y = Path(i, 1);
        p_0.z = Path(i, 2);

        geometry_msgs::Point p_1;
        p_1.x = Path(i+1, 0);
        p_1.y = Path(i+1, 1);
        p_1.z = Path(i+1, 2);

        double delta_x = p_1.x - p_0.x;
        double delta_y = p_1.y - p_0.y;
        double delta_z = p_1.z - p_0.z;
        double path_length = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));

        double t1 = _Vel/_Acc;

        double t2 = path_length/_Vel - t1;

        time(i) = floor((2*t1 + t2)*100)/100;

    }

    return time;
}

Eigen::Vector3d traj::getTrajPos(double t_cur)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    double t_ = 0.0;

    for (int i = 0; i < _polyTime.size(); i++) {
      for (double t = 0.0; t < _polyTime(i); t += 0.01) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);

        if(t_cur - t_ < 0.001 )
        {
          return pos;
        }
        t_ = t_ +0.01;
      }
    }
    return pos;
}


double traj::getTrajTime(Eigen::Vector3d pos_)
{
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();

    double t_ = 0.0;

    for (int i = 0; i < _polyTime.size(); i++) {
      for (double t = 0.0; t < _polyTime(i); t += 0.01) {
        pos = _trajGene->getPosPoly(_polyCoeff, i, t);

       // std::cout << "(pos-pos_).norm()"<< (pos-pos_)<<std::endl <<std::endl;
      
      Eigen::VectorXd ju = (pos-pos_);
      if( ju(2)< 0.002 && ju(1)< 0.002 && ju(0)< 0.002)
      {
        return t_;    
        
      }
      t_ = t_ +0.01;
     }
    }
    return t_;

}

Eigen::Vector3d traj::getVel(double t_cur) {
  double time = 0;
  Eigen::Vector3d Vel = Eigen::Vector3d::Zero();
  for (int i = 0; i < _polyTime.size(); i++) {
    for (double t = 0.0; t < _polyTime(i); t += 0.01) { 
      Vel = _trajGene->getVelPoly(_polyCoeff, i, t);
      if (t_cur - time < 0.001) { 
        return Vel;
      }
      time = time + 0.01; 
    }
  }
  return Vel;
}


  void traj::visTrajectoryandOrientation(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)
{
    double draw_seg = 0.8;
    geometry_msgs::PoseArray odom;
    // odom.header.frame_id = "base_link";
    odom.header.frame_id =  represent_base;
    // odom.header.frame_id = "torso_2";
    odom.header.stamp = ros::Time::now();

     Eigen::Vector3d pos;
     Eigen::MatrixXd orienation;
     Eigen::MatrixXd Desire_Oritation_Traj;
     Eigen::Matrix3d Desire_Oritation_;
     Eigen::Quaterniond q_;
     
     geometry_msgs::Pose pt;
     double t_segment = 0;
    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += draw_seg) {
     // 
        pos = _trajGene->getPosPoly(polyCoeff, i, t);
        pt.position.x = pos(0);
        pt.position.y = pos(1);
        pt.position.z = pos(2);
     
        t_segment = t_segment + (draw_seg/dt) * dt_orientation;

        Orientation_Trajectory_Generation(t_segment, Desire_Oritation_Traj);

        Desire_Oritation_ = Desire_Oritation_Traj;
        q_ = Desire_Oritation_;
       // q_ = Desire_Oritation_;
        Eigen::VectorXd QXYZ;
        QXYZ = q_.coeffs();
        pt.orientation.x = QXYZ(0);
        pt.orientation.y = QXYZ(1);
        pt.orientation.z = QXYZ(2);
        pt.orientation.w = QXYZ(3);

        odom.poses.push_back(pt);
       
      }
    }
       
      
          odom_pub.publish(odom);
} 


 void traj:: Orientation_Trajectory_Generation(double t_segment,  Eigen::MatrixXd& Desire_Oritation)
{
  if(t_segment<=1)
  {
  Desire_Oritation =  init_Orientation_Traj * (t_segment * Rotation_Error_Traj).exp() ;
  }
  else
  {
  t_segment =1; 
  Desire_Oritation =  init_Orientation_Traj * (t_segment * Rotation_Error_Traj).exp() ;

  }
} 




void traj::visTrajectory(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time)
{
     visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    // _traj_vis.header.frame_id = "torso_2";
    // _traj_vis.header.frame_id = "base_link";   
    _traj_vis.header.frame_id = represent_base;
    // _traj_vis.header.frame_id = "base_link";   
    _traj_vis.ns = "trajectory";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.01;
    _traj_vis.scale.y = 0.01;
    _traj_vis.scale.z = 0.01;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 0.5;
    _traj_vis.color.b = 1.0;

    _traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += 0.01) {
        pos = _trajGene->getPosPoly(polyCoeff, i, t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
       // std::cout << "pt == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        _traj_vis.points.push_back(pt);
      }
    }
    tra_visual_.publish(_traj_vis);
}


void traj::visTrajectory_online(int number_)
{
     visualization_msgs::Marker _traj_vis;

    _traj_vis.header.stamp = ros::Time::now();
    //_traj_vis.header.frame_id = "torso_2";
    // _traj_vis.header.frame_id = "base_link";   
    _traj_vis.header.frame_id = represent_base;
    // _traj_vis.header.frame_id = "base_link";   
    _traj_vis.ns = "trajectory_Online";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::LINE_STRIP;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = 0.01;
    _traj_vis.scale.y = 0.01;
    _traj_vis.scale.z = 0.01;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 0.5;
    _traj_vis.color.g = 0.5;
    _traj_vis.color.b = 1.0;

    _traj_vis.points.clear();
    Eigen::Vector3d pos;
    geometry_msgs::Point pt;

    for (int i = 0; i < number_; i++) 
    {   
        pt.x = Online_Point_(0+i*3);
        pt.y = Online_Point_(1+i*3);
        pt.z = Online_Point_(2+i*3);
//        std::cout << "pt == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        _traj_vis.points.push_back(pt);
    }

    _traj_vis_online.publish(_traj_vis);
}




void traj::visEllipise(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time )
{
  visualization_msgs::MarkerArray _traj_vis;

  _traj_vis.markers.clear();

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    geometry_msgs::Point pt;
    geometry_msgs::Point pt_vel;

    geometry_msgs::Point pt_next;
    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += 0.08) {
        visualization_msgs::Marker cirecle;

        pos = _trajGene->getPosPoly(polyCoeff, i, t);
        pt.x = pos(0);
        pt.y = pos(1);
        pt.z = pos(2);
        //std::cout << "pt_ell == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        vel = _trajGene->getVelPoly(_polyCoeff, i, t);
        pt_vel.x = vel(0);
        pt_vel.y = vel(1);
        pt_vel.z = vel(2);
        //std::cout << "vt == " << " " << pt_vel.x << " " << pt_vel.y << " " << pt_vel.z << std::endl;
        //_traj_vis.points.push_back(pt); 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        Vector3d expection_4Obj_Pos; 
        expection_4Obj_Pos << pos(0), pos(1), pos(2) ; 
        Vector3d Covarious_4Obj_Pos;
        Covarious_4Obj_Pos<< 0.1,0.1,0.1;
        MatrixXd Eigen_Covarious_4Obj_Pos(Covarious_4Obj_Pos.asDiagonal());


        Vector3d expection_4Obj_Vel; 
        expection_4Obj_Vel << vel(0), vel(1), vel(2) ; 
        Vector3d Covarious_4Obj_Vel;
        Covarious_4Obj_Vel<< 0.1,0.1,0.01;
        MatrixXd Eigen_Covarious_4Obj_Vel(Covarious_4Obj_Vel.asDiagonal());


        Vector3d expection_4Obj_Pos_4Next; 
        Vector3d Covarious_4Obj_Pos_4Next;

        expection_4Obj_Pos_4Next = expection_4Obj_Pos + expection_4Obj_Vel *dt;
        Covarious_4Obj_Pos_4Next = Covarious_4Obj_Pos + Covarious_4Obj_Vel *dt *dt;

        MatrixXd Eigen_Covarious_4Obj_Pos_4Next(Covarious_4Obj_Pos_4Next.asDiagonal());

        Vector3d axis_distance_on_;
        Vector3d Center_4Ellipisoid;

        axis_distance_on_(0) = 4 * Covarious_4Obj_Pos_4Next(0);
        axis_distance_on_(1) = 4 * Covarious_4Obj_Pos_4Next(1);
        axis_distance_on_(2) = 4 * Covarious_4Obj_Pos_4Next(2);
        //std::cout << axis_distance_on_ <<std::endl;

        Center_4Ellipisoid = expection_4Obj_Pos_4Next; 
        ////////////////////////////////////////////////////////////////////////////////
        //cirecle.points.push_back(pt);
        cirecle.header.stamp = ros::Time::now();
        // cirecle.header.frame_id = "torso_2";
        
        // cirecle.header.frame_id = "base_link";
        cirecle.header.frame_id = represent_base;       
        cirecle.ns = "ProperVolume";
        cirecle.id = i+t*1000+1;
        cirecle.type = visualization_msgs::Marker::SPHERE;
        cirecle.action = visualization_msgs::Marker::ADD;
        cirecle.scale.x = axis_distance_on_(0);
        cirecle.scale.y = axis_distance_on_(1);
        cirecle.scale.z = axis_distance_on_(2);
        cirecle.pose.position.x = pt.x;
        cirecle.pose.position.y = pt.y;
        cirecle.pose.position.z = pt.z;
        cirecle.pose.orientation.x = 0.0;
        cirecle.pose.orientation.y = 0.0;
        cirecle.pose.orientation.z = 0.0;
        cirecle.pose.orientation.w = 1.0;

        cirecle.color.a = 0.75;
        cirecle.color.r = 0.5;
        cirecle.color.g = 0.5;
        cirecle.color.b = 1.0;

        _traj_vis.markers.push_back(cirecle);

        sphere_visual_.publish(cirecle);

        cirecle.points.clear();

        //ros::Duration(0.25).sleep();

      }
    }

    val_visual_.publish(_traj_vis);


 
} 



void traj::visEllipise_Random(Eigen::MatrixXd polyCoeff, Eigen::VectorXd time )
{
  visualization_msgs::MarkerArray _traj_vis;

  _traj_vis.markers.clear();

    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    geometry_msgs::Point pt;
    geometry_msgs::Point pt_vel;

    geometry_msgs::Point pt_next;
    srand( (unsigned)time( NULL ) );
    for (int i = 0; i < time.size(); i++) {
      for (double t = 0.0; t < time(i); t += 0.08) {
        visualization_msgs::Marker cirecle;
        if(t + dt < time(i))
        {
            pos = _trajGene->getPosPoly(polyCoeff, i, t + dt);
            //pt.x = pos(0) + 0.1 * (rand()-0.5);
            //pt.y = pos(1) + 0.1 * (rand()-0.5);
            //pt.z = pos(2) + 0.1 * (rand()-0.5);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            std::cout << "pt == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;

        }
        else{
            pos = _trajGene->getPosPoly(polyCoeff, i, t);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            std::cout << "pt == " << " " << pt.x << " " << pt.y << " " << pt.z << std::endl;
        }
        

        //std::cout << "vt == " << " " << pt_vel.x << " " << pt_vel.y << " " << pt_vel.z << std::endl;
        //_traj_vis.points.push_back(pt); 
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
       
        ////////////////////////////////////////////////////////////////////////////////
        //cirecle.points.push_back(pt);
        cirecle.header.stamp = ros::Time::now();
        // cirecle.header.frame_id = "torso_2";
        // cirecle.header.frame_id = "base_link";
        cirecle.header.frame_id = represent_base;     
        cirecle.ns = "ProperVolume_random";
        //cirecle.id = i+t*100000+1;
        cirecle.id = 1000001+1;
        cirecle.type = visualization_msgs::Marker::SPHERE;
        cirecle.action = visualization_msgs::Marker::ADD;
        cirecle.scale.x = 0.100;
        cirecle.scale.y = 0.100;
        cirecle.scale.z = 0.100;
        cirecle.pose.position.x = pt.x;
        cirecle.pose.position.y = pt.y;
        cirecle.pose.position.z = pt.z;
        cirecle.pose.orientation.x = 0.0;
        cirecle.pose.orientation.y = 0.0;
        cirecle.pose.orientation.z = 0.0;
        cirecle.pose.orientation.w = 1.0;

        cirecle.color.a = 0.75;
        cirecle.color.r = 0.5;
        cirecle.color.g = 0.5;
        cirecle.color.b = 1.0;

       // _traj_vis.markers.push_back(cirecle);

        sphere_visual_Random.publish(cirecle);

        cirecle.points.clear();

        //ros::Duration(0.25).sleep();

      }
    }

    //sphere_visual_Random.publish(_traj_vis);


 
} 


void traj::trajOptimization(Eigen::MatrixXd path, Eigen::Vector3d vel_)
{

  Eigen::MatrixXd vel = Eigen::MatrixXd::Zero(2, 3);
  Eigen::MatrixXd acc = Eigen::MatrixXd::Zero(2, 3);
  start_vel[0] = vel_[0];
  start_vel[1] = vel_[1];
  start_vel[2] = vel_[2];

  vel.row(0) = start_vel;

  _polyTime = timeAllocation(path);


  _polyCoeff =
        _trajGene->PolyQPGeneration(_dev_order, path, vel, acc, _polyTime);

// std::cout << "trajectory generation fail !!" << std::endl;

  if(std::isfinite(_polyCoeff(0,0)))
  {
    visTrajectory(_polyCoeff,_polyTime);
    //visEllipise (_polyCoeff,_polyTime);
    // visTrajectoryandOrientation(_polyCoeff,_polyTime);
  // std::cout << "trajectory generation fail !!" << std::endl;
  }
  else
  {
    std::cout << "trajectory generation fail !!" << std::endl;
  }
  
}



bool traj::trajGeneration(std::vector<Eigen::Vector3d> p, Eigen::Vector3d v)
{

  std::vector<Eigen::Vector3d> short_path;

  for(int num = 0; num < p.size(); num++)
  {
      short_path.push_back(p[num]);
  }

  Eigen::MatrixXd path_traj(int(short_path.size()), 3);
  for (int k = 0; k < int(short_path.size()); k++) {
        path_traj(k,0) = floor(short_path[k][0]*10000)/10000;
        path_traj(k,1) = floor(short_path[k][1]*10000)/10000;
        path_traj(k,2) = floor(short_path[k][2]*10000)/10000;
  }
  trajOptimization(path_traj,v);
  time_duration = _polyTime.sum();

  if (_polyCoeff.rows() > 0)
      return true;
  else
      return false;
}
