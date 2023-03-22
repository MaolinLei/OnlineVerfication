#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/LinkStates.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

    
ros::Publisher mark;
void poseCallback(const gazebo_msgs::LinkStates& msg){
/*      static tf::TransformBroadcaster br;
     tf::Transform transform;
     transform.setOrigin( tf::Vector3(msg.pose[7].position.x, msg.pose[7].position.y, msg.pose[7].position.z) );

     transform.setRotation(tf::Quaternion(msg.pose[7].orientation.x,msg.pose[7].orientation.y,
     msg.pose[7].orientation.z,msg.pose[7].orientation.w));
     br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "base_link")); */
     visualization_msgs::Marker marker;
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "world";

    marker.ns = "Frame";
    marker.id = 20;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.pose.position.x = msg.pose[7].position.x;
    marker.pose.position.y = msg.pose[7].position.y;
    marker.pose.position.z = msg.pose[7].position.z;

    marker.pose.orientation.x = msg.pose[7].orientation.x;
    marker.pose.orientation.y = msg.pose[7].orientation.y;
    marker.pose.orientation.z = msg.pose[7].orientation.z;
    marker.pose.orientation.w = msg.pose[7].orientation.w;

    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;

   mark.publish(marker);
   }

int main(int argc, char** argv){
     ros::init(argc, argv, "TF_Pub_End");
   
     ros::NodeHandle node;
     ros::Subscriber sub = node.subscribe("/gazebo/link_states", 10, &poseCallback);
      mark                = node.advertise<visualization_msgs::Marker >("End_frame",100);
     
      
     ros::spin();
     return 0;
   };