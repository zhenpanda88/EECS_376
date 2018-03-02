//my_path_client:

#include <ros/ros.h>
#include <my_path_service/PathSrv.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_path_client");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<my_path_service::PathSrv>("my_path_service");
    geometry_msgs::Quaternion quat;
    
    while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");
    my_path_service::PathSrv path_srv;
    
    //create some path points...this should be done by some intelligent algorithm, but we'll hard-code it here
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;
    
    //pose 0
    pose.position.x = 3.2; // say desired x-coord is 1
    pose.position.y = 0.0;
    pose.position.z = 0.0; // let's hope so!
    pose.orientation.x = 0.0; //always, for motion in horizontal plane
    pose.orientation.y = 0.0; // ditto
    pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
    pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
    pose_stamped.pose = pose;
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    //pose 1
    quat = convertPlanarPhi2Quaternion(5.42); 
    pose_stamped.pose.orientation = quat;   
    pose_stamped.pose.position.y= 2.91; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    //pose 2
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y = 0.0; 
    pose_stamped.pose.position.x = 0.0; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    
    //pose 3
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y = 0.0; 
    pose_stamped.pose.position.x = 8.2; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    //pose 4
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;  
    pose_stamped.pose.position.y = 4.0; 
    pose_stamped.pose.position.x = 0.1; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    //pose 5
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = 1.0; 
    pose_stamped.pose.position.x = 0.0; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    //pose 6
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = -2.2; 
    pose_stamped.pose.position.x = 0.0; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);

    //pose 7
    quat = convertPlanarPhi2Quaternion(0.0);
    pose_stamped.pose.orientation = quat;
    pose_stamped.pose.position.y = -2.0; 
    pose_stamped.pose.position.x = 0.0; 
    path_srv.request.nav_path.poses.push_back(pose_stamped);
    client.call(path_srv);

    return 0;
}
