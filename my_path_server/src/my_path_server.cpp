//my_path_server

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <my_path_server/pathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <math.h>
#include <cmath>
using namespace std;

//some tunable constants, global
const double g_move_speed = 1.0; // set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 1.0; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm

//global variables, including a publisher object
geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; 
geometry_msgs::Pose g_current_pose;
const double pi = 3.1415;

class MyPathServer {
private:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<my_path_server::pathAction> as_;
	
    my_path_server::pathGoal goal_;
	my_path_server::pathResult result_;
	my_path_server::pathFeedback feedback_;
	
    double sgn(double x);
	double min_spin(double spin_angle);
	double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
	geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);
	void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading);
	void do_halt();
	void do_move(double distance);
	void do_spin(double spin_ang);

public:
	MyPathServer();

	~MyPathServer(void){

	}

	void executeCB(const actionlib::SimpleActionServer<my_path_server::pathAction>::GoalConstPtr& goal);

};

MyPathServer::MyPathServer() :
	as_(nh_, "path_server", boost::bind(&MyPathServer::executeCB, this, _1),false)
	{
		ROS_INFO("in constructor of myPathServer...");
		as_.start();
	}

//signum function: strip off and return the sign of the argument
double MyPathServer::sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double MyPathServer::min_spin(double spin_angle) {
        if (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        if (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double MyPathServer::convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion MyPathServer::convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void MyPathServer::do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}

//a function to move forward by a specified distance (in meters), then halt
void MyPathServer::do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void MyPathServer::do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

//THIS FUNCTION IS NOT FILLED IN: NEED TO COMPUTE HEADING AND TRAVEL DISTANCE TO MOVE
//FROM START TO GOAL
void MyPathServer::get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose, double &dist, double &heading) {
 double x_start, x_goal, y_start, y_goal, dx, dy;
 
 x_start = current_pose.position.x;
 y_start = current_pose.position.y;
 x_goal = goal_pose.position.x;
 y_goal = goal_pose.position.y;

 dx = x_goal - x_start;
 dy = y_goal - y_start;

 dist = sqrt(dx*dx + dy*dy);

 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else {
    heading = atan2(dy,dx); 
 }

}



void MyPathServer::executeCB(const actionlib::SimpleActionServer<my_path_server::pathAction>::GoalConstPtr& goal){
ROS_INFO("callback activated");
    double theta_current = 0.0;//to track current orientation
    
    int npts = goal->x_values.size();//get number of poses to be visited
    double dist = 0.0;
    double delta_theta = 0.0;

    for(int i=0;i<npts;i++){
        dist = sqrt(pow(goal->x_values[i],2.0)+pow(goal->y_values[i],2.0));//calculate the next move distance
        delta_theta = (goal->phi_values[i]) - theta_current;//calculate change in angle
        do_spin(delta_theta);//assume correct angle
        theta_current = goal->phi_values[i];//update current joint angle
        do_move(dist);//command the move
        if (as_.isPreemptRequested()){  
            ROS_WARN("goal cancelled!");
            result_.goal_reached = false;
            as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
            return; // done with callback
        }
    }

    result_.goal_reached = true;
    as_.setSucceeded(result_);
}

int main(int argc, char** argv){
	ros::init(argc,argv, "my_path_server");
	ros::NodeHandle nh;

	ROS_INFO("instantianting the Path Server: ");

    g_twist_commander = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);

	MyPathServer as_object;

	ROS_INFO("going into spin");

	ros::spin();
	
    return 0;
}