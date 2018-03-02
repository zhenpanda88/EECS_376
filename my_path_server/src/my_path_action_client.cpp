//my_path_action_client:

#include <ros/ros.h>
#include <my_path_server/pathAction.h> // this message type is defined in the current package
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <std_msgs/Bool.h>

bool lidar_alarm = false;
const double pi = 3.14;


void doneCb(const actionlib::SimpleClientGoalState& state, const my_path_server::pathResultConstPtr& result){
    ROS_INFO("Done");
}

void alarm_Cb(const std_msgs::Bool& message_holder){
    if(!lidar_alarm){
        lidar_alarm = message_holder.data;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_path_action_client");
    ros::NodeHandle n;
    ros::Subscriber alarm = n.subscribe("lidar_alarm", 1, alarm_Cb); //subscribes to lidar alarm

    bool goal_cancel = false;

    my_path_server::pathGoal goal;
    my_path_server::pathGoal cancellation;
    
    actionlib::SimpleActionClient<my_path_server::pathAction> action_client("path_server", true);

    ROS_INFO("waiting for server: ");
    
    bool server_exists = action_client.waitForServer(ros::Duration(5.0)); 
    
    while (!server_exists) {
      ROS_INFO("could not connect to server; halting");
      return 0;
    }
    
    ROS_INFO("connected client to server");
    
    double next_x_move = 0.0;
    double next_y_move = 0.0;
    double next_orientation = 0.0;
    
    goal.x_values.push_back(next_x_move);
    goal.y_values.push_back(next_y_move);
    goal.phi_values.push_back(next_orientation);

    //Pose 1
    next_x_move = 3.0;
    next_y_move = 0.0;
    next_orientation = 0.0;
    goal.x_values.push_back(next_x_move);
    goal.y_values.push_back(next_y_move);
    goal.phi_values.push_back(next_orientation);
    
    //Pose 2
    next_x_move = 0.0;
    next_y_move = 3.0;
    next_orientation = pi/2.0;
    
    goal.x_values.push_back(next_x_move);
    goal.y_values.push_back(next_y_move);
    goal.phi_values.push_back(next_orientation);
    
    //Pose 3
    next_x_move = 5.0;
    next_y_move = 0.0;
    next_orientation = 0;
    
    goal.x_values.push_back(next_x_move);
    goal.y_values.push_back(next_y_move);
    goal.phi_values.push_back(next_orientation);
    
    //Pose 4
    next_x_move = 0.0;
    next_y_move = 1.0;
    next_orientation = pi/2.0;
    
    goal.x_values.push_back(next_x_move);
    goal.y_values.push_back(next_y_move);
    goal.phi_values.push_back(next_orientation);
    

    action_client.sendGoal(goal, &doneCb);
    ROS_INFO("Goal message constructed and sent");

    while(true){
        bool completed = action_client.waitForResult(ros::Duration(0.5));

        if(completed){
            ROS_INFO("Goal Reached");
            
        }

        ROS_INFO("Checking for alarm state");
        if(lidar_alarm){
            action_client.cancelGoal();
            goal_cancel = true;
            ROS_INFO("Cancelling Goal");

        }
        ros::spinOnce();
        if(goal_cancel){
            ROS_INFO("Goal cancelled");

            next_orientation = 0.0;
            next_x_move = 0.0;
            next_y_move = 0.0;
            
            cancellation.x_values.push_back(next_x_move);
            cancellation.y_values.push_back(next_y_move);
            cancellation.phi_values.push_back(next_orientation);

            action_client.sendGoal(cancellation, &doneCb);

            next_orientation = pi/2.0;
            cancellation.x_values.push_back(next_x_move);
            cancellation.y_values.push_back(next_y_move);
            cancellation.phi_values.push_back(next_orientation);

            next_orientation = pi;
            cancellation.x_values.push_back(next_x_move);
            cancellation.y_values.push_back(next_y_move);
            cancellation.phi_values.push_back(next_orientation);

            next_orientation = pi/2.0;
            cancellation.x_values.push_back(next_x_move);
            cancellation.y_values.push_back(next_y_move);
            cancellation.phi_values.push_back(next_orientation);

            next_orientation = 0.0;
            cancellation.x_values.push_back(next_x_move);
            cancellation.y_values.push_back(next_y_move);
            cancellation.phi_values.push_back(next_orientation);

            action_client.sendGoal(cancellation, &doneCb);
        }
    }

    



    return 0;
}
