// wsn example program to illustrate LIDAR processing.  1/23/15

#include <ros/ros.h> //Must include this for all ROS cpp projects
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h> //Including the Float32 class from std_msgs
#include <std_msgs/Bool.h> // boolean message 


const double MIN_SAFE_DISTANCE_FRONT = 1.0; // set alarm if anything is within 0.5m of the front of robot
const double MIN_SAFE_DISTANCE_LEFT = 0.6;
const double MIN_SAFE_DISTANCE_RIGHT = 0.6;
const double MIN_SAFE_DISTANCE_RIGHT_45 = 0.75;
const double MIN_SAFE_DISTANCE_RIGHT_30 = 0.8;
const double MIN_SAFE_DISTANCE_LEFT_45 = 0.75;
const double MIN_SAFE_DISTANCE_LEFT_30 = 0.8;
// these values to be set within the laser callback
float ping_dist_in_front_=3.0; // global var to hold length of a SINGLE LIDAR ping--in front
float ping_dist_left = 3.0;
float ping_dist_right = 3.0;
float ping_dist_left_45 = 3.0;
float ping_dist_right_45 = 3.0;
float ping_dist_left_30 = 3.0;
float ping_dist_right_30 = 3.0;
float ping_test = 3.0;
int ping_front_= -1; // NOT real; callback will have to find this
double angle_min_=0.0;
double angle_max_=0.0;
double angle_increment_=0.0;
double range_min_ = 0.0;
double range_max_ = 0.0;
int index_size = 0;
int ping_left = 0;
int ping_right = 0;
int ping_left_45 = 0;
int ping_right_45 = 0;
int ping_right_30 = 0;
int ping_left_30 = 0;
bool laser_alarm_=false;

ros::Publisher lidar_alarm_publisher_;
ros::Publisher lidar_dist_publisher_;
// really, do NOT want to depend on a single ping.  Should consider a subset of pings
// to improve reliability and avoid false alarms or failure to see an obstacle

void laserCallback(const sensor_msgs::LaserScan& laser_scan) {
    if (ping_front_<0)  {
        //for first message received, set up the desired index of LIDAR range to eval
        angle_min_ = laser_scan.angle_min;
        angle_max_ = laser_scan.angle_max;
        angle_increment_ = laser_scan.angle_increment;
        range_min_ = laser_scan.range_min;
        range_max_ = laser_scan.range_max;
        // what is the index of the ping that is straight ahead?
        // BETTER would be to use transforms, which would reference how the LIDAR is mounted;
        // but this will do for simple illustration
        ping_front_ = (int) ((0.0 -angle_min_)/angle_increment_);
        index_size = ((angle_max_ - angle_min_)/angle_increment_);
        ping_right = (int)((-1.5708 - angle_min_)/angle_increment_);
        ping_right_45 = (int)((-0.785398 - angle_min_)/angle_increment_);
        ping_right_30 = (int)((-0.523599 - angle_min_)/angle_increment_);
        ping_left = (int)(index_size - ((angle_max_- 1.5708)/angle_increment_));
        ping_left_45 = (int)(index_size - ((angle_max_- 0.785398)/angle_increment_));        
        ping_left_30 = (int)(index_size - ((angle_max_- 0.523599)/angle_increment_));
        ROS_INFO("\nLIDAR setup:\nfront_ping_index = %d;\nleft_ping_index = %d;\nleft_ping_45_index = %d\nleft_ping_30_index = %d\nright_ping_index = %d;\nright_ping_30_index = %d\nright_ping_45_index = %d\nindex_size = %d;",ping_front_, ping_left, ping_left_45, ping_left_30, ping_right, ping_right_30, ping_right_45, index_size);
    }
   
   ping_dist_in_front_ = laser_scan.ranges[ping_front_];
   ping_dist_left = laser_scan.ranges[ping_left];
   ping_dist_left_45 = laser_scan.ranges[ping_left_45];
   ping_dist_left_30 = laser_scan.ranges[ping_left_30];
   ping_dist_right = laser_scan.ranges[ping_right];
   ping_dist_right_45 = laser_scan.ranges[ping_right_45];
   ping_dist_right_30 = laser_scan.ranges[ping_right_30];
   //ROS_INFO("left=%g; left_45=%g; front=%g; right_45=%g; right=%g", ping_dist_left, ping_dist_left_45, ping_dist_in_front_, ping_dist_right_45, ping_dist_right);
   
   if (ping_dist_in_front_<MIN_SAFE_DISTANCE_FRONT) {
       ROS_WARN("Warning: Obstacle ahead\n					Front = %g", ping_dist_in_front_);
       laser_alarm_=true;
   }
   else if(ping_dist_left<MIN_SAFE_DISTANCE_LEFT){
   		ROS_WARN("Warning: Obstacle left\n					L_90 = %g", ping_dist_left);
   		laser_alarm_=true;
   }
   else if(ping_dist_left_45<MIN_SAFE_DISTANCE_LEFT_45){
   	   	ROS_WARN("Warning: Obstacle left\n					L_45 = %g", ping_dist_left_45);
   		laser_alarm_=true;
   }
   else if(ping_dist_left_30<MIN_SAFE_DISTANCE_LEFT_30){
   	   	ROS_WARN("Warning: Obstacle left\n 					L_30 = %g", ping_dist_left_30);
   		laser_alarm_=true;
   }
   else if(ping_dist_right<MIN_SAFE_DISTANCE_RIGHT){
   		ROS_WARN("Warning: Obstacle right\n 				R_90 = %g", ping_dist_right);
   		laser_alarm_=true;
   }
   else if(ping_dist_right_45<MIN_SAFE_DISTANCE_RIGHT_45){
   	   	ROS_WARN("Warning: Obstacle right\n 				R_45 = %g", ping_dist_right_45);
   		laser_alarm_=true;
   }
   else if(ping_dist_right_30<MIN_SAFE_DISTANCE_RIGHT_30){
   	   	ROS_WARN("Warning: Obstacle right\n 				R_30 = %g", ping_dist_right_30);
   		laser_alarm_=true;
   }
   else {
       laser_alarm_=false;
   }
   std_msgs::Bool lidar_alarm_msg;
   lidar_alarm_msg.data = laser_alarm_;
   lidar_alarm_publisher_.publish(lidar_alarm_msg);
   std_msgs::Float32 lidar_dist_msg;
   lidar_dist_msg.data = ping_dist_in_front_;
   lidar_dist_publisher_.publish(lidar_dist_msg);   
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "revised_lidar_alarm"); //name this node
    ros::NodeHandle nh; 
    //create a Subscriber object and have it subscribe to the lidar topic
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("lidar_alarm", 1);
    lidar_alarm_publisher_ = pub; // let's make this global, so callback can use it
    ros::Publisher pub2 = nh.advertise<std_msgs::Float32>("lidar_dist", 1);  
    lidar_dist_publisher_ = pub2;
    ros::Subscriber lidar_subscriber = nh.subscribe("robot0/laser_0", 1, laserCallback);
    //ros::Subscriber lidar_subscriber_1 = nh.subscribe("robot0/laser_1", 1, laserCallback);
    ros::spin(); //this is essentially a "while(1)" statement, except it
    // forces refreshing wakeups upon new data arrival
    // main program essentially hangs here, but it must stay alive to keep the callback function alive
    return 0; // should never get here, unless roscore dies
}

