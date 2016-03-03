
#include <ros/ros.h>

#include <gazebo/math/Quaternion.hh>
#include <math.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>



void callback(const ros::TimerEvent&)
{
    ROS_INFO("fuck");
}


void getPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("fuck11111111111");
  ROS_INFO("fuck11111111111");
  ROS_INFO("fuck11111111111");
  ROS_INFO("fuck11111111111");
  
}

int main(int argc,char **argv)
{
    ros::init(argc,argv,"gettime");

    ros::NodeHandle n;

    // ros::Timer timer1=n.createTimer(ros::Duration(0.1),callback);
    ros::Subscriber sub = n.subscribe("/ground_truth_odom", 10, getPose);
    ros::spin();

    return 0;
}
