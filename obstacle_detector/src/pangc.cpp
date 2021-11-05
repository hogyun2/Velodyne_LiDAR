#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>


void numberCallback(const std_msgs::Int32::ConstPtr& msg){
   ROS_INFO("Recieved: [%d]", msg->data);
}

int main(int argc, char **argv){

   ros::init(argc, argv, "listner");
   
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("/numbers", 10, numberCallback);
   
   ros::spin();
   
   return 0;
}
