#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>

int main(int argc, char **argv){

  ros::init(argc, argv, "talker");
	
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int32>("/numbers", 10);
    
  ros::Rate loop_rate(10);
    
  int count = 0;   
  while(ros::ok()){    
    
    std_msgs::Int32 msg;
    
    msg.data = count;
        
    ROS_INFO("%d", msg.data);
        
    pub.publish(msg);
        
    ros::spinOnce();
        
    loop_rate.sleep();
    ++count;
  }
  
  return 0;
}
