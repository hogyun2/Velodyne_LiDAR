#include "ros/ros.h"
#include "../include/pp.h"

void clbk(const obstacle_detector::PangPang::ConstPtr& msg) {
    ROS_INFO("first point: x=%.2f, y=%.2f", msg->points[0].x, msg->points[0].y);
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "ppc");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<obstacle_detector::PangPang>("pangpang",10,clbk);

  ros::spin();

}
