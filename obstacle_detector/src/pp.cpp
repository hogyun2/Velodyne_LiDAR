#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <obstacle_detector/PangPang.h>
#include <iostream>
#include <vector>
#include "../include/point.h"
#include "../include/pp.h"
using namespace obstacle_detector;

pp::pp() : ssibal_("") {
  pangpangsub=ssibal_.subscribe<obstacle_detector::Obstacles>("obstacles",10,&pp::pangpang,this);
  pangpang_pub_ = ssibal_.advertise<obstacle_detector::PangPang>("pangpang" ,10);

  ROS_INFO("jotgarnae [OK]");
  ros::spin();
}
void pp::pangpang(const obstacle_detector::Obstacles::ConstPtr& obstacles){
  p.clear();
  geometry_msgs::Point center_info;
  PangPang ppp;
  ppp.points.clear();
  ppp.centers.clear();
  for (auto c : obstacles->circles) {
    /*if(c.center.x == 0 && c.center.y == 0){
      p.push_back(Point(0,0));
      break;
    }*/
    center_info.x = c.center.x;
    center_info.y = c.center.y;
    ppp.centers.push_back(center_info);
    for(double s=45;s!=360;){
      
      double p_x=0;
      double p_y=0;

      p_x = c.radius*cos(s)+c.center.x;
      p_y = c.radius*sin(s)+c.center.y;

      p.push_back(Point(p_x,p_y));
      s=s+45;
    }
  }
  ppp.circle_number = obstacles->circle_number;
  // for (auto s : obstacles->segments) {
  //   for(double ss=0.125;ss!=1.000;){
  //     double p_x=0;
  //     double p_y=0;      

  //     p_x = (s.first_point.x+s.last_point.x)*ss;
  //     p_y = (s.first_point.y+s.last_point.y)*ss;

  //     p.push_back(Point(p_x,p_y));
  //     ss=ss+0.125;  
  //   }
  // }
  int i = 0;
  for (std::vector<Point>::iterator it = p.begin(); it != p.end(); ++it) {
        geometry_msgs::Point point;
        point.x = (*it).x;
        point.y = (*it).y;
        point.z = 0;
        ppp.points.push_back(point);
        i++;
    }

  pangpang_pub_.publish(ppp);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "pang");
  pp ov;
  return 0;
}
