from master_node.msg import Path, Serial_Info, Planning_Info, Local  # 개발할 메세지 타입
import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import cos
from math import atan2
from math import hypot
from math import atan2
from math import pi
from math import sqrt
# import LPP
import sys
import serial
import struct
import time
import matplotlib.pyplot as plt
# from hybrid_a_star import path_plan 
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Time
# from master_node.msg import BoundingBoxes
from master_node.msg import BoundingBox
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from std_msgs.msg import Int64
from master_node.msg import Obstacles

LEFT_MAX_STEER = 10
RIGHT_MAX_STEER = -10
BGEAR = 0x02
NGEAR = 0x01
FGEAR = 0x00
MAX_BRAKE = 200

class Control:
    def __init__(self):
        rospy.init_node("Control", anonymous=False)

        control_pub = rospy.Publisher("/control", Serial_Info, queue_size=1)
        self.pub_msg = Serial_Info()

        rospy.Subscriber("/serial", Serial_Info, self.serialCallback)
        self.serial_info = Serial_Info()
        
        # rospy.Subscriber("/Displacement_right", Int64, self.Get_Dis_right)

        rospy.Subscriber("/obstacles", Obstacles, self.get_Obstacles)

        self.visual_pub = rospy.Publisher("/visual_goal", PointCloud, queue_size=1)

        rate = rospy.Rate(50)  # 50hz

        #변수
        self.displacement_right = 0
        self.cur_data_right = 0
        self.dis_DR_flag = 0
        self.dis_DR_enc_right = 0
        self.velocity_enc = 0
        self.goal_point = [9999,9999]
        self.WB = 1.04

        # main loop
        while not rospy.is_shutdown():
            cur_x = 0
            cur_y = 0
            cur_yaw = 0

            self.pub_msg.speed = 0 
            self.pub_msg.steer = 0
            self.pub_msg.brake = 0
            self.pub_msg.encoder = 0
            self.pub_msg.gear = 0
            self.pub_msg.emergency_stop = 0
            self.pub_msg.auto_manual = 1

            self.mode_decision()
            self.pub_msg.steer = self.calc_steering(cur_x, cur_y, cur_yaw)
            self.pub_msg.speed = int(150)

            control_pub.publish(self.pub_msg)
            rate.sleep()

    def get_distance(self, point_set):
        return (point_set.x ** 2 + point_set.y ** 2) ** 0.5

    def get_Obstacles(self, msg):
        x_temp, y_temp = 0, 0
        
        dis_temp = {}
        point_temp = []
        final_point = []

        calc_a = 0

        c_x, c_y = 0, 0

        if len(msg.circles) <= 1 :
            self.goal_point = [1, 0]
        
        elif len(msg.circles) == 2 or len(msg.circles) == 3:
            for i in range(len(msg.circles)):
                x_temp += msg.circles[i].center.x
                y_temp += msg.circles[i].center.y
            self.goal_point = [x_temp / len(msg.circles), y_temp / len(msg.circles)]
        
        elif len(msg.circles) >= 4:
            # print(len(msg.circles))
            for i in range(len(msg.circles)):
                temp = {i : self.get_distance(msg.circles[i].center)}
                # print(temp)
                dis_temp.update(temp)
            sorted_dis = sorted(dis_temp.items(), key=(lambda x:x[1]))

            # get_value = msg.circles.center[sorted_dis[0][0]]

            # for p in range(4):
            #     temp = {p, msg.circles.center[sorted_dis[p][0]]-msg.circles.center[sorted_dis[0][0]]}
            #     point_temp.update(temp)
            # sorted_dis = sorted(dis_temp.items(), key=lambda x:atan2(x[1].y, x[1].x)%360)

            for i in range(4):
                index_temp = sorted_dis[i][0]
                c_x += msg.circles[index_temp].center.x / 4
                c_y += msg.circles[index_temp].center.y / 4
            #     point_temp.append([msg.circles[index_temp].center.x, msg.circles[index_temp].center.y])
            
            # for i in range(3):
            #     t_i, t_j = i, (i+1) % 4
            #     calc_a += (point_temp[t_i][0] * point_temp[t_j][1] - point_temp[t_j][0] * point_temp[t_i][1])/2

            # for i in range(3):
            #     t_i, t_j = i, (i+1) % 4
            #     c_x += (point_temp[t_i][0] + point_temp[t_j][0]) * (point_temp[t_i][0] * point_temp[t_j][1] - point_temp[t_j][0] * point_temp[t_i][1]) / (6*calc_a)
            #     c_y += (point_temp[t_i][1] + point_temp[t_j][1]) * (point_temp[t_i][0] * point_temp[t_j][1] - point_temp[t_j][0] * point_temp[t_i][1]) / (6*calc_a)
            
            self.goal_point = [c_x, c_y]
        
        visual_point = PointCloud()
        goal_point = Point32()
        goal_point.x, goal_point.y = self.goal_point[0], self.goal_point[1]
        visual_point.points.append(goal_point)
        visual_point.header.frame_id = 'world'
        visual_point.header.stamp = rospy.Time.now()

        self.visual_pub.publish(visual_point)



    def calc_steering(self, cur_x, cur_y, cur_yaw):
        goal_x, goal_y = self.goal_point[0], self.goal_point[1]
        steer = self.steering_angle(cur_x, cur_y, cur_yaw, goal_x, goal_y)
        return steer

    # def calc_speed(self):
    #     return speed

    def mode_decision(self):
        pass
        


    # Callback Function
    def serialCallback(self, msg):
        
        self.serial_info.encoder = msg.encoder
        self.serial_info.auto_manual = msg.auto_manual
        self.serial_info.gear = msg.gear
        self.serial_info.steer = msg.steer
        self.serial_info.speed = msg.speed
        self.serial_info.emergency_stop = msg.emergency_stop
        self.serial_info.brake = msg.brake

        # print(self.serial_info) # 얜 잘 받음 / 근데  general 에서 못받아.ㅇㄹ이러이라ㅓㅁ댜ㅐ렁마러ㅑㅐㄷ머랑ㅁ르

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        cur_yaw = 0
        # print(target_x, target_y)
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        alpha =  cur_yaw - tmp_th                   # 
        # print("alpha before : ",alpha)
        if abs(alpha)>180: # -pi ~ pi
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print("alpha mid : ",alpha)
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print("alpha final : ",alpha)
        # print("x, y", target_x, target_y)
        
        #PUREPURSUIT

        distance = ((target_x - cur_x)**2 + (target_y - cur_y)**2)**(1/2)
        # print('distance:',distance)
        # distance = 5
        # print('distance : ', distance)
        
        delta = degrees(atan2(2*self.WB*sin(radians(alpha))/distance,1))   # 
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360

        # if abs(delta)>30:
        #     if delta > 0:
        #         return 1999
        #     else :
        #         return -1999
        # else :    
        #     delta = delta

        return int(delta)

# --------------------------------시뮬에서는 사용 못함----------------------------------------------
    def calc_velocity_encoder(self):
        self.displacement_right = self.cur_data_right

        if self.dis_DR_flag == 0 :
            before = self.displacement_right
            now = self.displacement_right
            self.dis_DR_flag = 1

        elif self.dis_DR_flag != 0 :
            before = now
            now = self.displacement_right

        if ((now - before) < -10000000):
            self.dis_DR_enc_right = (now + (256**4 - before))*1.6564/100 
        else:
            self.dis_DR_enc_right = (now - before)*1.6564/100

        self.velocity_enc = self.dis_DR_enc_right / 0.1 # 파라미터 rate에 맞게 수정할 것.

    def Get_Dis_right(self, data):
        res = data.data
        # print(res)
        self.cur_data_right = int(res)

print("Control start")
control = Control()
