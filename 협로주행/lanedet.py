#-*- coding:utf-8 -*-

'''
self.control_data 는 실시간으로 master.py, sender.py와 공유됨 
'''

# from master_node.msg import lane
# from std_msgs.msg import Time
# rospy.Subscriber("/laneinformation", lane, self.getline)          
# def calc_velocity(self, k):
#     V_ref_max = 15 # km/h
#     n = 0.8 #safety factor

#     critical_k = ((n/V_ref_max)**2) * 19.071

#     if k < critical_k:
#         V_ref = V_ref_max
#     else:
#         V_ref = n * (sqrt(19.071/k))

#     return 10 * V_ref # 10*(km/h)



import pymap3d
#import csv
import rospy
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry
from std_msgs.msg import String
# from obstacle_detector.msg import Obs
import numpy as np
from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
from math import pi
# import LPP
import sys
import serial
import struct
import matplotlib.pyplot as plt
# from hybrid_a_star import path_plan 
# from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import Pose, Point, Vector3, Quaternion
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from std_msgs.msg import Time
from master_node.msg import lane
from master_node.msg import BoundingBoxes
from master_node.msg import BoundingBox
from visualization_msgs.msg import Marker
# sys.path.append("./map1/")

# try:
#     import makeTarget
# except:
#     raise

class Controller:
    def __init__(self,  master):
        self.control_data = master.control_data
        self.path_x = []
        self.path_y = []
        self.path_yaw = []
        self.lpath_x = []
        self.lpath_y = []
        self.goal_x = 0
        self.goal_y = 0
        self.final_switch = 0
        self.cur_state = 'vision_lane'
        self.vision_lane_data = lane()
        self.lidar_lane_data = lane()
        self.corn_flag = True
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주

        self.drive_mode = 'normal'

        self.Kp_ld = 0.03
        self.left_line_pub = rospy.Publisher('left_line', Marker, queue_size = 10)
        self.right_line_pub = rospy.Publisher('right_line', Marker, queue_size = 10)
        self.Kp_v = 50
        self.Ki_v = 5
        self.Kd_v = 10

        self.Kp_s = 0.5
        self.Ki_s = 0.01
        self.Kd_s = 0
        
        self.lane_curv = 0

        self.curvature = 0
        self.speed_look_ahead = 6
        self.V_err = 0
        self.V_err_old = 0
        self.V_in = 0
        self.V_veh = 0
        self.V_err_pro = 0
        self.V_err_inte = 0
        self.V_err_deri = 0

        self.steering_veh = 0
        self.steering_err =0
        self.steering_err_old = 0
        self.steering_in = 0
        self.steering_err_inte = 0
        self.steering_err_deri = 0

    # def connect(self):
    #         self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #UDP LAN
    #         recv_address = ('127.0.0.1', 3052)
    #         self.sock.bind(recv_address)

    #         print("Connect!")

    def line_marker(self, fir, last, name, idd):
        scale = Vector3(0.04, 0.04, 0.04)
        line_mk = Marker()
        line_mk.header.frame_id = "world"
        line_mk.header.stamp = rospy.Time.now()
        line_mk.ns = name
        line_mk.id = idd
        line_mk.type = Marker.ARROW
        line_mk.action = Marker.ADD
        line_mk.pose.position.x = 0.0
        line_mk.pose.position.y = 0.0
        line_mk.pose.position.z = -0.1
        line_mk.pose.orientation.x = 0.0
        line_mk.pose.orientation.y = 0.0
        line_mk.pose.orientation.z = 0.0
        line_mk.pose.orientation.w = 0.0
        line_mk.scale = scale
        line_mk.color.r = 1.0
        line_mk.color.g = 0.0
        line_mk.color.b = 0.0
        line_mk.color.a = 1.0
        line_mk.points.append(fir)
        line_mk.points.append(last)
        return line_mk

    def getcurstate(self, msg):
        temp_msg = msg.data
        if temp_msg == 'vision_start':
            self.cur_state = 'vision_lane'
        ## 지금 lidar 정보에서 curvature로 가장 최소 거리 들어오고 있는거 그거 추가해서 상황 판단 가능합니ㅏㄷ~~~
        elif temp_msg == 'lidar_start':
            self.cur_state = 'lidar_lane'
        elif temp_msg =='':
            print("pass")
        else:
            print("get state message Error!!")

    def getlidarlineline(self, msg):
        # self.lidar_lane_data = lane()
        # print("get lidar lane~~~")
        self.lidar_lane_data = msg
        ##라이다에서 들어오는 정보 없으면 차선으로 돌리는 부분인데 확인 함 필요해 보임더~~~
        

    def getvisionline(self, msg):
        self.vision_lane_data = msg

    def getboundingbox(self, msg):
        temp = msg.bounding_boxes
        # print(len(temp))
        for i in range(len(temp)):
            if temp[i].Class == "CONE":
                self.cur_state = 'lidar_lane'
        # print(temp[0].probability)
        # print("??", msg.bounding_boxes[0][0])

    def get_xy(self,  lat,  lon,  alt):
        e, n, u = pymap3d.geodetic2enu(lat, lon, alt, self.control_data['base_lat'],  self.control_data['base_lon'],  self.control_data['base_alt'])
        return e,  n
             
    def cal_steering(self, cur_x, cur_y, cur_yaw):
        # print(self.cur_state)
        if self.cur_state == 'vision_lane' :
            self.getline(self.vision_lane_data)
        elif self.cur_state == 'lidar_lane':
            print("left :", len(self.lidar_lane_data.left))
            print("right :", len(self.lidar_lane_data.right))
            self.getline(self.lidar_lane_data)
        else:
            print("Error!! State Empty")
        return self.steering_angle(cur_x, cur_y, cur_yaw, self.goal_x, self.goal_y)

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        cur_yaw = 0
        print(target_x, target_y)
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        alpha =  cur_yaw - tmp_th
        # print("alpha before : ",alpha)
        if abs(alpha)>180:
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        # print("alpha mid : ",alpha)
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        print("alpha final : ",alpha)
        delta = alpha
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        print("delta", delta)
        if abs(delta)>30:
            if delta > 0:
                return 1999
            else :
                return -1999
        else :
            delta = 71*delta
        return int(delta)

    def line_detect(self, point):
        x_sum = 0
        y_sum = 0
        number = len(point)
        for i in range(number):
            x_sum += point[i].x
            y_sum += point[i].y
        number = float(number)
        x_mean = x_sum / number
        y_mean = y_sum / number

        up = 0
        down = 0

        for i in range(int(number)):
            up += (point[i].x-x_mean) * (point[i].y - y_mean)
            down += (point[i].x-x_mean) ** 2

        a = up / down
        b = y_mean - a * x_mean

        return a, b

    def getline(self, lane_data):
        left_first = Point32()
        left_last = Point32()
        right_first = Point32()
        right_last = Point32()
        center_first = Point32()
        center_last = Point32()

        # left is not empty
        left_x=[]
        left_y=[]
        right_x=[]
        right_y=[]

        for i in range(len(lane_data.left)):
            left_x.append(lane_data.left[i].x)
            left_y.append(lane_data.left[i].y)

        for i in range(len(lane_data.right)):
            right_x.append(lane_data.right[i].x)
            right_y.append(lane_data.right[i].y)

        if len(lane_data.left) != 0:
            l_a, l_b = self.line_detect(lane_data.left)
            left_first.x = min(left_x)
            left_first.y = min(left_x) * l_a + l_b
            left_last.x = max(left_x)
            left_last.y = max(left_x) * l_a + l_b
        else:
            l_a, l_b = 0, 0
            left_first.x = 0
            left_first.y = 0
            left_last.x = 0
            left_last.y = 0

        # right is not empty
        if len(lane_data.right) != 0:
            r_a, r_b = self.line_detect(lane_data.right)
            right_first.x = min(right_x)
            right_first.y = min(right_x) * r_a + r_b
            right_last.x = max(right_x)
            right_last.y = max(right_x) * r_a + r_b
        else:
            r_a, r_b = 0, 0
            right_first.x = 0
            right_first.y = 0
            right_last.x = 0
            right_last.y = 0


        c_a = (l_a + r_a) / 2
        c_b = 0.0
        

        center_first.x = 0.0
        center_first.y = 0.0
        center_last.x = 1.0
        center_last.y = 1.0 * c_a + c_b
        d=1.5
        # print("number :", len(lane_data.left), len(lane_data.right))
        l_rad=np.arctan2(left_last.y - left_first.y, left_last.x - left_first.x) - pi / 2
        r_rad=np.arctan2(right_last.y - right_first.y, right_last.x - right_first.x) +pi / 2

        if len(lane_data.left)!= 0 and len(lane_data.right) != 0:
            self.goal_x, self.goal_y = (left_last.x + right_last.x)/2 , (left_last.y + right_last.y)/2

        elif len(lane_data.left) == 0 and len(lane_data.right) == 0:
            self.goal_x, self.goal_y = 1, 0
            
        elif len(lane_data.left) != 0 and len(lane_data.right) == 0:
            # print("left_deg : ",np.rad2deg(rad2))
            self.goal_x = left_last.x + (d*cos(l_rad))
            self.goal_y = left_last.y + (d*sin(l_rad))
            # self.goal_x, self.goal_y = 1, -1

        elif len(lane_data.left) == 0 and len(lane_data.right) != 0:
            # print("right_deg : ",np.rad2deg(r_rad))
            self.goal_x = right_last.x + (d*cos(r_rad))
            self.goal_y = right_last.y + (d*sin(r_rad))
            # self.goal_x, self.goal_y = 1, 1

        l_line = self.line_marker(left_first, left_last, "left", 0)
        r_line = self.line_marker(right_first, right_last, "right", 1)
        # print(self.goal_x,self.goal_y)


        self.left_line_pub.publish(l_line)
        self.right_line_pub.publish(r_line)

    def serWrite(self, steering, cnt):
        break_val = 0x01

        # if self.drive_mode == 'stop':
        #     V_in = 0x00
        #     break_val = 50
        print(steering)    
        #스피드 바꾸어 주는 부분이 아래 int(10) 되어있는 부분임다 빠르게 진행하시려면 숫자 키워주시면 됩니다링
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, int(60),
                    steering, break_val, cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        self.ser.write(result)

    def getOdoMsg(self,  msg):
        # print(self.cur_state)
        if len(self.lidar_lane_data.left) == 0 and len(self.lidar_lane_data.right) == 0:
            self.cur_state = 'vision_lane'
        print(self.cur_state)
        self.control_data['cur_yaw']  = 0
        self.control_data['cur_x'], self.control_data['cur_y']  = 0, 0

        cnt=0x00

        result = self.ser.readline()
        
        self.ser.flushInput()
        # print(result)
        if (result[0] is 0x53 and result[1] is 0x54 and result[2] is 0x58):
            # print("do?")
            res_arr = []
            res_idx = 0

            while True:
                for i in range(len(result)):
                    if result[i] is 0x0A and i is not 17:
                        res_arr.append(0x0B)
                    else :
                        res_arr.append(result[i])

                if len(res_arr) < 18:
                    result = self.ser.readline()
                    # print(result)
                else:
                    break

            # cnt = int(ord(res_arr[15]))
            cnt = res_arr[15]


            # if len(self.vision_lane_data) != 0 or len(self.lidar_lane_data)!=0:
            self.control_data['steering']= self.cal_steering(self.control_data['cur_x'], self.control_data['cur_y'],  self.control_data['cur_yaw'])
            print("check", self.control_data['steering'])
            self.serWrite(int(self.control_data['steering']), cnt)

    def run(self):
        # rospy.init_node("vision_lidar", anonymous=True)
        # self.connect()
        rospy.Subscriber("/timer", Time, self.getOdoMsg)
        rospy.Subscriber("/lidar_pub", lane, self.getlidarlineline)
        rospy.Subscriber("/laneinformation", lane, self.getvisionline)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.getboundingbox)
        

