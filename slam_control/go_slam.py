
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
#from geometry_msgs.msg import Odometry
from nav_msgs.msg import Odometry
import serial, struct

from math import radians
from math import degrees
from math import sin
from math import cos
from math import hypot
from math import atan2
from math import pi
from math import sqrt

class get_odo:
    def __init__(self):
        # self.map = PointCloud2()
        # rospy.Subscriber("/corrected_cloud", PointCloud2, self.get_flag)
        rospy.Subscriber("/key_pose_origin", PointCloud2, self.getmapMsg)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.getodoMsg)
        rospy.Subscriber("/pose", Odometry, self.getIMU)
        self.pub_map = rospy.Publisher("/visual_path", PointCloud, queue_size=1)
        self.pub_odo = rospy.Publisher("/visual_odo", PointCloud, queue_size=1)
        self.pub_tar = rospy.Publisher("/visual_tar", PointCloud, queue_size=1)
        self.ser = serial.Serial('/dev/ttyUSB0',115200) # USB 권한 주
        self.WB = 1.04
        self.goal_x = 0
        self.goal_y = 0
        self.goal_value = 10
        self.speed= int(150)
        # self.first_flag = False
        self.first_do = False
        self.map = PointCloud()
        self.cur_position = Point32()
        self.tar_position = Point32()

    def getIMU(self, msg):
        self.heading = msg.twist.twist.angular.z - 90
        # print(self.heading)


    def slam_control(self):
        packet = self.ser.readline()
        # print(packet) 
        if len(packet) == 18:
            header = packet[0:3].decode()

            if header == "STX":
                tmp1, tmp2 = struct.unpack("2h", packet[6:10])
                self.V_veh = tmp1   # *10 곱해진 값임.
                self.cnt = struct.unpack("B", packet[15:16])[0]

            ###  seiral에 넣어줄  steering, speed
            self.steering= self.cal_steering(self.cur_position.x, self.cur_position.y, self.heading)
            
            print("conrtorl :",self.speed, self.steering)
            self.serWrite(self.speed,int(self.steering), self.cnt)
         
    def cal_steering(self, cur_x, cur_y, cur_yaw):
        min_index = 0
        min_val = 9999
        for i in range(len(self.map.points)):
            if self.calc_distance(self.map.points[i], self.cur_position) < min_val:
                min_val = self.calc_distance(self.map.points[i], self.cur_position)
                min_index = i
        
        self.goal_x = self.map.points[(min_index+self.goal_value) % (len(self.map.points)-1)].x
        self.goal_y = self.map.points[(min_index+self.goal_value) % (len(self.map.points)-1)].y

        print("cur :", cur_x, cur_y)
        print("goal :", self.goal_x, self.goal_y)

        return self.steering_angle(cur_x, cur_y, cur_yaw, self.goal_x, self.goal_y)

    def steering_angle(self,  cur_x, cur_y, cur_yaw, target_x, target_y):
        tmp_th = degrees(atan2((target_y - cur_y), (target_x - cur_x)))
        tmp_th = tmp_th%360
        print("angular :", cur_yaw , tmp_th)
        alpha =  cur_yaw - tmp_th
        print("alpha:", alpha)
        if abs(alpha)>180: # -pi ~ pi
            if (alpha < 0) :
                alpha += 360
            else :
                alpha -= 360
        alpha = max(alpha,  -90)
        alpha = min(alpha,  90)
        # print("x, y", target_x, target_y)
        
        distance = ((target_x - cur_x)**2 + (target_y - cur_y)**2)**(1/2)
        delta = degrees(atan2(2*self.WB*sin(radians(alpha))/distance,1))   # 
        
        if abs(delta)>180:
            if (delta < 0) :
                delta += 360
            else :
                delta -= 360
        if abs(delta)>30:
            if delta > 0:
                return 1999
            else :
                return -1999
        else :    
            delta = 71*delta
        return int(delta)

    def serWrite(self,speed, steering, cnt):
        break_val = 0x01
        result = struct.pack('!BBBBBBHhBBBB', 0x53, 0x54, 0x58, 0x01, 0x00, 0x00, speed,
                    steering, break_val, self.cnt, 0x0D, 0x0A )    # big endian 방식으로 타입에 맞춰서 pack   
        self.ser.write(result)

    def calc_distance(self, start, end):
        return ((start.x - end.x) ** 2 + (start.y - end.y) ** 2) ** 0.5

    # def get_flag(self, msg):
    #     # print("do?")
    #     if self.first_do is False:
    #         self.first_flag = True
    #         self.first_do = True

    def getmapMsg(self, msg):
        # if self.first_flag is True:
            # self.map = msg
        temp = point_cloud2.read_points(msg, skip_nans=True)
        path_list = PointCloud()
        
        path_list.points = []
        for p in temp:
            get_point = Point32()
            get_point.x = p[2]
            get_point.y = p[0]
            get_point.z = p[1]
            path_list.points.append(get_point)
        print(len(path_list.points))
        print(path_list.points[-1])
        # print(self.cur_position)
        if len(path_list.points) > 10:
            for i in range(0, len(path_list.points)-10):
                # print(i)
                if self.calc_distance(path_list.points[-1], path_list.points[i]) <= 1 and self.first_do is False:
                    self.first_do = True
                    self.map.header.frame_id = 'map'
                    self.map.header.stamp = rospy.Time.now()
                    self.map.points = path_list.points[i:]
                    continue
        self.pub_map.publish(self.map)
        # if self.calc_distance(path_list.points[-1], path_list.points[0]) <= 0.5 and self.first_do is False:            
        # self.first_flag = False
        # else:
        #     return

    def getodoMsg(self, msg):
        self.cur_position.x = msg.pose.pose.position.z
        self.cur_position.y = msg.pose.pose.position.x
        cur_pos = PointCloud()
        cur_pos.header.frame_id = 'map'
        cur_pos.header.stamp = rospy.Time.now()
        cur_pos.points.append(self.cur_position)
        self.pub_odo.publish(cur_pos)

        self.tar_position.x = self.goal_x
        self.tar_position.y = self.goal_y
        tar_pos = PointCloud()
        tar_pos.header.frame_id = 'map'
        tar_pos.header.stamp = rospy.Time.now()
        tar_pos.points.append(self.tar_position)
        self.pub_tar.publish(tar_pos)
        if self.first_do:
            self.slam_control()



print("ON")
rospy.init_node("LiDAR_loam", anonymous=False)
get_function = get_odo()

rate = rospy.Rate(10)

while not rospy.is_shutdown():
    # get_function.slam_control()
    # rospy.spin()
    rate.sleep()