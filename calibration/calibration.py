import rospy
from sensor_msgs.msg import PointCloud
from master_node.msg import Obstacles
from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from shapely.geometry import Point, Polygon
import numpy as np

class calibration:
    def __init__(self):
        rospy.init_node("calibration", anonymous=False)
        rospy.Subscriber("lidar_pub", PointCloud, self.get_lidar)
        rospy.Subscriber("obstacles", Obstacles, self.get_obstacles)
        rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.get_bbox)
        self.lidar_msg = PointCloud()
        self.obstacle_msg = Obstacles()
        self.new_obstacle = Obstacles()
        self.bbox_msg = BoundingBox()
        self.flag = False
        self.camera_matrix = np.array([[1166.853156, 0.000000, 958.517180], [0.000000, 1172.932471, 556.692563], [0.000000, 0.000000, 1.000000]])
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.circle_z()
            rate.sleep()

    def get_lidar(self, msg):
        self.lidar_msg = msg
    
    def get_obstacles(self, msg):
        self.obstacle_msg = msg

    def get_bbox(self, msg):
        self.bbox_msg = msg.bounding_boxes
        self.flag = True

    def circle_z(self):
        obstacle_space_poly = []
        obs_z = []

        for i in range(len(self.obstacle_msg.circles)):
            x1, x2 = self.obstacle_msg.circles[i].center.x - self.obstacle_msg.circles[i].radius, self.obstacle_msg.circles[i].center.x + self.obstacle_msg.circles[i].radius
            y1, y2 = self.obstacle_msg.circles[i].center.y - self.obstacle_msg.circles[i].radius, self.obstacle_msg.circles[i].center.y + self.obstacle_msg.circles[i].radius

            obstacle_space = [[x1, y1], [x1, y2], [x2, y2], [x2, y1]]
            obstalce_temp = Polygon(obstacle_space)
            obstacle_space_poly.append(obstalce_temp)

        for i in range(len(self.lidar_msg.points)):
            obs_z.append([0,0])

        for i in range(len(self.lidar_msg.points)):
            test_point = Point(self.lidar_msg.points[i].x, self.lidar_msg.points[i].y)
            for p in range(len(obstacle_space_poly)):
                if test_point.within(obstacle_space_poly[p]):
                    obs_z[p][0] += self.lidar_msg.points[i].z
                    obs_z[p][1] += 1

        for i in range(len(obs_z)):
            if obs_z[i][1] == 0:
                obs_z[i][1] = 1

        for i in range(len(self.obstacle_msg.circles)):
            self.obstacle_msg.circles[i].center.z = obs_z[i][0] / obs_z[i][1]
        
        self.calibration()
    
    def calibration(self):
        atf_cali = []
        bbox_space_poly = []
        
        # print(self.bbox_msg[1])
        
        if self.flag:
            # print(self.bbox_msg)
            # print(self.bbox_msg[0].xmin)
            for i in range(len(self.bbox_msg)):
                bbox_space = [[self.bbox_msg[i].xmin, self.bbox_msg[i].ymin], [self.bbox_msg[i].xmin, self.bbox_msg[i].ymax], [self.bbox_msg[i].xmax, self.bbox_msg[i].ymax], [self.bbox_msg[i].xmax, self.bbox_msg[i].ymin]]
                bbox_temp = Polygon(bbox_space)
                bbox_space_poly.append(bbox_temp)

            for i in range(len(self.obstacle_msg.circles)):
                c_x = -self.obstacle_msg.circles[i].center.y
                c_y = 1.13 - self.obstacle_msg.circles[i].center.z
                c_z = 0.6 + self.obstacle_msg.circles[i].center.x
                obs_matrix = np.array([[c_x], [c_y], [c_z]])

                pixel_matrix = np.dot(self.camera_matrix, obs_matrix)

                # print(pixel_matrix.shape)
                print("obs :", i)
                print(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2], pixel_matrix[2]/pixel_matrix[2])
                cali_point = Point(pixel_matrix[0]/pixel_matrix[2], pixel_matrix[1]/pixel_matrix[2])

                for p in range(len(bbox_space_poly)):
                    if cali_point.within(bbox_space_poly[p]):
                        print(self.bbox_msg[p].id)
                        print(self.bbox_msg[p].Class)
        

        

print("ON") 
Calibration = calibration()       
