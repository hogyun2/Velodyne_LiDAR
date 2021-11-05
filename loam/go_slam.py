#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Odometry


class get_odo:
    def __init__(self):
        # self.map = PointCloud2()
        rospy.Subscriber("/corrected_cloud", PointCloud2, self.get_flag)
        rospy.Subscriber("/key_pose_origin", PointCloud2, self.getmapMsg)
        rospy.Subscriber("/aft_mapped_to_init", Odometry, self.getodoMsg)
        self.pub_map = rospy.Publisher("/visual_path", PointCloud, queue_size=1)
        self.pub_odo = rospy.Publisher("/visual_odo", Point32, queue_size=1)
        self.first_flag = False
        self.first_do = False
        self.cur_position = Point32()

    def get_flag(self, msg):
        if self.first_do is False:
            self.first_flag = True
            self.first_do = True

    def getmapMsg(self, msg):
        if self.first_flag is True:
            # self.map = msg
            temp = point_cloud2.read_points(msg, skip_nans=True)
            path_list = PointCloud()
            path_list.points = []
            for p in temp:
                get_point = Point32()
                get_point.x = p[0]
                get_point.y = p[1]
                path_list.points.append(get_point)
            path_list.header.frame_id = 'world'
            path_list.header.stamp = rospy.Time.now()
            self.pub_map.publish(path_list)
            self.first_flag = False
        else:
            return

    def getodoMsg(self, msg):
        self.cur_position.x = msg.pose.pose.position.x
        self.cur_position.y = msg.pose.pose.position.y

    def run(self):
        self.pub_odo.publish(self.cur_position)


print("ON")
rospy.init_node("LiDAR_loam", anonymous=False)

while not rospy.is_shutdown():
    get_odo.run()
    rospy.spin()
