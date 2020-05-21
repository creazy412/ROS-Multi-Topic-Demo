#!/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
红外墙检
"""
import rospy
from sensor_msgs.msg import Range
from ak_ros_pkg.msg import Wall_detection_msg

class Processer:
    def __init__(self):
        # 实例化订阅多个 Topic
        self.sub1 = rospy.Subscriber("/sensor/wall_detection_left", Range, self.callback1)
        self.sub2 = rospy.Subscriber("/sensor/wall_detection_left_front", Range, self.callback2)
        self.sub3 = rospy.Subscriber("/sensor/wall_detection_right", Range, self.callback3)
        self.sub4 = rospy.Subscriber("/sensor/wall_detection_right_front", Range, self.callback4)

        self.pub1 = rospy.Publisher('demo_wall_detection', Wall_detection_msg, queue_size=10)
        # self.pub2 = rospy.Publisher('demo_wall_tof', Wall_tof_all_msg, queue_size=10)

        self.wall_detection_distance = {}

    def callback1(self, data):
        """
        左墙检
        """
        distance = int(data.range * 1000)

        self.wall_detection_distance['l_wd'] = distance
        self.check_wall_detection_distance()
    
    def callback2(self, data):
        """
        左前墙检
        """
        distance = int(data.range * 1000)

        self.wall_detection_distance['lf_wd'] = distance
        self.check_wall_detection_distance()

    def callback3(self, data):
        """
        右墙检
        """
        distance = int(data.range * 1000)

        self.wall_detection_distance['r_wd'] = distance
        self.check_wall_detection_distance()

    def callback4(self, data):
        """
        右前墙检
        """
        distance = int(data.range * 1000)

        self.wall_detection_distance['rf_wd'] = distance
        self.check_wall_detection_distance()

    def check_wall_detection_distance(self):
        """
        检查 wall_detection_distance 字典是否满足两个值的条件
        """
        if len(self.wall_detection_distance) == 4:
            wall_detection_single = Wall_detection_msg()
            # 单位 m
            wall_detection_single.l_wd = self.wall_detection_distance['l_wd']
            wall_detection_single.lf_wd = self.wall_detection_distance['lf_wd']
            wall_detection_single.r_wd = self.wall_detection_distance['r_wd']
            wall_detection_single.rf_wd = self.wall_detection_distance['rf_wd']
            self.pub1.publish(wall_detection_single)

if __name__ == '__main__':
    rospy.init_node("wall_detection")
    
    p = Processer()

    rospy.spin()
