#!/usr/bin/env python
from __future__ import print_function, absolute_import, division
import numpy as np

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from collections import deque


class Node:
    def __init__(self):
        rospy.init_node('check_dt', anonymous=True)
        self.sub_imu = rospy.Subscriber("~imu", Imu, self.imu_cb)
        self.pub_dt = rospy.Publisher("~dt", Float64, queue_size=1)
        self.imu_msg = None

    def imu_cb(self, imu_msg):
        if self.imu_msg is None:
            self.imu_msg = imu_msg
            return

        dt_msg = Float64()
        dt_msg.data = (imu_msg.header.stamp -
                       self.imu_msg.header.stamp).to_sec()
        self.pub_dt.publish(dt_msg)
        print("dt: ", dt_msg.data)

        self.imu_msg = imu_msg


if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
