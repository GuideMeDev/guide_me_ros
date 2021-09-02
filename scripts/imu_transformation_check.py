#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import tf.transformations

global toc


def cb_imu(msg):
    global toc
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    euler = (tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[2])
    tic = rospy.Time.now().to_sec()
    if (tic - toc) > 2.:
        print("euler: ", euler)
        toc = rospy.Time.now().to_sec()


rospy.init_node("imu_check", anonymous=True)
toc = rospy.Time.now().to_sec()
rospy.Subscriber("/imu/data", Imu, cb_imu)
rospy.spin()
