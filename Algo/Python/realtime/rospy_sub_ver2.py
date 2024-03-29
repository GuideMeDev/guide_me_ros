#!/usr/bin/env python

# in order to use python3 change the first line.
# from:
# #!/usr/bin/env python
# to:
# #!/usr/bin/env python3

from __future__ import print_function
import sys
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, CompressedImage, PointCloud2,Image
import sensor_msgs.point_cloud2 as pc2
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
import time
import ros_numpy
from math import sqrt
import scipy.signal as sig
from multiprocessing import Process, Queue
import queue as qu
from Modules.user_feedback import send_feedback
global imu,pcl,img,msg_time

class RingBuffer:
    """ class that implements a not-yet-full buffer """
    def __init__(self,size_max):
        self.max = size_max
        self.data = []

    class __Full:
        """ class that implements a full buffer """
        def append(self, x):
            """ Append an element overwriting the oldest one. """
            self.data[self.cur] = x
            self.cur = (self.cur+1) % self.max
        def get(self):
            """ return list of elements in correct order """
            return self.data[self.cur:]+self.data[:self.cur]

    def append(self,x):
        """append an element at the end of the buffer"""
        self.data.append(x)
        if len(self.data) == self.max:
            self.cur = 0
            # Permanently change self's class from non-full to full
            self.__class__ = self.__Full

    def get(self):
        """ Return a list of elements from the oldest to the newest. """
        return self.data

buffer_size = 200
buff_acc = RingBuffer(buffer_size)
FPS = 6

bb = []
def shutdown():
    cv.destroyAllWindows()

def imu_cb(msg):
    global imu
    imu = msg
    X = msg.linear_acceleration.x
    Y = msg.linear_acceleration.y
    Z = msg.linear_acceleration.z
    buff_acc.append([X,Y,Z])
    # bb.append([X,Y,Z])
    # ts = msg.header.stamp.secs
    # bb.append(ts)
    # if(len(bb) == 200):
    #     print(bb)
    
    
def get_acc(msg):
    pass


def pcl_cb(msg):
    global pcl,msg_time
    msg_time = time.time()
    pcl = msg


def img_cb(msg):
    global img
    img = msg


def RT_writer(pqueue):
    global imu, img, pcl,msg_time
    key_pressed = 0
    bridge = CvBridge()
    list_data = []
    rospy.init_node('example_sub_node', anonymous=True)

    rospy.Subscriber("imu/data", Imu, imu_cb)
    #rospy.Subscriber("/camera/imu", Imu, get_acc)
    rospy.Subscriber("camera/depth/color/points", PointCloud2, pcl_cb)
    #rospy.Subscriber("/camera/color/image_raw/compressed", CompressedImage, img_cb)
    rospy.Subscriber("camera/color/image_raw", Image, img_cb)
    print("stuck")
    rospy.wait_for_message("imu/data", Imu)
    #rospy.wait_for_message("/camera/imu", Imu)
    rospy.wait_for_message("camera/depth/color/points", PointCloud2)
    #rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage)
    rospy.wait_for_message("camera/color/image_raw", Image)
    print("unstuck")
    rospy.on_shutdown(shutdown)
    # Setting our rate - to 6hz
    time.sleep(2)
    #spin_time = time.time()
    r = rospy.Rate(6)    
    # Run while rospy is on and last time we recieved a pointcloud message was less than 5 seconds.
    while not rospy.is_shutdown() and time.time() - msg_time < 5:
        #imu_ts = imu
        #img_ts = img
        #pcl_ts = pcl
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        euler = (tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[2])
        acc_raw = buff_acc.get()
        #print(len(acc_raw))
        #times = [imu_ts.header.stamp.secs,img.header.stamp.secs,pcl.header.stamp.secs,np.array(acc_raw)[:,3]]]
        #print("size of accel buffer:",len(acc_raw))
        # get rgb frame
        img_cv = bridge.imgmsg_to_cv2(img, 'bgr8')
        img_rgb = cv.cvtColor(img_cv, cv.COLOR_BGR2RGB)
        # Convert pointcloud to numpy array
        np_pcl = ros_numpy.point_cloud2.pointcloud2_to_array(pcl)
        # convert raw pcl data into x,y,z array, using the convered np_pcl
        xyz_arr = np.c_[np_pcl['z'],-np_pcl['x'],-np_pcl['y']]
        xyz_arr = xyz_arr[::4]
        x_sum = sum(xyz_arr.T**2)
        f1=np.sqrt(x_sum)<5
        xyz_arr_1=xyz_arr[f1,:]
        x_sum = sum(xyz_arr_1.T**2)
        f2=np.sqrt(x_sum) > 1.5
        xyz_arr_2=xyz_arr_1[f2,:]
        f3=np.abs(xyz_arr_2[:,1])<1.5
        xyz_arr_3=xyz_arr_2[f3,:]
        xyz_arr = xyz_arr_3
        # split 'rgb' field data into r,g,b channels in numpy array
        rgb_arr = np_pcl['rgb']
        rgb_arr.dtype = np.uint32
        red = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
        green = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
        blue = np.asarray(rgb_arr & 255, dtype=np.uint8)
        # rgb point cloud
        prgb_arr = np.c_[red,green,blue]
        prgb_arr =  prgb_arr[::4]
        prgb_arr_1=prgb_arr[f1,:]
        prgb_arr_2=prgb_arr_1[f2,:]
        prgb_arr_3=prgb_arr_2[f3,:]
        
        prgb_arr = prgb_arr_3
        #tstamps.append([imu.header.stamp.secs,img.header.stamp.secs,pcl.header.stamp.secs])
        # insert data to our queue
        pqueue.put([img_rgb,xyz_arr,np.array(acc_raw),euler,prgb_arr])
        #list_data.append([img_rgb,xyz_arr,np.array(acc_raw),euler,prgb_arr])
        # sleeping according to our rate - running every 1/6 seconds
        r.sleep()
    # from scipy.io import savemat
    # savemat("rec5.mat", {"Frames":list_data})

    rospy.signal_shutdown("No more data from queue")
    print("done with rospy")

if __name__ == '__main__':
    RT_writer([])
    rospy.spin()
