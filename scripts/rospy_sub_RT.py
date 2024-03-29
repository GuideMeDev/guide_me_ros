#!/usr/bin/env python

# in order to use python3 change the first line.
# from:
# #!/usr/bin/env python
# to:
# #!/usr/bin/env python3

from __future__ import print_function
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Imu, Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import tf.transformations
import numpy as np
import matplotlib.pyplot as plt
import time
import ros_numpy
import scipy.signal as sig
from multiprocessing import Process, Queue 

global imu,pcl,img

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
FPS = 1/6.2

def shutdown():
    cv.destroyAllWindows()

def imu_cb(msg):
    global imu
    imu = msg
    
def get_acc(msg):
    X = msg.linear_acceleration.x
    Y = msg.linear_acceleration.y
    Z = msg.linear_acceleration.z
    buff_acc.append([X,Y,Z])


def pcl_cb(msg):
    global pcl
    pcl = msg
    #main(fig,ax1,ax2)


def img_cb(msg):
    global img
    img = msg


def main(pqueue):
    global imu, pcl, img
    key_pressed = 0
    bridge = CvBridge()
    list_data = []
    # Setting our rate - to 6hz
    rospy.init_node('example_sub_node', anonymous=True)

    #rospy.Subscriber("/imu/data", Imu, imu_cb)
    rospy.Subscriber("/imu/data", Imu, imu_cb)
    rospy.Subscriber("/camera/imu", Imu, get_acc)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, pcl_cb)
    rospy.Subscriber("camera/color/image_raw", Image, img_cb)

    rospy.wait_for_message("/imu/data", Imu)
    rospy.wait_for_message("/camera/imu", Imu)
    rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    rospy.wait_for_message("camera/color/image_raw", Image)

    rospy.on_shutdown(shutdown)
    r = rospy.Rate(6)    

    while not rospy.is_shutdown() and key_pressed < 1:
        st = time.time()
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        euler = (tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[2])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        acc_raw = buff_acc.get()
        #print("size of accel buffer:",len(acc_raw))
        #print("roll: ", roll, "\npitch: ", pitch, "\nyaw: ", yaw)
        # get rgb frame
        img_cnv = bridge.imgmsg_to_cv2(img, 'bgr8')
        # Convert pointcloud to numpy array
        np_pcl = ros_numpy.numpify(pcl)
        # convert raw pcl data into x,y,z array, using the convered np_pcl
        xyz_arr = np.c_[np_pcl['x'],np_pcl['y'],np_pcl['z']]
        # split 'rgb' field data into r,g,b channels in numpy array
        rgb_arr = np_pcl['rgb']
        rgb_arr.dtype = np.uint32
        red = np.asarray((rgb_arr >> 16) & 255, dtype=np.uint8)
        green = np.asarray((rgb_arr >> 8) & 255, dtype=np.uint8)
        blue = np.asarray(rgb_arr & 255, dtype=np.uint8)
        # rgb point cloud
        prgb_arr = np.c_[red,green,blue]
        # insert data to our queue
        pqueue.put([img_cnv,xyz_arr,acc_raw,euler,prgb_arr])
        #xyz = np.asarray(list((pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))))
        #prgb = np.asarray(list((pc2.read_points(pcl, field_names=("r", "g", "b"), skip_nans=True))))
        # print data proccessing time
        print("realtime data proc time: ",time.time() - st)
        # sleeping according to our rate - running every 1/6 seconds
        r.sleep()


if __name__ == '__main__':
    rospy.spin()
