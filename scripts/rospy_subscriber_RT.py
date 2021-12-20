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
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import time
import ros_numpy
import scipy.signal as sig
from ros_test import *
import threading
#global imu, pcl, img

global imu,pcl,img

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


def main():
    global imu, pcl, img
    key_pressed = 0
    bridge = CvBridge()
    list_data = []
    # Setting our rate - to 6hz
    r = rospy.Rate(6)
    one_minute = time.time()
    fig, (ax1, ax2) = plt.subplots(1, 2,figsize=(7,7))
    dummy_img = np.zeros((720,1280,3))
    ax1_data = ax1.imshow(dummy_img)
    ax2_data = ax2.plot([],[],'.')[0]
    fig.show()
    # p = Process(target=get_acc)
    # p.start()
    while not rospy.is_shutdown() and key_pressed < 1:
        st = time.time()
    #st_time1 = time.time()
    #while True:
        print("size of accel buffer:",len(buff_acc.get()))
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        euler = (tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[2])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print("\nroll: ", roll, "\npitch: ", pitch, "\nyaw: ", yaw)
        # get rgb frame
        img_cnv = bridge.imgmsg_to_cv2(img, 'bgr8')
        ax1_data.set_data(img_cnv)
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
        #
        #xyz = np.asarray(list((pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))))
        #prgb = np.asarray(list((pc2.read_points(pcl, field_names=("r", "g", "b"), skip_nans=True))))

        # if xyz.shape:
        ax2_data.set_data(xyz_arr[:,0],xyz_arr[:,1])
        # ax2.plot(xyz_arr[:,0],xyz_arr[:,1],'.')
        # plt.pause(0.05)
        # print data proccessing time
        print(time.time() - st)
        print("imu acc: ",buff_acc.get()[-1])
        fig.canvas.draw()
        fig.canvas.flush_events()
        # sleeping according to our rate - running every 1/6 seconds
        r.sleep()
    #time.sleep(FPS)
    #print("num of frames: ", len(list_data))
    # When finished running, store the data
    #rosDict = dict({"rgb":img,"pcl":xyz_arr,"prgb":prgb_arr})
    #exit(0)


if __name__ == '__main__':

    rospy.init_node('example_sub_node', anonymous=True)
    
    #rospy.Subscriber("/imu/data", Imu, imu_cb)
    rospy.Subscriber("/imu/data", Imu, imu_cb)
    rospy.Subscriber("/camera/imu", Imu, get_acc)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, pcl_cb)
    rospy.Subscriber("camera/color/image_raw", Image, img_cb)

    rospy.wait_for_message("/imu/data", Imu)
    rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    rospy.wait_for_message("camera/color/image_raw", Image)

    rospy.on_shutdown(shutdown)
    # giving time for IMU to retrive 200 samples (200hz)
    time.sleep(1)
    main()
    #time.sleep(1)
    #rospy.Timer(rospy.Duration(1.0 / FPS), main())

    #timer_algo = threading.Timer(FPS,main)
    #timer_algo.start()
    #main()
    rospy.spin()
