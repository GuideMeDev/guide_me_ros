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
global imu, pcl, img


def shutdown():
    cv.destroyAllWindows()


def imu_cb(msg):
    global imu
    imu = msg


def pcl_cb(msg):
    global pcl
    pcl = msg


def img_cb(msg):
    global img
    img = msg


def main():
    global imu, pcl, img
    key_pressed = 0
    st_time2 = 0
    st_time1 = 0
    bridge = CvBridge()
    list_data = []
    r = rospy.Rate(6)
    run_minute = True
    one_minute = time.time()
    while not rospy.is_shutdown() and key_pressed < 1:
        st_time1 = time.time()
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        euler = (tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[2])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        #print("\nroll: ", roll, "\npitch: ", pitch, "\nyaw: ", yaw, "\n")
        img_cnv = bridge.imgmsg_to_cv2(img, 'bgr8')
        #cv.imshow("hhh", bridge.imgmsg_to_cv2(img, 'bgr8'))
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
        prgb_arr = np.c_[red,green,blue]
        #xyz = np.asarray(list((pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))))
        #prgb = np.asarray(list((pc2.read_points(pcl, field_names=("r", "g", "b"), skip_nans=True))))

        # if xyz.shape:
        #     plt.scatter(xyz[:,1],xyz[:,2])
        #     plt.pause(0.05)
        #ey_pressed = cv.waitKey(27)
        list_data.append([img_cnv,xyz_arr,prgb_arr])
        end_stamp = time.time()
        end_time = end_stamp - st_time1
        print("time for data processing",end_time)
        r.sleep()
        print("time left for computation in frametime:",time.time()-end_stamp)
        if time.time() - one_minute >=60:
            break
    print("num of frames: ", len(list_data))
    # When finished running, store the data
    cv.destroyAllWindows()
    rosDict = dict({"rgb":img,"pcl":xyz_arr,"prgb":prgb_arr})
    #exit(0)



if __name__ == '__main__':
    rospy.init_node('example_sub_node', anonymous=True)

    rospy.Subscriber("/imu/data", Imu, imu_cb)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, pcl_cb)
    rospy.Subscriber("camera/color/image_raw", Image, img_cb)

    rospy.wait_for_message("/imu/data", Imu)
    rospy.wait_for_message("/camera/depth/color/points", PointCloud2)
    rospy.wait_for_message("camera/color/image_raw", Image)

    rospy.on_shutdown(shutdown)

    main()

    rospy.spin()