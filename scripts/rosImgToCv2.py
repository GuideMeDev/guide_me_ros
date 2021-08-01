#!/usr/bin/env python

import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

global out, out_rgb, c, c_rgb, frame_num
frame_num = 300
c = 0
c_rgb = 0

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'XVID')


def out_fun(shape):
    global out
    out = cv2.VideoWriter('depth/outdoors_depth.avi', fourcc, 5.0, (shape[1], shape[0]))


def out_fun_rgb(shape):
    global out_rgb
    out_rgb = cv2.VideoWriter('rgb/outdoors_rgb.avi', fourcc, 5.0, (shape[1], shape[0]))


def _shutdown():
    global out, c, out_rgb, c_rgb
    out.release()
    out_rgb.release()
    print(c, c_rgb, " frames has been captured")
    print("shutdown")


def normalize_depth_image(depth_image, max_range):
    depth_image = (depth_image/max_range)*255
    return np.round(depth_image).astype(np.uint8)


def _Img_callback(ros_data, bridge):
    global out, c

    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    # encoding of simulated depth image: 32FC1, encoding of real depth image: 16UC1
    frame = bridge.imgmsg_to_cv2(ros_data, desired_encoding="8UC1")
    # frame = np.array(cv_image, dtype=np.uint8)
    # frame = normalize_depth_image(cv_image, 6.0)

    if (c == 0):
        frame_shape = np.shape(frame)
        out_fun(frame_shape)

    # frame = cv2.flip(frame, 0)
    cv2.imwrite("depth/outdoors_depth_"+str(c)+'.jpg', frame)
    out.write(frame)
    rospy.sleep(0.1)
    c = 1+c

    print("The number amount of captured frames: ", c)
    if (c == frame_num):
        print("Video end ")
        rospy.signal_shutdown("Video end")


def _Img_callback_rgb(ros_data):
    global out_rgb, c_rgb

    # np_arr = np.fromstring(ros_data.data, np.uint8)
    # frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame = bridge.imgmsg_to_cv2(ros_data)

    if (c_rgb == 0):
        frame_shape = np.shape(frame)
        out_fun_rgb(frame_shape)

    # frame = cv2.flip(frame, 0)
    cv2.imwrite("rgb/outdoors_rgb_"+str(c_rgb)+'.jpg', frame)
    out_rgb.write(frame)
    rospy.sleep(0.1)
    c_rgb = 1+c_rgb

    print("The number amount of captured frames: ", c_rgb)
    if (c_rgb == frame_num):
        print("Video end ")
        rospy.signal_shutdown("Video end")


rospy.init_node('video_handler', anonymous=True)
rospy.on_shutdown(_shutdown)

bridge = CvBridge()

rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",
                 Image, lambda img: _Img_callback(img, bridge), queue_size=2)
rospy.Subscriber("/camera/color/image_raw",
                 Image, _Img_callback_rgb, queue_size=2)

rospy.wait_for_message("/camera/aligned_depth_to_color/image_raw", Image)
rospy.wait_for_message("/camera/color/image_raw", Image)

rospy.spin()
