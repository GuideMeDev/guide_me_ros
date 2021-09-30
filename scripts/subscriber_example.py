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

    bridge = CvBridge()

    r = rospy.Rate(6)
    while not rospy.is_shutdown():
        quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
        euler = (tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[2])
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        print("\nroll: ", roll, "\npitch: ", pitch, "\nyaw: ", yaw, "\n")

        cv.imshow("", bridge.imgmsg_to_cv2(img, 'bgr8'))
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

        xyz = list(pc2.read_points(pcl, field_names=("x", "y", "z"), skip_nans=True))
        rgb = list(pc2.read_points(pcl, field_names=("r", "g", "b"), skip_nans=True))

        r.sleep()


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
