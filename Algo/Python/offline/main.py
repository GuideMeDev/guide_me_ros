import numpy as np
import rospy
import rosbag
import os
import tf
from cv_bridge import CvBridge
import cv2
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from Modules.plane_fit import plane_fit
from Modules.utils import *
from Modules.translation_filter import translation_filter

# Loading from a rosbag recording
# topic - the source of the message, msg - the message/data of the source, t- timestamp
bag_dir = r"/home/nuc_guideme/Algo_Workspace/Recordings/trimmed_obj.bag"
csv_dir = r"/home/nuc_guideme/Algo_Workspace/Recordings/trimmed_obj/"
bridge = CvBridge()
imu_euler = []
imu_acc = []
rgb_data = []
xyz_data = []
imu_ts = []
xyz_ts = []
rgb_ts = []

bag = rosbag.Bag(bag_dir)

for topic, msg, t in bag.read_messages(topics=['/imu/data']):
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    # quat_R = tf.transformations.quaternion_matrix(quat)
    # quat_X = quat_R[0:3,0]
    # quat_Y = quat_R[0:3,1]
    # quat_Z = quat_R[0:3,2]
    euler = (tf.transformations.euler_from_quaternion(quat)[1], tf.transformations.euler_from_quaternion(quat)[0], tf.transformations.euler_from_quaternion(quat)[2])
    #R= np.array([quat_X.T,quat_Y.T,quat_Z]).T
    # pitch = np.arctan2(quat_Y[2],quat_Z[2])
    # roll = -np.arcsin(quat_X[2])
    # yaw = np.arctan2(quat_X[1],quat_X[0])
    # print(euler[0]-roll,euler[1]-pitch,euler[2]-yaw)
    X = msg.linear_acceleration.x
    Y = msg.linear_acceleration.y
    Z = msg.linear_acceleration.z
    imu_euler.append(euler)
    imu_acc.append([X,Y,Z])
    imu_ts.append(rospy.Time.to_time(msg.header.stamp))

for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw/compressed']):
    img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    rgb_data.append(img)
    rgb_ts.append(rospy.Time.to_time(msg.header.stamp))

for topic, msg, t in bag.read_messages(topics=['/camera/depth/color/points']):
    # pcl = pc2.read_points_list(msg,skip_nans=True)
    # np_pcl = np.array(pcl,dtype=np.float32)
    np_pcl = ros_numpy.point_cloud2.pointcloud2_to_array(msg)
    # convert raw pcl data into x,y,z array, using the convered np_pcl
    #Xdr=[Xd(range,3)';-Xd(range,1)';-Xd(range,2)']';Xdr(sqrt(sum(Xdr'.^2))>6,:)=0;
    xyz_arr = np.c_[np_pcl['z'],-np_pcl['x'],-np_pcl['y']]
    x_sum = sum(xyz_arr.T**2)

    xyz_arr[np.sqrt(x_sum)>6,:]=0
    xyz_arr[np.sqrt(x_sum)<1,:]=0
    xyz_data.append(xyz_arr)
    xyz_ts.append(rospy.Time.to_time(msg.header.stamp))

rgb_ts = np.array(rgb_ts)
imu_ts = np.array(imu_ts)
rgb_data = np.array(rgb_data)
imu_acc = np.array(imu_acc)
imu_euler = np.array(imu_euler)

# get matching timestamps
res_t_rgb = []
res_t_imu = []
for ts in xyz_ts:
    rgb_t = abs(ts - rgb_ts)
    idx1 = np.argwhere(rgb_t == min(rgb_t))
    res_t_rgb.append(idx1[0][0])
    imu_t = abs(ts - imu_ts)
    idx2 = np.argwhere(imu_t == min(imu_t))
    res_t_imu.append(idx2[0][0])
# get the matching frames according to the timestamps

euler_match = imu_euler[res_t_imu]
rgb_match = rgb_data[res_t_rgb]
vi = None

# Run the Modules
translation_filter(imu_acc,euler_match[:,1])
plane_fit(rgb_match,xyz_data,euler_match,euler_match)
    # pitch = angles[1]
    # dx,vi = TF_x(imu_acc[idx:idx + 200],pitch,pitch_prev,vi)
    # pitch_prev = pitch


# np.save('xyz.npy',xyz_data)
bag.close()
# imu_data = pd.read_csv(os.path.join(csv_dir,'imu-data.csv'))
# rgb_data = pd.read_csv(os.path.join(csv_dir,'camera-color-image_raw-compressed.csv'))
# xyz_data = pd.read_csv(os.path.join(csv_dir,'camera-depth-color-points.csv'))
# print("end")
#topics=[, '/camera/depth/color/points','/camera/color/image_raw/compressed__Image'])


