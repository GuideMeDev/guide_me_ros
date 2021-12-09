import numpy as np
import rosbag

# Loading from a rosbag recording
# topic - the source of the message, msg - the message/data of the source, t- timestamp
bag = rosbag.Bag('test.bag')
for topic, msg, t in bag.read_messages(topics=['/imu/data', '/camera/depth/color/points']):
   print(msg)
   bag.close()

