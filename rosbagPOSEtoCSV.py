#!/usr/bin/env python3
import rosbag
import csv
import tf
import geometry_msgs.msg


bag_file = "/home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/controlledEnv/lidar/bags/ctrlEnvT4.bag"
output_file = "/home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/controlledEnv/lidar/kartoctrldEnv.csv"
# Open the bag file
bag = rosbag.Bag(bag_file)

with open(output_file, 'w') as csvfile:
    fieldnames = ['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    for topic, msg, t in bag.read_messages(topics=['/odom']):
        quat = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quat)
        writer.writerow({'time': t.to_sec(),
                         'x': msg.pose.pose.position.x,
                         'y': msg.pose.pose.position.y,
                         'z': msg.pose.pose.position.z,
                         'qx': euler[0],
                         'qy': euler[1],
                         'qz': euler[2],
                         'qw': msg.pose.pose.orientation.w})

bag.close()
