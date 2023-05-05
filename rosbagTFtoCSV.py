#!/usr/bin/env python3
import rosbag
import tf
import csv

bag_file = "/home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/hospital/lidar/bags/kartoHospital.bag"
output_file = "/home/linus/o2s_ws/src/o2s_robot/bagfile/raadDataset/hospital/lidar/kartoHospital.csv"

# Open the rosbag
bag = rosbag.Bag(bag_file)

with open(output_file, 'w') as csvfile:
    fieldnames = ['time', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()

    for topic, msg, t in bag.read_messages(topics=['/tf']):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                quat = (transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w)
                euler = tf.transformations.euler_from_quaternion(quat)
                writer.writerow({'time': t.to_sec(),
                                 'x': transform.transform.translation.x,
                                 'y': transform.transform.translation.y,
                                 'z': transform.transform.translation.z,
                                 'qx': euler[0],
                                 'qy': euler[1],
                                 'qz': euler[2],
                                 'qw': transform.transform.rotation.w})

bag.close()
