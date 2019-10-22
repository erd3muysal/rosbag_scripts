# -*- coding: utf-8 -*-

import pandas as pd
import argparse
import rospy
import rosbag
from mavros_msgs.msg import Altitude
from std_msgs.msg import Int32, Float64, String

TOPIC = '/mavros/altitude'

def csv2rosbag(csvFile, outputFile):
    log = pd.read_csv(csvFile)
    try:
        with rosbag.Bag(outputFile, 'w') as bag:
            for row in range(log.shape[0]):
                timestamp = rospy.Time.from_sec(log['Time(seconds)'][row])
                altitude_msg = Altitude()
                altitude_msg.header.stamp = timestamp
                altitude_msg.local = (log['Altitude(meters)'][row])

                # Populate the data elements for IMU
                # e.g. imu_msg.angular_velocity.x = df['a_v_x'][row]

                bag.write(TOPIC, altitude_msg, timestamp)
                print(bag)
    finally:
        bag.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = 'csv2rosbag.py', description='Convert csv file to ROS bag file.')

    parser.add_argument('-csvFile', type=str, required=False, help='Name of the csv file', default = None)
    parser.add_argument('-outputFile', type=str, required = False, help='Name of the output file converted from the csv file', default = None)

    args = parser.parse_args()

    csv2rosbag(args.csvFile, args.outputFile)

