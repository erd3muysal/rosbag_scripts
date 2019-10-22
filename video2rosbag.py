# -*- coding: utf-8 -*-

import time, sys, os
import roslib, rospy
import cv2
import argparse
from ros import rosbag
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

TOPIC = 'camera/image_raw'
roslib.load_manifest('sensor_msgs')

def video2rosbag(videoFile, bagFile):
    '''Creates a bag file with a video file'''
    try:
        bag = rosbag.Bag(bagFile, 'w')
        cap = cv2.VideoCapture(videoFile)
        bridge = CvBridge()
        
        prop_fps = (cap.get(cv2.CAP_PROP_FPS))
        if prop_fps != prop_fps or prop_fps <= 1e-2:
            print("Warning: Can't get FPS. Assuming 24.")
            prop_fps = 24
        
        #prop_fps = 24
        ret = True
        frame_id = 0
        while(ret):
            ret, frame = cap.read()
            if not ret:
                break

            timestamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
            frame_id += 1
            image = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            image.header.stamp = timestamp
            image.header.frame_id = "camera"
            bag.write(TOPIC, image, timestamp)
        cap.release()

    except (cap.isOpened() == False): 
        print("Error opening video stream or file")
        sys.exit()

    finally:
        bag.close()
    

if __name__ == '__main__':
    parser = argparse.ArgumentParser(prog = 'video2rosbag.py', description='Convert video file to ROS bag file.')

    parser.add_argument('-videoFile', type=str, required=False, help='Name of the video file', default = None)
    parser.add_argument('-bagFile', type=str, required = False, help='Name of the output file converted from the video file', default = None)

    args = parser.parse_args()

    video2rosbag(args.videoFile, args.bagFile)

