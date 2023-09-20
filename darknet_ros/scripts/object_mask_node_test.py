#!/usr/bin/env python
from detect_helper import *
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import *


def on_detect(msg):
    header = msg.image_header
    print("Converting to mask")
    detect = detect_mask(msg.bounding_boxes,["car",'bus','truck','person'])
    if pub_detect.get_num_connections() > 0:
        m = cv_bridge.cv2_to_imgmsg(detect.astype(np.uint8), encoding = 'mono8')
        m.header.stamp.secs = header.stamp.secs
        m.header.stamp.nsecs = header.stamp.nsecs
        pub_detect.publish(m)

    if pub_detect_color.get_num_connections() > 0:
        m = cv_bridge.cv2_to_imgmsg(mask_to_color(detect,color_map), encoding = 'rgb8')
        m.header.stamp.secs = header.stamp.secs
        m.header.stamp.nsecs = header.stamp.nsecs
        pub_detect_color.publish(m)

if __name__ == "__main__":
    rospy.init_node('detection_node')

    TOPIC_BOXES = rospy.get_param('/topic_boxes', '/darknet_ros/bounding_boxes')
    TOPIC_DETECT = rospy.get_param('/topic_detect', 'detect')
    TOPIC_DETECT_COLOR = rospy.get_param('/topic_detect_color', 'detect_color')

    sub_image = rospy.Subscriber(TOPIC_BOXES, BoundingBoxes, on_detect)
    pub_detect = rospy.Publisher(TOPIC_DETECT, Image, queue_size = 1000)
    pub_detect_color = rospy.Publisher(TOPIC_DETECT_COLOR, Image, queue_size = 1000)

    print("Initializing Python node")
    rospy.spin()
