#!/usr/bin/env python
from detect_helper import *
import numpy as np
import rospy
import cv_bridge
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import *


def on_detect(msg):
    on_detect.last_image = msg

on_detect.last_image = None

if __name__ == "__main__":
    rospy.init_node('detection_node')

    TOPIC_BOXES = rospy.get_param('/topic_boxes', '/darknet_ros/bounding_boxes')
    #TOPIC_OBJECTS = rospy.get_param('/topic_boxes', '/darknet_ros/found_object')
    TOPIC_DETECT = rospy.get_param('/topic_detect', 'detect')
    TOPIC_DETECT_COLOR = rospy.get_param('/topic_detect_color', 'detect_color')
    RATE = rospy.get_param('~rate', 500.0)

    sub_image = rospy.Subscriber(TOPIC_BOXES, BoundingBoxes, on_detect)
    #sub_object = rospy.Subscriber(TOPIC_BOXES, ObjectCount, on_img)
    pub_detect = rospy.Publisher(TOPIC_DETECT, Image, queue_size = 1000)
    pub_detect_count = rospy.Publisher('detect_count', ObjectCount, queue_size = 1000)
    pub_detect_color = rospy.Publisher(TOPIC_DETECT_COLOR, Image, queue_size = 1000)

    rate = rospy.Rate(RATE)
    rospy.loginfo("Initializing Python node")

    while not rospy.is_shutdown():
        rate.sleep()

        if on_detect.last_image is not None:
            rospy.loginfo("Converting to mask")
            header = on_detect.last_image.image_header
            #t_begin = rospy.Time.now()
            detect,counter = detect_mask(on_detect.last_image.bounding_boxes,["car",'bus','truck','person'])
            on_detect.last_image = None
            #t_end = rospy.Time.now()
            #rospy.loginfo(t_end - t_begin)
            if pub_detect_count.get_num_connections() > 0:
                m = ObjectCount()
                m.count = counter
                m.header.stamp.secs = header.stamp.secs
                m.header.stamp.nsecs = header.stamp.nsecs
                pub_detect_color.publish(m)
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
        else:
            rospy.loginfo("Waiting for boxes")

