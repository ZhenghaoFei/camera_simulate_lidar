#!/usr/bin/env python
import time
import numpy as np
from std_msgs.msg import String

import roslib
import rospy
from sensor_msgs.msg import LaserScan,Image
import message_filters
from cv_bridge import CvBridge
# OpenCV
import cv2

def rimgstore(right_image):
	filename = str(right_image.header.stamp.to_sec())+"right"+".png"
	cv_bridge = CvBridge()
	#right image
	cv_image = cv_bridge.imgmsg_to_cv2(right_image,"bgr8")
	image_np = np.asarray(cv_image)
	# print image_np.shape
	image_np = image_np[128:256,]
	print image_np.shape
	# np.savetxt(filename, image_np, fmt='%i')
	cv2.imwrite(filename,image_np)
	rospy.loginfo("Image saved as %r" %filename)

def limgstore(left_image):
	filename = str(left_image.header.stamp.to_sec())+"left"+".png"
	cv_bridge = CvBridge()
	#right image
	cv_image = cv_bridge.imgmsg_to_cv2(left_image,"bgr8")
	image_np = np.asarray(cv_image)
	# print image_np.shape
	image_np = image_np[128:256,]
	# np.savetxt(filename, image_np, fmt='%i')
	cv2.imwrite(filename,image_np)
	rospy.loginfo("Image saved as %r" %filename)

def scanstore(laser_scan):
    filename = str(laser_scan.header.stamp.to_sec()) + ".txt"
    scan_data = np.asarray(laser_scan.ranges)
    scan_data = scan_data[120:240]
    scan_data[scan_data==np.inf] = 10
    # print scan_data.shape
    np.savetxt(filename, scan_data[None,:], delimiter=',',fmt='%.10f',newline=' ')
    rospy.loginfo("laser scan saved as %r", filename)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/zed/right/image_raw_color",  Image,  rimgstore)
    rospy.Subscriber("/zed/left/image_raw_color",   Image,  limgstore)
    rospy.Subscriber("/scan",LaserScan,scanstore)
    # rospy.Subscriber("/zed/left/image_raw_color/", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
