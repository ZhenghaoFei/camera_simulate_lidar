#!/usr/bin/env python
import roslib
import rospy
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
# from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

# OpenCV
import cv2

def rimgstore(right_image):
    # now = rospy.get_rostime()
    # # timestamp = time.time()
    # timestamp = "right" + str(now.secs) + str(now.nsecs)
    # filename = str(timestamp) + ".jpg"
    filename = str(right_image.header.stamp.to_sec())+"right"+".png"
    image_array = np.fromstring(right_image.data, np.uint8)
    image_np = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    # print image_np.shape
    image_np = image_np[128:256,]
    # np.savetxt(filename, image_np, fmt='%i')
    cv2.imwrite(filename,image_np)
    rospy.loginfo("Image saved as %r" %filename)
def limgstore(left_image):
    filename = str(left_image.header.stamp.to_sec())+"left"+".png"
    image_array = np.fromstring(left_image.data, np.uint8)
    image_np = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    # print image_np.shape
    image_np = image_np[128:256,]
    # np.savetxt(filename, image_np, fmt='%i')
    cv2.imwrite(filename,image_np)
    rospy.loginfo("Image saved as %r" %filename)
def scanstore(laser_scan):
    filename = str(laser_scan.header.stamp.to_sec()) + ".txt"
    scan_data = np.asarray(laser_scan.ranges)
    scan_data[scan_data==np.inf] = 10
    scan_data = scan_data[125:235]    
    # print scan_data.shape
    np.savetxt(filename, scan_data[None,:], delimiter=',',fmt='%.4f',newline=' ')
    rospy.loginfo("laser scan saved as %r", filename)
    # print scan_data.shape
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/zed/left/image_rect_color/compressed",  CompressedImage, limgstore)
    rospy.Subscriber("/zed/right/image_rect_color/compressed", CompressedImage, rimgstore)
    rospy.Subscriber("/scan",LaserScan,scanstore)
    # rospy.Subscriber("/zed/left/image_raw_color/", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()