#!/usr/bin/env python
import roslib
import rospy
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# OpenCV
import cv2

def callback(compressed_image):
    now = rospy.get_rostime()
    # timestamp = time.time()
    timestamp = str(now.secs) + str(now.nsecs)
    filename = str(timestamp) + ".jpg"
    image_array = np.fromstring(compressed_image.data, np.uint8)
    image_np = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
    # np.savetxt(filename, image_np, fmt='%i')
    cv2.imwrite(filename,image_np)
    rospy.loginfo("Image saved as %r" %filename )
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber("chatter", String, callback)
    rospy.Subscriber("/camera/left/image_rect_color/compressed", CompressedImage, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()