#!/usr/bin/env python
import rospy
from sensor_msgs.msg import camera_0
import cv2
from cv_bridge import CvBridge, CvBridgeError


def show_image(img):
    cv2.imshow("Image Window", img)
    cv2.waitKey(3)


def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image)


if __name__ == "__main__":
    rospy.init_node('rb5_imageShow')
    rospy.Subscriber('/camera_0', image_raw, image_callback, queue_size=1) 