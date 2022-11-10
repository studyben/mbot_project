#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy
from sensor_msgs.msg import Joy
from mpi_control import MegaPiController

def


if __name__ == "__main__":
    rospy.init_node('megapi_controller')
    rospy.Subscriber('/joy', Joy, mpi_ctrl_node.joy_callback, queue_size=1) 
    
    rospy.spin()