#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Twist
import numpy as np
from pid_controller import PID


if __name__ == "__main__":
    rospy.init_node("open_loop_control")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)