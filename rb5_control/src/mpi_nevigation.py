#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
import numpy as np 

class ControlModel:
    def __init__(self, init_x, init_y, init_theta):

        # Parameters for mbot
        self.fwd = 0.17 # m/s
        self.rvs = 0.19 # m/s
        self.ccw = 1.20 # rad/s
        self.cw = 1.15  # rad/s
        self.right = 0.58 # m/s
        self.left = 0.52 # m/s
        self.curr_x, self.curr_y, self.curr_the = init_x, init_y, init_theta

        rospy.init_node("controlModel")
        rospy.loginfo("Mbot Control Node start")

        self.publisher = rospy.publisher("/joy", Joy, queue_size=10)      
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.publisher.publish(self.joy_msg)

        rospy.sleep(0.5) # seconds

    def forward(self, distance):
        if distance > 0:
            times = (abs(distance)/ self.fwd)
            self.joy_msg.axes[1] = 0.5
        else:
            times = (abs(distance)/ self.rvs)
            self.joy_msg.axes[1] = -0.5
        
        self.publisher.publish(self.joy_msg)

        rospy.sleep(times)

        self.joy_msg.axes[1] = 0.0

        self.publisher.publish(self.joy_msg)
    
    def rotate(self, angle):
        if angle > 0:
            times = (abs(angle) / self.ccw)
            self.joy_msg.axes[2] = 0.5
        else:
            times = (abs(angle)/ self.cw)
            self.joy_msg.axes[2] = -0.5

        rospy.sleep(times)

        self.joy_msg.axes[2] = 0.0

        self.publisher.publish(self.joy_msg)

    def slide(self, distance):
        if distance > 0:
            times = (abs(distance)/ self.fwd)
            self.joy_msg.axes[0] = 0.5
        else:
            times = (abs(distance)/ self.rvs)
            self.joy_msg.axes[0] = -0.5

        rospy.sleep(times)

        self.joy_msg.axes[0] = 0.0

        self.publisher.publish(self.joy_msg)

    def move(self, new_x, new_y, new_theta):
        delta_d = [new_x - self.curr_x, new_y - self.curr_y]
        distance = np.linalg.norm(delta_d)

        angle = np.math.atan2(delta_d[1], delta_d[0])

        angle = np.floor(angle * 100) / 100 if angle > 0 else np.ceil(angle * 100) / 100

        delta_angle = angle - self.curr_the

        if ((delta_angle == 0.0) or (abs(delta_angle) == 3.14)):
            if self.curr_the == angle:
                rospy.loginfo("Moving Forward %s m", dist)
                self.forward(distance)
            else:
                rospy.loginfo("Moving Backward %s m", dist)
                self.forward(-distance)
        elif (abs(delta_angle) == 1.57):
            if delta_theta == 1.57:
                rospy.loginfo("Slide Left: - %s m", dist)
                mpi_Nav.slide(-distance)
            else:
                rospy.loginfo("Slide Right: %s m", dist)
                mpi_Nav.slide(distance)
        else:
            rospy.loginfo("Rotating CCW: %s rad", delta_angle) if delta_angle > 0 else rospy.loginfo("Rotating CW: - %s rad", delta_angle)
            self.rotate(delta_theta)
            self.curr_the = angle
            rospy.loginfo("Then, Moving Forward: %s m", dist)
            self.forward(dist)

        angle_change = new_theta - self.curr_the

        if angle_change != 0.0:
            rospy.loginfo("Angle adjust %s rad", angle_change)

            self.rotate(angle_change)

        self.curr_x, self.curr_y, self.curr_the = new_x, new_y, new_theta


if __name__ == "__main__":
    movingLog = np.array([[0, 0, 0], [-1, 0, 0], [-1, 1, 1.57], [-2, 1, 0], [-2, -2, -1.57], [-1, 1, -0.78], [0, 0, 0]])
    [init_x, init_y, init_theta] = movingLog[0, :]

    moveControl = ControlModel(init_x, init_y, init_theta)

    # for i in range(movingLog.shape[0] - 1):
    #     [new_x, new_y, new_theta] = movingLog[i + 1, :]
    #     moveControl.move(new_x, new_y, new_theta)
    #     rospy.loginfo("Waypoint Reached ==============================")
