#!/usr/bin/env python
""" MegaPi Controller ROS Wrapper"""
import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Joy
import math


p_z = 0.0
z_thresh = 0.35


def print_result(april_msg):
    if len(april_msg.poses) == 0:
        rospy.loginfo("no info")
        return
        pose_one = april_msg.poses[0]
        position = pose_one.position
        rospy.loginfo(position.z)


class VisionControlModel:
    def __init__(self, init_x, init_y, init_theta):

        self.publisher = rospy.Publisher("/joy", Joy, queue_size=1)
        rospy.init_node("VisionControlModel")
        rospy.loginfo("Mbot Control Node start")
        

        # Parameters for mbot
        self.fwd = 0.285 # m/s
        self.rvs = 0.285 # m/s
        self.ccw = 1.47 # rad/s
        self.cw = 1.47  # rad/s
        self.right = 0.58 # m/s
        self.left = 0.52 # m/s
        self.rotation = rospy.Rate(50)
        self.curr_x, self.curr_y, self.curr_the = init_x, init_y, init_theta


        #self.publisher = rospy.publisher("/joy", Joy, queue_size=10)      
        self.joy_msg = Joy()
        self.joy_msg.axes = [0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0]
        self.joy_msg.buttons = [0, 0, 0, 0, 0, 0, 0, 0]
        self.publisher.publish(self.joy_msg)

        rospy.sleep(0.5) # seconds


    def aprilTag_detection(self):
        april_msg = rospy.wait_for_message("/april_poses", PoseArray, timeout=None)
        if len(april_msg.poses) == 0:
            return
        pose_one = april_msg.poses[0]
        position = pose_one.position
        self.tagZ = position.z
        self.tagX = position.x 
        self.tagY = position.y
        self.q_x = pose_one.orientation.x
        self.q_y = pose_one.orientation.y
        self.q_z = pose_one.orientation.z 
        self.q_w = pose_one.orientation.w


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z # in radians


        if distance > 0:
            times = (abs(distance)/ self.fwd)
            self.joy_msg.axes[0] = 0.5
            self.publisher.publish(self.joy_msg)
            curr_times = rospy.get_rostime().secs 

            while rospy.get_rostime().secs - curr_times < times:
                rospy.spinOnce()
                self.rotation.sleep()
        else:
            times = (abs(distance)/ self.rvs)
            self.joy_msg.axes[0] = -0.5
            self.publisher.publish(self.joy_msg)
            curr_times = rospy.get_rostime().secs 

            while rospy.get_rostime().secs - curr_times < times:
                rospy.spinOnce()
                self.rotation.sleep()

        self.joy_msg.axes[0] = 0.0

        self.publisher.publish(self.joy_msg)


    def pid_control(self, X_target, Z_target, Yaw_target):
        self.aprilTag_detection()
        roll, pitch, yaw = self.euler_from_quaternion(self.q_x, self.q_y, self.q_z, self.q_w)
        deltaZ = self.tagZ - Z_target
        deltaX = self.tagX - X_target
        delta_yaw = -(roll - Yaw_target)

        x_p = deltaX * 300
        roll_p = roll * 300

        while abs(delta_yaw) > 0.1:
            self.aprilTag_detection()
            roll, pitch, yaw = self.euler_from_quaternion(self.q_x, self.q_y, self.q_z, self.q_w)
            deltaZ = -(self.tagZ - Z_target)
            deltaX = -(self.tagX - X_target)
            delta_yaw = -(roll - Yaw_target)
            x_p = max(min(deltaX * 10, 0.4), 0)
            roll_p = max(min(delta_yaw * 10, 0.4), 0)
            z_p = max(min(deltaZ * 5, 0.4), 0)

            if delta_yaw < 0:
                while delta_yaw < -0.12:
                    self.rotation.sleep()
                    self.aprilTag_detection()
                    roll, pitch, yaw = self.euler_from_quaternion(self.q_x, self.q_y, self.q_z, self.q_w)
                    delta_yaw = -(roll - Yaw_target)
                    x_p = min(abs(delta_yaw) * 10, 0.4)
                    
                    rospy.loginfo("The control input: %s, current roll: %s; target roll: %s; curr qy: %s", x_p, roll, Yaw_target, self.q_y)
                    self.joy_msg.axes[2] = x_p
                    self.publisher.publish(self.joy_msg)
                    
                    
            elif delta_yaw > 0:
                while delta_yaw > 0.12:
                    self.rotation.sleep()
                    self.aprilTag_detection()
                    roll, pitch, yaw = self.euler_from_quaternion(self.q_x, self.q_y, self.q_z, self.q_w)
                    delta_yaw = -(roll - Yaw_target)
                    x_p = min(abs(delta_yaw) * 10, 0.4)
                    rospy.loginfo("The control input: %s, current roll: %s; target roll: %s; curr qy: %s", x_p, roll, Yaw_target, self.q_y)
                    self.joy_msg.axes[2] = -x_p
                    self.publisher.publish(self.joy_msg)
                    
        self.joy_msg.axes[2] = 0.0

        self.publisher.publish(self.joy_msg)
        self.aprilTag_detection()
        rospy.loginfo("reached the position: ")
        rospy.loginfo("yaw: %s", self.q_y)
            # if deltaX < 0:
            #     while deltaX < -0.1:
            #         deltaX = self.tagX - X_target
            #         x_p = min(abs(deltaX) * 10, 0.7)
            #         self.joy_msg.axes[2] = -x_p
            #         self.publisher.publish(self.joy_msg)
            #         self.rotation.sleep()
            # elif deltaX > 0:
            #     while deltaX > 0.1:
            #         deltaX = self.tagX - X_target
            #         x_p = min(abs(deltaX) * 10, 0.7)
            #         self.joy_msg.axes[2] = x_p
            #         self.publisher.publish(self.joy_msg)
            #         self.rotation.sleep()
            
            # if deltaZ < 0:
            #     while deltaZ 
        

    def move(self, new_x, new_y, new_theta, X_tag, Z_tag):
        self.pid_control(0.071, 0.64, 0.00)


if __name__ == "__main__":
    #rospy.init_node('april_poses_reader')
    vision_control = VisionControlModel(0, 0, 0)
    vision_control.move(1, 1, 1, 1,1)
    
    #rospy.Subscriber('/april_poses', PoseArray, vision_control.aprilTag_detection, queue_size=1) 
        