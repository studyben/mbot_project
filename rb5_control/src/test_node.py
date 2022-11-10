#!/usr/bin/env python
import sys
import rospy
from geometry_msgs.msg import Twist
import numpy as np
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import Joy
import math

"""
The class of the pid controller.
"""

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



class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = None
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1
        self.q_x = None
        self.pub_twist_ctr = rospy.Publisher("/twist", Twist, queue_size=1)
    

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


    def aprilTag_detection(self):
        april_msg = rospy.wait_for_message("/april_poses", PoseArray, timeout=None)
        if len(april_msg.poses) == 0:
            self.q_x = None
            rospy.loginfo("No Vision")
            return
        rospy.loginfo(april_msg.poses[0])
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

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        self.I = np.array([0.0,0.0,0.0]) 
        self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the different between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result 

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv
    


    def vision_adjust(self, curr_pos, tar_position, tar_pose):
        self.aprilTag_detection()
        if self.q_x != None:
            roll, pitch, yaw = self.euler_from_quaternion(self.q_x, self.q_y, self.q_z, self.q_w)
        else:
            return curr_pos
        return np.array([curr_pos[0] + self.tagZ - tar_position[2], 
                                curr_pos[1] + self.tagX - tar_position[2], roll])



    def update(self, currentState):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep 
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value. 
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist):
    """
    Convert the twist to twist msg.
    """
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0] 
    twist_msg.linear.y = desired_twist[1] 
    twist_msg.linear.z = 0
    twist_msg.angular.x = 0
    twist_msg.angular.y = 0
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)
    


if __name__ == "__main__":
    import time
    rospy.init_node("hw1")
    pub_twist = rospy.Publisher("/twist", Twist, queue_size=1)

    waypoint = np.array([[0.0,0.0,0.0], 
                            [1.0,0.0,0.0],
                      [1.0, 2.0, np.pi],[0, 0, 0]]) 
    
    pose_position = np.array([[0.0307, -0.084, 0.645],
                            [0.156, 0.026, 1.252],
                            [0.086, -0.0295, 1.397]])
    pose_orient = np.array([[-0.13, -0.09, -0.69, 0.705],
                            [-0.09, 0.172, 0.704, 0.682],
                            [-0.160, -0.106, -0.685, 0.703]])

    # init pid controller
    pid = PIDcontroller(0.02,0.005,0.005)
    

    # init current state
    current_state = np.array([0.0,0.0,0.0])

    # in this loop we will go through each way point.
    # once error between the current state and the current way point is small enough, 
    # the current way point will be updated with a new point.
    for itm, wp in enumerate(waypoint):
        print("move to way point", wp)
        # set wp as the target point
        pid.setTarget(wp)

        # calculate the current twist
        update_value = pid.update(current_state)
        # publish the twist
        pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
        #print(coord(update_value, current_state))
        time.sleep(0.05)
        # update the current state
        current_state += update_value
        while(np.linalg.norm(pid.getError(current_state, wp)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid.update(current_state)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state += update_value

        if itm == 0:
            continue
        pid_vision = PIDcontroller(0.02,0.005,0.005)
        current_state2 = np.array([0.0, 0.0, 0.0])
        time.sleep(1)
        rospy.loginfo("start vision control")
        targ_vision = pid_vision.vision_adjust(np.array([0.0, 0.0, 0.0]), pose_position[itm - 1], pose_orient[itm - 1])
        rospy.loginfo(targ_vision)

        pid_vision.setTarget(targ_vision)
        
        update_value = pid_vision.update(current_state2)
        pub_twist.publish(genTwistMsg(coord(update_value, current_state2)))
        time.sleep(0.05)
        while(np.linalg.norm(pid_vision.getError(current_state2, targ_vision)) > 0.05): # check the error between current state and current way point
            # calculate the current twist
            update_value = pid_vision.update(current_state2)
            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, current_state2)))
            #print(coord(update_value, current_state))
            time.sleep(0.05)
            # update the current state
            current_state2 += update_value

    # stop the car and exit
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0])))

