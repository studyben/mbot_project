 def forward(self, distance):
        if distance > 0:
            times = (abs(distance)/ self.fwd)
            curr_times = rospy.get_rostime().secs 
            
            self.joy_msg.axes[1] = 0.5
            self.publisher.publish(self.joy_msg)

            while rospy.get_rostime().secs - curr_times < times:
                rospy.spinOnce()
                self.rotation.sleep()
        else:
            times = (abs(distance)/ self.rvs)
            self.joy_msg.axes[1] = 0.5
            self.publisher.publish(self.joy_msg)

            curr_times = rospy.get_rostime().secs 
            while rospy.get_rostime().secs - curr_times < times:
                rospy.spinOnce()
                self.rotation.sleep()
        
        
        self.joy_msg.axes[1] = 0.0

        self.publisher.publish(self.joy_msg)
    
    def rotate(self, angle):
        if angle > 0:
            times = (abs(angle) / self.ccw)
            self.joy_msg.axes[2] = 0.5
            self.publisher.publish(self.joy_msg)
            curr_times = rospy.get_rostime().secs 

            while rospy.get_rostime().secs - curr_times < times:
                self.read_AprilTag()
                self.rotation.sleep()
        else:
            times = (abs(angle)/ self.cw)
            self.joy_msg.axes[2] = -0.5
            self.publisher.publish(self.joy_msg)
            curr_times = rospy.get_rostime().secs 

            while rospy.get_rostime().secs - curr_times < times:
                self.read_AprilTag()
                self.rotation.sleep()


        self.joy_msg.axes[2] = 0.0

        self.publisher.publish(self.joy_msg)

    def slide(self, distance):