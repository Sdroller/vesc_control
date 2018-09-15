#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from std_msgs.msg import Float32

topic_motor_throttle_L = 'set_throttle_L'
topic_motor_throttle_R = 'set_throttle_R'
topic_motor_brake_L = 'set_brake_L'
topic_motor_brake_R = 'set_brake_R'
topic_dist_to_person = "/distance_to_person"
topic_position_x = 'centroid_pos_x'



# ADD ADDITIONAL SUBSCRIBER LOOKING AT CENTROID x LOCATION
# CALCULATE THE THROTTLE SIGNALS FOR CORRECTION
# ADD THROTTLE SIGNALS TO SYMMETRIC THROTTLE SIGNALS
# THIS IS CHALLENGING AND IMPRECISE SINCE WE ARE USING THROTTLE INSTEAD OF WHEEL VELOCITY
class vesc_control:
    def __init__(self):
        self.pub_TL = rospy.Publisher(
            topic_motor_throttle_L, Float32, queue_size=10)

        self.pub_TR = rospy.Publisher(
            topic_motor_throttle_R, Float32, queue_size=10)
        
        self.pub_BL = rospy.Publisher(
            topic_motor_brake_L, Float32, queue_size=10)
                
        self.pub_BR = rospy.Publisher(
            topic_motor_brake_R, Float32, queue_size=10)

        self.sub_z = rospy.Subscriber(topic_dist_to_person,
                                    Float32, self.callback_z,  queue_size=10)
        
        self.sub_x = rospy.Subscriber(topic_position_x,
                                    Float32, self.callback_x,  queue_size=10)

        # variables for the PID
        self.setpoint = 0.7  # in meters
        self.kp_z = 10000.0 # gain for distance pid
        self.kp_x = 1000.0 # gain for turning pid
        self.wheelbase = 36 # inches
        self.wheel_radius = 8 # inches
        self.motor_throttle_asym_L = 0.0
        self.motor_throttle_asym_R = 0.0
        self.motor_throttle_sym = 0.0
        self.img_width = 376

    def callback_z(self, sub_position_z):
        # subrcriber provides a data and the throttle and brake topics are updated
        # PID code
        centroid_pos_z = sub_position_z.data
        rospy.loginfo("distance_to_person: %f", centroid_pos_z)
        self.motor_throttle_sym = 0.0

        # A reading of 0 means that the distance is invalid
        # calculate the throttle from the PID to keep the distance at the setpoint
        if centroid_pos_z > 0.35:
            error = centroid_pos_z - self.setpoint
            self.motor_throttle_sym = error * self.kp_z
            rospy.loginfo("Throttle Symmetric: %f", self.motor_throttle_sym)
        self.publish_throttle()    
    
    def callback_x(self, sub_position_x):
        # subrcriber provides horiz position data and the throttle and brake topics are updated
        # PID code
        centroid_pos_x = sub_position_x.data
        rospy.loginfo("centroid_pos_x: %f", centroid_pos_x)
        self.motor_throttle_asym_L = 0.0
        self.motor_throttle_asym_R = 0.0

        # calculate the throttle reading necessary to turn the stroller
        # this is another basic PID loop tying the deflection angle to the wheel speeds
        # centroid reads 0 when no person is present
        if np.abs(centroid_pos_x - 336)/672 > 0.1: # if centroid is >10% off center
            error = centroid_pos_x - self.img_width/2
            asym = error * self.kp_x
            self.motor_throttle_asym_L = asym
            self.motor_throttle_asym_R = -asym
            rospy.loginfo("Throttle Asymmetric: %f", self.motor_throttle_sym)
        self.publish_throttle()

    def publish_throttle(self):
        # publish to the two throttle topics, sum the throttle resonse for turning and distance maintenance.
        throttle_L = self.motor_throttle_sym + self.motor_throttle_asym_L
        throttle_R = self.motor_throttle_sym + self.motor_throttle_asym_R
        rospy.loginfo("Motor Throttle L: %f", throttle_L)
        rospy.loginfo("Motor Throttle R: %f", throttle_R)   
        if throttle_L > 100:
            throttle_L = 100
        if throttle_R > 100:
            throttle_R = 100
        self.pub_TL.publish(throttle_L)
        self.pub_TR.publish(throttle_R)

def main(args):
    '''Initializes and cleanup ros node'''
    vc = vesc_control()
    rospy.init_node('vesc_control_node', anonymous=True)
    rospy.loginfo("Yolo Depth Measure node started")

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
