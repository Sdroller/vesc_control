#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *

topic_motor_throttle = 'set_throttle_L'
topic_dist_to_person = "/distance_to_person"

class vesc_control:
    def __init__(self):
        self.pub = rospy.Publisher(
            topic_motor_throttle, Int16, queue_size=10)

        self.sub = rospy.Subscriber(topic_dist_to_person,
                                    Float32, self.callback,  queue_size=10)

        # variables for the PID
        self.setpoint = 0.7  # in meters
        self.kp = 3000.0

    def callback(self, sub_distance_to_person):
        # PID code
        distance_to_person = sub_distance_to_person.data
        rospy.loginfo("distance_to_person: %f", distance_to_person)
        motor_throttle = 0.0

        # A reading of 0 means that the distance is invalid
        if distance_to_person > 0.7:
            error = distance_to_person - self.setpoint
            motor_throttle = error * self.kp
            rospy.loginfo("Motor Throttle: %f", motor_throttle)
        self.pub.publish(motor_throttle)


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
