#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *

def talker():
    rospy.init_node('vesc_control_node', anonymous=True)
    pub_R = rospy.Publisher('/motor_R/commands/motor/current', Float64, queue_size=10)
    pub_L = rospy.Publisher('/motor_L/commands/motor/current', Float64, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        values=1.0
        rospy.loginfo(values)
        pub_L.publish(values)
	pub_R.publish(-1*values)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
