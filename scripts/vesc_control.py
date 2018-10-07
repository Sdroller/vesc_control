#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import *
import message_filters

topic_motor_speed_L = "/motor_L/commands/motor/speed"
topic_motor_speed_R = "/motor_R/commands/motor/speed"

topic_dist_to_person_raw = "/distance_to_person"
topic_dist_to_person_filtered = "/distance_to_person_filtered"
topic_position_y = 'centroid_pos_y'

class vesc_control:
    def __init__(self):
        self.pub_L = rospy.Publisher(topic_motor_speed_L,
                                    Float64, queue_size=10)

        self.pub_R = rospy.Publisher(topic_motor_speed_R,
                                    Float64, queue_size=10)

        # We use time TimeSynchronizer to sub to multiple topics
        self.sub_x = message_filters.Subscriber(topic_dist_to_person_filtered, Float32)
        self.sub_y = message_filters.Subscriber(topic_position_y, Float32)        

        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub_x, self.sub_y], 5, 1, allow_headerless=True)
        self.ts.registerCallback(self.callback)       

        # variables for the PID
        self.setpoint = 1.25  # in meters
        self.kp_x = 2500.0
	self.kp_y = 5.0

    def callback(self, sub_position_x, sub_position_y):

        # get data from the subscribers
        centroid_pos_x = sub_position_x.data
        rospy.loginfo("distance_to_person: %f", centroid_pos_x)
        centroid_pos_y = sub_position_y.data
        rospy.loginfo("centroid_pos_y: %f", centroid_pos_y)

	motor_speed_sym = 0.0
	motor_speed_asym = 0.0

	# Calculate the symmetric velocity component based on the distance to the user
        # A reading of 0 means that the distance is invalid
        if (centroid_pos_x > 0.5) or (centroid_pos_x < 2.0):
            error = centroid_pos_x - self.setpoint
            motor_speed_sym = error * self.kp_x
	else:
	    motor_speed_sym = 0.0

	# Calculate the asymmetric velocity component based on the user's diplacement from the center of the video frame
        # centroid reads 0 when no person is present
        error = centroid_pos_y - 672/2 #self.img_width/2 # diff from center of image
        #if np.abs(error)/self.img_width > 0.1: # if centroid is >10% off center
    	motor_speed_asym = error * self.kp_y


	rospy.loginfo("Setpoint: %f", self.setpoint)
	rospy.loginfo("Error: %f", error)
	rospy.loginfo("Motor Speed Sym: %f", motor_speed_sym)
	rospy.loginfo("Motor Speed Asym: %f", motor_speed_asym)
	rospy.loginfo("Motor Speed L: %f", motor_speed_sym + motor_speed_asym)
	rospy.loginfo("Motor Speed R: %f", -1*(motor_speed_sym - motor_speed_asym))
        self.pub_L.publish(-1*(motor_speed_sym - motor_speed_asym))
	self.pub_R.publish((motor_speed_sym + motor_speed_asym))


def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node("vesc_controL_node", anonymous=True)
    vc = vesc_control()
    rospy.loginfo("Vesc_control constructor finished")
    #rospy.init_node('vesc_control_node', anonymous=True)
    rospy.loginfo("Yolo Depth Measure node started")

    try:
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main(sys.argv)
