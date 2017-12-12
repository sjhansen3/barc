#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from barc.msg import Ultrasound, Encoder, obj_offset, ECU
from project_231.msg import State
from time import sleep
import math

class linearController():
    def __init__(self):
        self.state_pub = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)
        self.ROS_RATE = 10	#Hz
        self.Kp = 1

    def run(self):
    	rospy.init_node('linearController', anonymous=True)
        rospy.Subscriber('state', State, self.state_callback)
        rate = rospy.Rate(self.ROS_RATE)
        while not rospy.is_shutdown():
            self.send_command()
            rate.sleep()

    self.control_action(self):
        acc=kp*error
        command = ECU()
        command.motor = 
        command.servo = 
        self.state_pub(command)

    self.state_callback(self):


if __name__ == '__main__':
    try:
		linearController = CarState()
		linearController.run()
    except rospy.ROSInterruptException:
        pass
