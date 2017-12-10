#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from barc.msg import Ultrasound, Encoder, obj_offset
from project_231.msg import State
from time import sleep
import math

class CarState():
	def __init__(self):
		self.state_pub = rospy.Publisher('state', State, queue_size = 10)
		self.first_run = 1
		self.car_vel = 0
		self.car_accl = 0
		
		self.prev_enc_time = 0
		self.prev_encoder = 0
		
		self.prev_us_time = 0
		self.us_dist = 0	# meters
		self.us_rate = 0	# m/s
		self.us_accl = 0	# m/s^2
		
		self.obj_psi = 0 # object meters offset from centerline
		
		self.ROS_RATE = 1	#Hz
		self.RADIUS = .05	#meters

	def run(self):
		rospy.init_node('state', anonymous=True)
		rospy.Subscriber('ultrasound', Ultrasound, self.us_callback)
		rospy.Subscriber('encoder', Encoder, self.car_vel_callback)
		rospy.Subscriber('obj_offset', obj_offset, self.cam_callback)
		rate = rospy.Rate(self.ROS_RATE)
		
		while not rospy.is_shutdown():
			state_msg = self.get_state()
			self.state_pub.publish(state_msg)
			rate.sleep()

	def get_state(self):
		return State(self.car_vel, self.car_accl, self.us_dist, self.us_rate, self.us_accl, self.obj_psi)
		
	def us_callback(self, us_data):
		""" Estimates the distance between the two cars using ultrasound"""
		curr_time = rospy.get_time()
		delta_time = curr_time - self.prev_us_time
		
		# uses curve fit to find distance from the ultrasound value
		
		distance = 0.01103*us_data.right + 0.04001
		# take derivatives to find rate of us distance change
		rate = (distance - self.us_dist) / delta_time
		accl = (rate - self.us_rate) / delta_time
		#print ("US: " + self.us_data.
		# prevents initial spike
		if self.first_run == 1:
			self.us_dist = distance
			self.us_rate = rate
			self.first_run = 0
		
		self.us_rate = rate
		self.us_accl = accl
		self.prev_us_time = curr_time
		self.us_dist = distance
	
	def cam_callback(self, obj_data):
		focal_length = 500 # Experimentally determined
		""" Assigns the estimations from obj_offset camera data to states """
		self.obj_psi = math.atan(obj_data.pixel_center_offset  / focal_length)
	
	def car_vel_callback(self, encoder_data):
		""" Estimate and update the velocity of the car for subscribing to encoder
			@Param encoder_data: data from the Encoder of type Encoder. Contains the counts since the last call		
		"""
		curr_time = rospy.get_time()
		delta_time = curr_time - self.prev_enc_time
		
		avg_enc = (encoder_data.FL + encoder_data.FR)/2.0
			
		# prevents initial spike if encoder doesn't start out as 0
		if self.first_run == 1:
			self.prev_encoder = avg_enc
			self.first_run = 0
		
		# convert counts to velocity
		counts_per_second = (avg_enc - self.prev_encoder)/delta_time
		curr_vel = (counts_per_second/8.0)*(2*math.pi*self.RADIUS)
		
		# sets member variables
		self.prev_enc_time = curr_time
		self.prev_encoder = avg_enc
		self.car_accl = (curr_vel - self.car_vel) / delta_time
		self.car_vel = curr_vel

if __name__ == '__main__':
    try:
		state = CarState()
		state.run()
    except rospy.ROSInterruptException:
        pass
