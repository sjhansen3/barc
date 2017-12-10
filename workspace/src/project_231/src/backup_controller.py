#!/usr/bin/env python

""" Creates a node which sends acceleration and positions commands to the car at a specified rate
"""
import rospy
from std_msgs.msg import String
from barc.msg import ECU, Encoder
from time import sleep
import math

class Controller():
	def __init__(self):
		self.ecu_pub = rospy.Publisher('ecu_pwm', ECU, queue_size = 10)
		self.tstep = [0, 1, 2, 3, 4, 5, 6, 7, 8]
		self.acc =   [0, 0, 0, 0, 0, 0, 0, 0, 0]		# m/s^2
		self.vel = 0
		self.acc_est = 0
		self.start_time = 0
		self.counter = -1			# makes sure counter starts at zero because pre-iterating counter during loop
		self.prev_time = 0
		self.prev_ct_time = 0
		self.first_run = 1
		
		self.prev_encoder = 0
		
		self.ROS_RATE = 10		#Hz
		self.RADIUS = 0.05		#meters
	
	def run(self):
		""" Setup the node and send the commands to the car"""
		rospy.init_node('controller', anonymous=True)
		rospy.Subscriber("encoder", Encoder, self.velocity_estimate_callback)
		rospy.Subscriber("ecu", ECU, self.mpc_callback)   
		#self.prev_time = rospy.get_time()
		rate = rospy.Rate(self.ROS_RATE) # 5 hz, 200ms
		sleep(2)
		self.start_time = rospy.get_time()
		while not rospy.is_shutdown() :
			now_time = rospy.get_time()
			if (now_time - self.prev_ct_time) >= 0.98:  # allows for node to run at higher rate but commands to be read at ~1Hz
				self.counter+=1
				self.prev_ct_time = now_time
				if self.counter >= len(self.acc):
					print("should quit")
					break
			
			ecu_cmd = self.get_ecu_cmd(self.acc[self.counter],self.steer[self.counter])
			#self.counter+=1
			rospy.logdebug("Sending ecu command: %s" % ecu_cmd)
			self.ecu_pub.publish(ecu_cmd)        
			rate.sleep()

	def velocity_estimate_callback(self, encoder_data):
		""" Estimate and update the velocity of the car for subscribing to encoder
			@Param encoder_data: data from the Encoder of type Encoder. Contains the counts since the last call		
		"""

		curr_time = rospy.get_time()		
		delta_time = curr_time - self.prev_time
		self.prev_time = curr_time
		
		if delta_time < 10: #throw away the first datapoint by checking time to be a big number		
			alpha = 0.8 #forgetting factor
			
			avg_enc = (encoder_data.FL + encoder_data.FR)/2.0
			
			# prevents initial spike if encoder doesn't start out as 0
			if self.first_run == 1:
				self.prev_encoder = avg_enc
				self.first_run = 0
			
			#convert counts to velocity
			counts_per_second = (avg_enc - self.prev_encoder)/delta_time
			curr_vel = (counts_per_second/8.0)*(2*math.pi*self.RADIUS)
			
			self.prev_encoder = avg_enc
			#average in the new velocity to the old velocity with a forgetting factor
			self.vel = curr_vel#self.vel*(1-alpha)+curr_vel*(alpha)

	def mpc_callback(self, ecu_cmd):
		self.acc = ecu_cmd.motor
		self.steer = ecu_cmd.servo
		
	def crop_float(self, flt):
		return "%.3f" % flt
		
		
	def get_ecu_cmd(self, acc, steer):
		""" convert the commands to the appropriate PWM signals
			@Param acc: The desired acceleration of the car in m/s^2
			@Param steer: The desired steering angle of the car in rad/s		
		"""
		max_servo = 1800
		min_servo = 1200
		max_motor = 2000
		min_motor = 1000
	
		################### CHECK WHAT IS GOING ON HERE
		################### IT WAS WRITTEN SO POSITIVE VELOCITY WAS ACTUALLY NEGATIVE
		################### FIXED IT, BUT THEN IT STARTED TO "RUN AWAY"
		################### SO PUT A NEGATIVE IN THE MTR_PWM SO RETAIN POSITIVE VELOCITY
		if acc > 0:
			motor_pwm = (acc-0.157*self.vel)/0.00305+1500 #TODO convert these mystery numbers into real numbers 
			#use acceleration model
		else:
			motor_pwm = (acc-0.1064*self.vel)/0.003297+1500
			#use deceleration model
		
		servo_pwm = (steer - 1.3784)/-0.00089358 # relationship in radians			(steer-0.4919)/(-3.1882*10^-4)
		
		tm = "time: " + self.crop_float(rospy.get_time()-self.start_time)
		mtr = "\tmotor_pwm: " + self.crop_float(acc) + " : " + self.crop_float(motor_pwm)
		srv = "\tservo_pwm: " + self.crop_float(steer) + " : " + self.crop_float(servo_pwm)
		vel = "\tvelocity: " + self.crop_float(self.vel)
		ctr = "\tcounter: " + self.crop_float(self.counter)
		#make sure we never exceed limits 
		print(tm + ctr + mtr + vel)
		if motor_pwm > max_motor:
			motor_pwm = max_motor
		elif motor_pwm < min_motor:
			motor_pwm = min_motor
	
		if servo_pwm > max_servo:
			servo_pwm = max_servo 
		elif servo_pwm < min_servo:
			servo_pwm = min_servo
		
		return ECU(motor_pwm,servo_pwm)
		

if __name__ == '__main__':
    try:
		controller = Controller()
		controller.run()
    except rospy.ROSInterruptException:
        pass
