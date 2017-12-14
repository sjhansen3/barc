#!/usr/bin/env python
import time
import rospy
from barc.msg import ECU
from project_231.msg import State
import numpy as np
import matplotlib.pyplot as plt

""" listen to the controller  ECU commands, 
propigate the state forward and return the state to sthe controller
"""

class Simulator():
    def __init__(self):
        self.ecu_sub = None
        self.dt = .1
        #publish the state
        self.state_pub = None

        self.vel = 0
        self.acc = 0
        self.us_rate = 0
        self.us_dist = 0
        self.obj_psi = 0
        self.counter = 0
       
        self.car_vels = []
        self.car_accls = []
        self.us_distances = []
        self.motor_cmds = []
        self.commands = []
   
    def run(self):
        rospy.init_node('simulator')
        self.ecu_sub = rospy.Subscriber('ecu_pwm',ECU, self._simulate)
        self.state_pub = rospy.Publisher('state', State, queue_size = 10)
        rospy.on_shutdown(self.save)
        rospy.spin()

    def _simulate(self, ecu_msg):
        motor_cmd=ecu_msg.motor
        servo_cmd=ecu_msg.servo

        prev_vel = self.vel
        prev_acc = self.acc
        prev_usrate = self.us_rate
        prev_usdist = self.us_dist
        prev_obj_psi = self.obj_psi
        
        car_vel = prev_vel+prev_acc*self.dt
        #rospy.loginfo("prev_vel {} prev_acc {} dt {}".format(prev_vel,prev_acc,self.dt))
        if motor_cmd>0:
            car_acc = (0.003050*(motor_cmd-1500)-0.1570*car_vel)#-1500
        else:
            car_acc = (0.003297*(motor_cmd-1500)-0.1064*car_vel)#-1500
        
        obj_vel = .5 #assume constant 1 m/s velocity of other car
        obj_acc = 0 #no acceleration

        us_rate = obj_vel-car_vel #negative moving towards the other car
        us_dist = 0.5*(obj_acc-car_acc)*self.dt**2+us_rate*self.dt+prev_usdist
        us_accl = 0 #I dont think this is used by MPC #TODO confirm this
        obj_psi = 0 #assume angles dont matter for the moment
        
        new_state = State()
        new_state.car_vel  = car_vel
        new_state.car_accl = car_acc
        new_state.us_dist = us_dist
        new_state.obj_psi = obj_psi

        self.vel = car_vel
        self.acc = car_acc
        self.us_rate = us_rate
        self.us_dist = us_dist
        self.obj_psi = obj_psi
        
        #TODO append
        self.car_vels.append(car_vel)
        self.car_accls.append(car_acc)
        self.us_distances.append(us_dist)
        self.motor_cmds.append(motor_cmd)
        rospy.loginfo("car vel: {} car_ac: {} us_dist: {} motor_cmd: {}".format(car_vel,car_acc,us_dist,motor_cmd))

        self.state_pub.publish(new_state)

    def save(self):
        rospy.loginfo("saving...")
        time_ar = np.linspace(0,self.dt*20,21)[0:-1]

        vels = np.asarray(self.car_vels)
        caraccls = np.asarray(self.car_accls)
        motor_cmds = np.asarray(self.motor_cmds)
        us_distances = np.asarray(self.us_distances)
        
        array_tofile = np.vstack((vels,caraccls,motor_cmds,us_distances))
        np.save('test2_stepresponse',array_tofile)
        
        #pass

if __name__ == '__main__':
    try:
        sim = Simulator()
    	sim.run()
    except rospy.ROSInterruptException:
        #sim.save()
        pass
