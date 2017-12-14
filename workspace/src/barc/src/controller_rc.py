#!/usr/bin/env python

# ---------------------------------------------------------------------------
# Licensing Information: You are free to use or extend these projects for
# education or reserach purposes provided that (1) you retain this notice
# and (2) you provide clear attribution to UC Berkeley, including a link
# to http://barc-project.com
#
# Attibution Information: The barc project ROS code-base was developed at UC
# Berkeley in the Model Predictive Control (MPC) lab by Jon Gonzales
# (jon.gonzales@berkeley.edu) and Greg Marcil (grmarcil@berkeley.edu). The cloud
# services integation with ROS was developed by Kiet Lam
# (kiet.lam@berkeley.edu). The web-server app Dator was based on an open source
# project by Bruce Wootton
# ---------------------------------------------------------------------------

# README: This node serves as an outgoing messaging bus from odroid to arduino
# Subscribes: steering and motor commands on 'ecu'
# Publishes: combined ecu commands as 'ecu_pwm'

from rospy import init_node, Subscriber, Publisher, get_param
from rospy import Rate, is_shutdown, ROSInterruptException, spin, on_shutdown
from barc.msg import ECU
from numpy import pi
import rospy
import random

def rc_inputs_callback(data):
    global throttle, steering
    throttle = data.motor
    steering = data.servo
#     pass

def linspace(mn, mx, num):
    """ creates a linearly spaced list of numbers
    Params        
    ---
    mx: maximum
    mn: minimum
    num: number of elements in the list
    """
    assert mx>mn
    assert type(num) is int
    lst = []
    stp = (mx-mn)/num
    for i in range(0,num-1):
        lst.append(mn + stp*i)
    #rospy.loginfo(lst)
    
    return lst
    #
    #return [a + mn for a in range(

def main_auto():
    global throttle, steering

    # initialize the ROS node
    init_node('manual_control', anonymous=True)
    Subscriber('rc_inputs', ECU, rc_inputs_callback)
    nh = Publisher('ecu_pwm', ECU, queue_size = 10)

    # set node rate
    rateHz = 50
    dt = 1.0/rateHz
    rate = Rate(rateHz)

    throttle = 1500.0
    steering = 1500.0
    #flip = 1

    mini = 1500.0
    maxi = 1580.0
    #[a+mini for a in range(maxi - mini)]
    PWM_count = 0
    #HIGHPWM = 2000.0
    #LOWPWM = 1000.0
    #PWM_array = [1500.0, 1550.0, 1600, 1650, 1700, 1750, 1800, 1850, 1900, 1950]
    # main loop
    ramp_array = linspace(1564.0, 1650.0, 100)
    while not is_shutdown():
        
        if PWM_count < len(ramp_array):
            throttle = ramp_array[PWM_count]            
            #throttle = flip*1000.0 + throttle#PWM_array[PWM_count]
            #if throttle >= 2000:
            #    flip = -1
            #elif throttle <= 1000:
               
            PWM_count = PWM_count + 1
        else:
            throttle = 1200
        #    if throttle < 1400.0:
        #        throttle = 1400.0
        #throttle -= 4
        #if throttle < 1400.0:
        #    throttle = 1400.0
        if True: #for i in range(0,8):
          ecu_cmd = ECU(throttle, steering)
          rospy.loginfo(str(ecu_cmd.motor) + " " + str(ecu_cmd.servo))
          nh.publish(ecu_cmd)
          rate.sleep()
        #rate.sleep()

#############################################################
if __name__ == '__main__':
    try:
        main_auto()
    except ROSInterruptException:
        pass
