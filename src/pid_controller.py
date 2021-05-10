#!/usr/bin/env python3

# This code generates PID Control by reading encoder ticks from motor
# Author: Mijaz Mukundan

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

#Parameters
wheeltrack = 0.35
wheelradius = 0.035
TPR = 600


left_ticks = 0
right_ticks = 0
last_left_ticks = 0
last_right_ticks = 0

left_motor_req_rpm = 0
right_motor_req_rpm = 0

last_error = dict()
last_error['left'] = 0
last_error['right'] = 0

int_error = dict()
int_error['left'] = 0
int_error['right'] = 0

def leftTicksCallback(msg):
    global left_ticks 
    left_ticks = msg.data

def rightTicksCallback(msg):
    global right_ticks 
    right_ticks = msg.data

def cmdVelCallback(msg):
    global left_motor_req_rpm, right_motor_req_rpm
    linear_vel = msg.linear.x*0.2
    angular_vel = msg.angular.z*0.55

    if angular_vel == 0:
        left_motor_req_rpm = linear_vel*60 / (2*pi*wheelradius)
        right_motor_req_rpm = left_motor_req_rpm
    
    elif linear_vel == 0:
        left_motor_req_rpm = wheeltrack*angular_vel*60 / (2*pi*wheelradius)
        right_motor_req_rpm = -left_motor_req_rpm
    
    else:
        left_motor_req_rpm = (linear_vel-(wheeltrack/2)*angular_vel)*60/(2*pi*wheelradius)
        right_motor_req_rpm = (linear_vel+(wheeltrack/2)*angular_vel)*60/(2*pi*wheelradius)
    
    print("LRPM:{},RRPM:{}".format(left_motor_req_rpm,right_motor_req_rpm))

def calc_pid(req_val,act_val,motor_id):
    Kp = 5
    Ki = 0.0001
    Kd = 0
    global last_error, int_error
    error = req_val - act_val
    int_error[motor_id]= int_error[motor_id] + error
    diff_error = error - last_error[motor_id]

    pid_term = Kp*error + Ki*int_error[motor_id] + Kd*diff_error

    last_error[motor_id] = error

    return pid_term



rospy.init_node('pid_publisher')
pid_pub = rospy.Publisher("/pid_control", Twist, queue_size=1)
left_ticks_sub =rospy.Subscriber("/lwheel", Int16, leftTicksCallback)
right_ticks_sub =rospy.Subscriber("/rwheel", Int16, rightTicksCallback)

cmd_vel_sub = rospy.Subscriber("/cmd_vel",Twist,cmdVelCallback)

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # Change in wheel position
    delta_L = left_ticks - last_left_ticks
    delta_R = right_ticks - last_right_ticks

    dt = (current_time - last_time).to_sec()
    
    #Wheel RPM calculation
    left_motor_act_rpm  = delta_L / TPR * (60/dt)
    right_motor_act_rpm = delta_R / TPR * (60/dt)

    # print("ACT RPM:")
    # print("l:{},r:{}".format(left_motor_act_rpm,right_motor_act_rpm))
    
    #Motor PID term calculation
    left_pid = calc_pid(left_motor_req_rpm,left_motor_act_rpm,motor_id='left')
    right_pid = calc_pid(right_motor_req_rpm,right_motor_act_rpm,motor_id='right')
    
    #we will hijack Twist() message to send left and right motor pid values
    ctrl_msg = Twist() 
    ctrl_msg.linear.x = left_pid
    ctrl_msg.angular.z = right_pid
    
    #Publish the control message
    pid_pub.publish(ctrl_msg)

    
    last_left_ticks = left_ticks
    last_right_ticks = right_ticks
    last_time = current_time
    r.sleep()