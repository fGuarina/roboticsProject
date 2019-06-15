#!/usr/bin/env python
# coding=utf-8

# import 
import RPi.GPIO as GPIO
import time
import rospy

# class to manage the compass
import py_qmc5883l

from std_msgs.msg import Float64

GPIO.setmode(GPIO.BOARD)

# setting the compass
sensor = py_qmc5883l.QMC5883L(output_range = py_qmc5883l.RNG_8G)
m = sensor.get_magnet()
sensor.declination = 2.9

def main() :
    # create the node
    rospy.init_node('Compass')
    
    # publisher
    pub = rospy.Publisher('compass_channel', Float64, queue_size = 1)
    
    # target angle calculation
    angle = sensor.get_bearing()
    
    pub.publish(angle)
    
    rate = rospy.Rate(6)
    
    while not rospy.is_shutdown() :
        # angle calculation
        angle = sensor.get_bearing()
        pub.publish(angle)
        
        rate.sleep()
    
    

# main function
main()
