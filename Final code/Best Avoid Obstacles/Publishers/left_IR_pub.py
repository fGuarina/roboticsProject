#!/usr/bin/env python
# coding=utf-8

# import the libraries
import RPi.GPIO as GPIO
import time
import rospy

from std_msgs.msg import Bool

GPIO.setmode(GPIO.BOARD)
LEFT_IR_PIN = 8 
GPIO.setup(LEFT_IR_PIN, GPIO.IN)

def main() :
	# create the node
	rospy.init_node('Left_IR')	
	
	# instantiate the publisher
	pub = rospy.Publisher('left_IR_channel', Bool, queue_size = 1)
	
	rate = rospy.Rate(6)	
	
	while not rospy.is_shutdown() :
		booleanValue = GPIO.input(LEFT_IR_PIN)
		
		pub.publish(booleanValue)
		rate.sleep()
		
	GPIO.cleanup()

# main function
main()
