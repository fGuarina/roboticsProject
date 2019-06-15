#!/usr/bin/env python
# coding=utf-8

# import the libraries
import RPi.GPIO as GPIO
import time
import rospy

from std_msgs.msg import Float64

# set our GPIO pin numbering
GPIO.setmode(GPIO.BOARD)

TRIG = 29	# output pin which triggers the sensor
ECHO = 31	# input pin which reads the return signal from the sensor

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# constants
SOUND_SPEED = 34300 # cm / s
IMPULSE = 0.00001 # 10 microseconds to trigger the module

def trigOff() :
    GPIO.output(TRIG, False)
    
def trigModule() :
    GPIO.output(TRIG, True)
    time.sleep(IMPULSE)
    GPIO.output(TRIG, False)
    
def distanceCalculation(p_start, p_end) :
    p_duration = p_end - p_start
    dist = (p_duration * SOUND_SPEED) / 2
    dist = round(dist, 2)
    return dist


def main() :
	# create the node
	rospy.init_node('Right_sonar')
	
	# instantiate the publisher
	pub = rospy.Publisher('right_sonar_channel', Float64, queue_size = 1)
	
	rate = rospy.Rate(6)	# the distance is published every 0.16 seconds
	
	while not rospy.is_shutdown() :
		trigOff()
		trigModule()

		while GPIO.input(ECHO) == 0 :
			pulse_start = time.time()

		while GPIO.input(ECHO) == 1 :
			pulse_end = time.time()

		distance = distanceCalculation(pulse_start, pulse_end)
		pub.publish(distance)
		rate.sleep()
        
	GPIO.cleanup()  
	
# main function	
main()
