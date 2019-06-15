#!/usr/bin/env python
# coding=utf-8

'''
	Our game phases:
	Phase 1: we first take our ball;
	Phase 2: when we see the opponent's ball, we first release our ball and then take the opponent's ball;
	
	Phase 3: release the opponent's ball (doing goal);
	Phase 4: let's get the opponent's ball repositioned.
'''

# import 
import RPi.GPIO as GPIO
import rospy

from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool

# global variables
phase_control = -1

# callback function	
def phase_CB(msg) :
	global phase_control
	
	# phase 1 complete, going to phase 2
	if msg.data == 1 :
		phase_control = 2
	
	# phase 2 complete, going to phase 3
	elif msg.data == 2 :
		phase_control = 3
	
	# phase 3 complete, going to phase 4
	elif msg.data == 3 :
		phase_control = 4
	
	# phase 4 complete, going againg to phase 3	(ping-pong)
	elif msg.data == 4 :
		phase_control = 3
	

def main() :
	global phase_control
	
	# phase 1: first we must take our ball
	phase_control = 1
	
	# create the node
	rospy.init_node('Main')
	
	# subscriber
	sub = rospy.Subscriber('/phase_complete_channel', Int64, phase_CB)
	
	# publisher
	pubPhaseControl = rospy.Publisher('phase_control_channel', Int64, queue_size = 1)	
	
	rate = rospy.Rate(10)	# warning! do not decrease the frequency!
	
	while not rospy.is_shutdown() :		
		pubPhaseControl.publish(phase_control)
				
		rate.sleep()

# main function
main()
