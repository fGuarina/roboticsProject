#!/usr/bin/env python
# coding=utf-8

# import 
import RPi.GPIO as GPIO
import time
import rospy

import random
import decimal

from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64

# global variables
state = -1			    
alert = -1
choiceCounter = 0		# Dreambot can go forward or can turn on itself 

# for compass
compassCounter = 0	
target_angle = -1
angle = -1
direction = -1

# for sampling angle
sampleNow = False
samplingTime = 0
block = False

phase_control = -1

# pins setup 
def init():
    GPIO.setmode(GPIO.BOARD)  
    GPIO.setwarnings(False)
          
    GPIO.setup(7, GPIO.OUT)         
    GPIO.setup(11, GPIO.OUT)        
    GPIO.setup(13, GPIO.OUT)        
    GPIO.setup(15, GPIO.OUT)        
    
def stop():                         
    GPIO.output(7, False)
    GPIO.output(11, False)
    GPIO.output(13, False)
    GPIO.output(15, False)

def forward(t):
    GPIO.output(7, True)
    GPIO.output(15, True)  
    time.sleep(t)                   
    stop()               
 
def infinityForward():
    GPIO.output(7, True)
    GPIO.output(15, True)  
 
def reverse(t):                     
    GPIO.output(11, True)
    GPIO.output(13, True)
    time.sleep(t)
    stop()

def turnRight(t):                   
    GPIO.output(7, True)            
    GPIO.output(13, True)           
    time.sleep(t)
    stop()
    
def turnLeft(t):                  
    GPIO.output(11, True)           
    GPIO.output(15, True)           
    time.sleep(t)
    stop() 

# callback functions

# from Best Avoid Obstacles
def alertFunction_CB(msg) :
    global alert
    if msg.data == True :
	alert = 1
    else :
	alert = 0
	
# to check if we must search the object or not
def objFunction_CB(msg) :
    global state
    global block
    if state == 0 :
	if msg.data == False and block == False :	
	    state = 2					# object not found, search!
	elif msg.data == True and block == False:
	    state = 3					# object found, stop the search!

# we must sampling (redirection) at the end of these phases 	    
def phaseComplete_CB(msg) :
    global sampleNow
    if msg.data == 2 or msg.data == 3 :
	sampleNow = True
    if msg.data == 4 :
	sampleNow = True

# to redirect we must know what phase we are
def controlFunction_CB(msg) :
    global phase_control
    phase_control = msg.data

# callback for redirecting Dreambot
def compass_CB(msg) :
    global state
    
    global compassCounter
    global target_angle
    global angle
    
    global sampleNow
    global samplingTime
    global phase_control
    
    global block
    
    if compassCounter <= 1 :
	compassCounter = compassCounter + 1
    
    # target angle: points to the opponent's door (taken only once)
    if compassCounter == 1 :
	target_angle = msg.data
    else :
	angle = msg.data

    if state == 0:
	if phase_control == 3 :
	    if sampleNow == True or samplingTime > 350: 		# 25 - 30 seconds
		block = True
		state = 4
	elif phase_control == 4 :
	    if sampleNow == True :
		block = True
		state = 5
    samplingTime = samplingTime + 1
    
def main():
    global state
    global alert
    global choiceCounter
    
    global compassCounter
    global target_angle
    global angle
    global direction
    
    global sampleNow
    global samplingTime
    global block
    
    global phase_control
    
    # create the node
    rospy.init_node('Search')
    
    # subscribers
    sub = rospy.Subscriber('/alert_channel', Bool, alertFunction_CB)
    sub = rospy.Subscriber('/object_found_channel', Bool, objFunction_CB)
    sub = rospy.Subscriber('/phase_control_channel', Int64, controlFunction_CB)
    
    sub = rospy.Subscriber('/phase_complete_channel', Int64, phaseComplete_CB)
    sub = rospy.Subscriber('/compass_channel', Float64, compass_CB)
    	
    init()
    stop()
    
    # taking the target angle 
    time.sleep(2)
    
    # 90 degree turn to see our ball
    turnRight(0.5)
    
    # detecting the object (pray for us)
    time.sleep(4)		
    
    state = 0			# set the zero state, R is stationary
    
    count = 0
    n = False
    
    while not rospy.is_shutdown() :
	if alert == 0 :
	    
	    if state == 2 :
		infinityForward()	# go Dreambot, go
		
		if choiceCounter > 4 :
		    
		    if n == False :
			# Dreambot will turns right or left?
			rand = float(decimal.Decimal(random.randrange(0, 1000))/1000) # between 0.0 and 1.0
			
			# how many rotations? 
			randRot = random.randint(10, 15) 		# (too many!)
			n = True
			
		    if rand <= 0.5 :		# turn right
			if count == randRot:
			    count = 0
			    choiceCounter = 0
			    n = False
			    state = 0
			    
			else :
			    stop()
			    turnRight(0.15)
			    time.sleep(1.5)
			    count = count + 1
			    state = 0
		    else :			# turn left
			if count ==  randRot:
			    count = 0
			    choiceCounter = 0
			    n = False
			    state = 0
			    
			else :
			    stop()
			    turnLeft(0.15)
			    time.sleep(1.5)
			    count = count + 1
			    state = 0
		else :
		    choiceCounter = choiceCounter + 1
		state = 0
		    
	    elif state == 3 :
		# do nothing
		state = 0
	    
	    # redirecting Dreambot (with sampling)
	    elif state == 4 :
		print("STATE 4")
		print(samplingTime)
		if samplingTime > 350 :
		    samplingTime = 0
		    
		sampleNow = False
		block = False
		
		direction = (angle - target_angle) % 360
		
		while direction >= 5 and direction <= 355 :
		    direction = (angle - target_angle) % 360
		    if direction >= 5 and direction <= 180 :
			turnRight(0.2 * direction / 90)
		    elif direction >= 181 and direction <= 355 :
			turnLeft(0.2 * (direction-180)/90)
		
		choiceCounter = 0
		time.sleep(3)
		state = 0
	    
	    # redirecting Dreambot (only once per phase)
	    elif state == 5 :
		print("STATE 5")
		    
		sampleNow = False
		block = False
		
		target_angle = (target_angle + 180) % 360
		direction = (angle - target_angle) % 360
		
		while direction >= 5 and direction <= 355 :
		    direction = (angle - target_angle) % 360
		    if direction >= 5 and direction <= 180 :
			turnRight(0.2 * direction / 90)
		    elif direction >= 181 and direction <= 355 :
			turnLeft(0.2 * (direction-180)/90)
		    
		target_angle = target_angle - 180
		
		choiceCounter = 0
		time.sleep(3)
		state = 0
	    		
    stop()

# main function	
main()
