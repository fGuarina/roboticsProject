#!/usr/bin/env python
# coding=utf-8
 
'''
    Questo S-P si sottoscrive a 5 topic: 
    1) 'left_IR_channel', legge un booleano (0) se c'è un ostacolo a sinistra di R
    2) 'right_IR_channel', legge un booleano (0) se c'è un ostacolo a destra di R
    3) 'left_sonar_channel', legge la distanza (cm) da ostacoli frontali a R provenienti da sx
    4) 'right_sonar_channel' legge la distanza (cm) da ostacoli frontali a R provenienti da dx
    5) 'stop_avoid_channel' topic per disattivare l'evita ostacoli durante la presa della palla avversaria
'''

# import 
import RPi.GPIO as GPIO
import time
import rospy

import random
import decimal

from std_msgs.msg import Float64
from std_msgs.msg import Bool

# global variables
state = -1   
stopAvoiding = 0     

# pins setup 
def init():
    GPIO.setmode(GPIO.BOARD)        
    GPIO.setup(7, GPIO.OUT)         # left motor: forward
    GPIO.setup(11, GPIO.OUT)        # left motor: reverse
    GPIO.setup(13, GPIO.OUT)        # right motor: reverse
    GPIO.setup(15, GPIO.OUT)        # right motor: forward
    
def stop():                         
    GPIO.output(7, False)
    GPIO.output(11, False)
    GPIO.output(13, False)
    GPIO.output(15, False)

def forward():
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

# callback function for left IR
def changeState1_CB(msg) :
    global state
    if state == 0 :
        if msg.data == 0 :
            state = 1
	    
# callback function for right IR
def changeState2_CB(msg) :
    global state
    if state == 0 :
        if msg.data == 0 :
            state = 2

# callback function for left sonar
def changeState3_CB(msg) :
    global state
    if state == 0 :
        if msg.data < 24.0 :
            state = 3
  
# callback function for right sonar
def changeState4_CB(msg) :
    global state
    if state == 0 :
        if msg.data < 24.0 :
            state = 4
	    
# callback function for stopping or restarting avoiding obstacles
def changeState5_CB(msg) :
    global stopAvoiding
    if msg.data == True :
	stopAvoiding = 1
    elif msg.data == False :
	stopAvoiding = 0
    	    

def main() :
    global state
    global stopAvoiding
    
    # create the node
    rospy.init_node('Avoid_Obstacles')
    
    # subscribers for distances sensor
    sub_1 = rospy.Subscriber('/left_IR_channel', Bool, changeState1_CB)
    sub_2 = rospy.Subscriber('/right_IR_channel', Bool, changeState2_CB)
    
    sub_3 = rospy.Subscriber('/left_sonar_channel', Float64, changeState3_CB)
    sub_4 = rospy.Subscriber('/right_sonar_channel', Float64, changeState4_CB)
    
    # if we are taking the ball, stop avoiding obstacles
    sub_5 = rospy.Subscriber('/stop_avoid_channel', Bool, changeState5_CB)
    
    # instantiate the publisher
    pub = rospy.Publisher('alert_channel', Bool, queue_size = 1)
    
    init()
    
    state = 0           # set the zero state, R is stationary and there are no obastacles
    
    rate = rospy.Rate(6)
        
    while not rospy.is_shutdown() :
	
	if stopAvoiding == 0 :
	    if state == 0 :
		pub.publish(False)
	
	    elif state == 1 : 
		pub.publish(True)
		stop()
		#time.sleep(0.5)
		randIRL = float(decimal.Decimal(random.randrange(100, 150))/1000) # between 0.100 and 0.150
		reverse(randIRL)    
		turnRight(randIRL)
		stop()
		state = 0
            
	    elif state == 2 :
		pub.publish(True)
		stop()
		#time.sleep(0.5)
		randIRR = float(decimal.Decimal(random.randrange(100, 150))/1000) 
		reverse(randIRR)    
		turnLeft(randIRR)
		stop()
		state = 0
    
	    elif state == 3 :
		pub.publish(True)
		stop()
		#time.sleep(0.5)
		randSonarL = float(decimal.Decimal(random.randrange(200, 300))/1000) # between 0.200 and 0.300
		reverse(randSonarL)  
		turnRight(randSonarL)
		stop()
		state = 0
	    
	    elif state == 4 :
		pub.publish(True)
		stop()
		#time.sleep(0.5)
		randSonarR = float(decimal.Decimal(random.randrange(100, 150))/1000) # between 0.100 and 0.150
		reverse(randSonarR)  
		turnLeft(randSonarR)
		stop()
		state = 0
	    
	rate.sleep()
	
    stop()
    GPIO.cleanup()      

# main function 
main()
