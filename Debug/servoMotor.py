#!/usr/bin/env python
# coding=utf-8

'''
	The following program will control the servo making it move 
	to its neutral position (for us 0 degrees), 
	wait 1 second and then move to its 110 degrees, 
	The cycle continues until interrupted
	
	dc = leght / period, so
	for 0.5 impulse we have dc = (0.5 / 20) * 100 = 2.5 clockwise rotation
	for 1.5 impulse we have dc = (1.5 / 20) * 100 = 7.5 (90° position)
	for 2.5 impulse we have dc = (2.5 / 20) * 100 = 12.5 
	
	So, the neutral position is 0° -> p.start(2)
	and clockwise rotation to 0° ->  p.ChangeDutyCycle(3.5)  
	
	NOTE: 
	If the servo motor shakes a bit while it is not moving, 
	you can pause the pulse with ChangeDutyCycle(0)
	
'''

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)

GPIO.setup(12, GPIO.OUT)

p = GPIO.PWM(12, 50) 	# That frequency was selected because the servo motor expect a pulse  	 
			# every 20ms (period), that means 50 pulses per second or Hertz
					 
p.start(2)	 	# Once instantiated the PWM module, to start sending a pulse we do
			# This is the neutral position (0° degree)

try:
    while True:
        p.ChangeDutyCycle(5.5)  # turn towards 100 degree
        time.sleep(0.8) # sleep 1 second
        
        p.ChangeDutyCycle(2)  # turn towards 0 degree
        time.sleep(0.8) # sleep 1 second
	
	p.ChangeDutyCycle(0)
	time.sleep(0.5) # sleep 1 second
	
	
except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
		 		
				

