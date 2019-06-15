#!/usr/bin/env python
# coding=utf-8

''' 
	Questo S-P si sottoscrive a 3 topic:
	'centroid_dist_topic' per leggere la distanza angolare;
	'object_dist_topic' per leggere la distanza dall'oggetto;
	'alert_channel' per fermarsi quando c'è un ostacolo nelle vicinanze.
	
	Se la distanza angolare, cioè la differenza tra la coordinata x del centroide dell'oggetto
	individuato, e la coordinata x del centro del frame è minore di una certa soglia,
	allora il robot si ritiene "centrato" ed è libero di andare avanti, 
	fermandosi davanti l'oggetto. Altrimenti effettuerà delle rotazioni per centrarsi.
	
	Implementa una macchina a stati:
	stato 0 (stato accettante): R è fermo 
	stato 1: R ha rilevato G ma deve centrarsi girandosi a destra
	stato 2: R ha rilevato G ma deve centrarsi girandosi a sinistra
	stato 3: R ha rilevato G ed è centrato
	stato 4: R ha rilevato G ed è centrato e va avanti fino a raggiungerlo 
	stato 5: R cattura la pallina
	stato 6: stato per confermare la cattura della pallina
	stato 7: stato per passare alla fase successiva della competizione
	stato 8: R rilascia la nostra pallina quando vede quella avversaria
	stato 9: R fa goal rilasciano la palla avversaria in prossimità della porta avversaria
	
	FSA:
	[stato 0] --> stato 1 --> [stato 0]	
	[stato 0] --> stato 2 --> [stato 0]	
	[stato 0] --> stato 3 --> stato 4 --> [stato 0]	(siamo centrati e andiamo avanti)
	[stato 0] --> stato 3 --> [stato 0]		(siamo centrati e ci fermiamo davanti G)	
	... 	  
'''

# import 
import RPi.GPIO as GPIO
import time
import rospy

from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import Bool

# global variables
state = -1
distance = -1			# velocity based on distance from the object	    
alert = -1			# from avoid obstacles

p = -1				# for servo motor
		    
current_phase = 1		# we start at phase 1
release = False			# to know if we have to release our ball

# pins setup 
def init():
    GPIO.setmode(GPIO.BOARD)  
    GPIO.setwarnings(False)
    
    GPIO.setup(7, GPIO.OUT)         
    GPIO.setup(11, GPIO.OUT)        
    GPIO.setup(13, GPIO.OUT)        
    GPIO.setup(15, GPIO.OUT)   
         
    GPIO.setup(12, GPIO.OUT)        # servo pin
    GPIO.setup(36, GPIO.IN)	    # central IR pin
    
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
def alert_CB(msg) :
	global alert
	if msg.data == True :
	    alert = 1
	else :
	    alert = 0

# centroid control
def centroidControl_CB(msg) :
	global state
	if state == 0 :
		if msg.data > 100 :
			state = 1
		elif msg.data < -100 :
			state = 2
		elif abs(msg.data) < 100.0 :
			state = 3					# activating the next callback

# distance control
def distanceControl_CB(msg) :
	global state
	global distance
	global release
	global current_phase

	if state == 3 :
	    
	    # if we have our ball and if we see opponent's ball, first realese our ball
	    if release == True :
		state = 8
	    
	    elif current_phase != 3 :
		if msg.data >= 33.0 : 		# distance greater than 33 centimeters
		    distance = msg.data
		    state = 4
		else :
		    state = 5 			# status "grab the object"
		    
	    elif current_phase == 3 :		
		if msg.data >= 50.0 :		# distance greater than 50 centimeters
		    distance = msg.data
		    state = 4
		else :
		    state = 9			# status "opponent's ball release"
    
def main() :
	global state
	global distance
	global alert
	
	global p
	
	global current_phase
	global release

	# create the node
	rospy.init_node('Post_Detection')
	
	# instantiate the subscribers
	sub1 = rospy.Subscriber('/centroid_dist_channel', Float64, centroidControl_CB)
	sub2 = rospy.Subscriber('/object_dist_channel', Float64, distanceControl_CB)
	subAlert = rospy.Subscriber('/alert_channel', Bool, alert_CB)
	
	# instantiate the publishers
	pub = rospy.Publisher('phase_complete_channel', Int64, queue_size = 1)
	pubAvoid = rospy.Publisher('stop_avoid_channel', Bool, queue_size = 1)
	
	init()
	stop()
	ticRelease = 0
	tocRelease = 0
	
	p = GPIO.PWM(12, 50) 		# set the servo's frequency 
	
	state = 0			# set the zero state, R is stationary
	
	while not rospy.is_shutdown() :
	    if alert == 0 :
		if release == True :
		    tocRelease = time.time()
		    if tocRelease - ticRelease >= 180 : # 3 minutes
			state = 8
	    		
		if state == 1 :
		    #stop()
		    turnRight(0.05)
		    #time.sleep(0.6)
		    state = 0
		    
		elif state == 2 :
		    #stop()
		    turnLeft(0.05)
		    #time.sleep(0.6)
		    state = 0
		    
		elif state == 4 :
		    if distance > 200 :
			#stop()
			forward(3.5)
			time.sleep(1)
			
		    elif distance > 150 :
			#stop()
			forward(2.5)
			time.sleep(1)
			
		    elif distance > 100 :
			#stop()
			forward(1.5)
			time.sleep(1)
			
		    elif distance <= 100 :
			#stop()
			forward (0.5)
			time.sleep(1)
		    
		    state = 0
			
		elif state == 5 :
		    pubAvoid.publish(True)	# stop avoiding obstacles
		    time.sleep(1.5)	
		    
		    p.start(2) 			# this is the neutral position for us (0 degree) for servo (starting closed)
		    		    
		    # the ball is close
		    p.ChangeDutyCycle(7.5)  	# turn towards 105 degree (opened)
		    time.sleep(0.8)		# very important sleep
		    
		    infinityForward()		# let's go get it
		    
		    booleanValue = GPIO.input(36)
		    tic = time.time()
		    toc = tic
		    
		    while booleanValue == True and (toc - tic) <= 4: # 3 seconds
			booleanValue = GPIO.input(36)
			toc = time.time()
		    
		    # the ball is inside (or maybe not) 
		    p.ChangeDutyCycle(2)  	# turn towards 0 degree (closed)
		    time.sleep(0.8)		# very important sleep
		    
		    stop()
		    
		    p.ChangeDutyCycle(0)  	# to stop servo oscillations
		    time.sleep(0.5)		# very important sleep
		    
		    time.sleep(1.5)
		    pubAvoid.publish(False)	# restarting avoiding obstacles
		    
		    state = 6 			# going to the confirmation status
		    
		elif state == 6 :
		    tic = time.time()
		    toc = tic
		    
		    while (toc - tic) <= 3: 	# 3 seconds
			booleanValue = GPIO.input(36)
			if booleanValue == False :
			    state = 7		# it's ok. Vamoosss!
			    break
			else :
			    state = 0		# d'oh!
			toc = time.time()	# 3 seconds
		
		elif state == 7 :				
		    if current_phase == 1 :
			pub.publish(current_phase) 		# phase 1 completed
			print("phase 1 completed")
			time.sleep(3)
			release = True				# we have our ball now
			ticRelease = time.time()
			tocRelease = ticRelease
			current_phase = current_phase + 1	# next phase (2)
			state = 0
			
		    elif current_phase == 2 :
			pub.publish(current_phase) 		# phase 2 completed
			print("phase 2 completed")
			time.sleep(3)
			current_phase = current_phase + 1	# next phase (3)
			state = 0
			
		    elif current_phase == 3 :
			pub.publish(current_phase) 		# phase 3 completed, release made
			print("phase 3 completed")
			time.sleep(3)
			current_phase = current_phase + 1	# next phase (4)
			state = 0
			
		    elif current_phase == 4 :
			pub.publish(current_phase) 		# phase 4 completed
			print("phase 4 completed")
			time.sleep(3)
			current_phase = current_phase - 1	# next phase (3)
			state = 0
		
		# in this state we leave our ball
		elif state == 8 :
		    pubAvoid.publish(True)		# stop avoiding obstacles
		    time.sleep(1.5)
		    
		    turnRight(0.3)
		    
		    p.start(2) 	
		    		
		    p.ChangeDutyCycle(7.5)  	
		    time.sleep(0.8)
		    
		    reverse(0.5)
		    
		    p.ChangeDutyCycle(2)  	
		    time.sleep(0.8)	
		    
		    p.ChangeDutyCycle(0)  	
		    time.sleep(0.6)
		    
		    forward(0.5)
		    
		    turnLeft(0.3)
		    
		    print("Our ball has been released!")
		    release = False
		    
		    time.sleep(3)
		    pubAvoid.publish(False)		# restarting avoiding obstacles
		    
		    state = 0
		    
		# in this state we release the opponent's ball near the opponent's door
		elif state == 9 :
		    pubAvoid.publish(True)		# stop avoiding obstacles
		    time.sleep(1.5)
		    
		    forward(0.5)
		    
		    p.start(2) 	
		    		
		    p.ChangeDutyCycle(7.5)  	
		    time.sleep(0.8)
		    
		    reverse(1)
		    
		    p.ChangeDutyCycle(2)  	
		    time.sleep(0.7)	
		    	
		    p.ChangeDutyCycle(0)  	
		    time.sleep(0.5)
		   
		    print("Opponent's ball has been released!")
		    
		    time.sleep(1.5)
		    pubAvoid.publish(False)		# restarting avoiding obstacles
		    
		    time.sleep(5)			# time for repositioning the ball
		    
		    state = 7
		
	stop()
	GPIO.cleanup()  	

# main function	
main()
