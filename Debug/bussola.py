import py_qmc5883l
import RPi.GPIO as GPIO
import time

def init():
    GPIO.setmode(GPIO.BOARD)        
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

init()
sensor = py_qmc5883l.QMC5883L()
#m = sensor.get_magnet()
#print(m)
sensor.declination=2.9
porta_avversaria = (sensor.get_bearing()+180)%360
print("heading angle: %d" %porta_avversaria)
time.sleep(5)
actual_position=sensor.get_bearing()
direction=(actual_position -porta_avversaria)%360
while direction >= 5 and direction <= 355 :
	actual_position=sensor.get_bearing()
	direction=(actual_position -porta_avversaria)%360
	print(direction)
	if direction >= 5 and direction <= 180 :
	    turnRight(0.2 * direction / 90)
	elif direction >= 181 and direction <= 355 :
	    turnLeft(0.2 * (direction-180)/90)
	#time.sleep(1)
