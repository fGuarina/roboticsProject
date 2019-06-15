# import 
import RPi.GPIO as GPIO
import time

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

def forward(t):
    GPIO.output(7, True)
    GPIO.output(15, True)
    time.sleep(t)                   # t second delay of the next instruction
    stop()                  
 
def reverse(t):                     
    GPIO.output(11, True)
    GPIO.output(13, True)
    time.sleep(t)
    stop()

def turnRight(t):                   # turn on itself to the right
    GPIO.output(7, True)            # left motor: forward 
    GPIO.output(13, True)           # right motor: reverse
    time.sleep(t)
    stop()
    
def turnLeft(t):                    # turn on itself to the left
    GPIO.output(11, True)           # left motor: reverse 
    GPIO.output(15, True)           # right motor: forward
    time.sleep(t)
    stop()

# square path
def square():
    init()

    print("Avanti")
    forward(1.5)
    time.sleep(1)

    print("Gira su se stesso verso destra")
    turnLeft(0.5)          # time for a 90-degree angle
    time.sleep(1)

    print("Avanti")
    forward(1.5)
    time.sleep(1)

    print("Gira su se stesso verso destra")
    turnLeft(0.5)          # time for a 90-degree angle
    time.sleep(1)

    print("Avanti")
    forward(1.5)
    time.sleep(1)

    print("Gira su se stesso verso destra")
    turnLeft(0.5)          # time for a 90-degree angle
    time.sleep(1)

    print("Avanti")
    forward(1.5)
    time.sleep(1)

    print("Percorso terminato")
    turnLeft(0.5)          # time for a 90-degree angle
    
    GPIO.cleanup()                      

# main                              
#square()

init()
turnRight(0.5);
GPIO.cleanup() 







    
