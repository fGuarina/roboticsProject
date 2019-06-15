# import
import RPi.GPIO as GPIO
 
# set mode BOARD and disable warnings
GPIO.setmode(GPIO.BOARD)
#GPIO.setwarnings(False)
 
'''
    Pins 7 and 11 -> left motor
    Pins 13 and 15 -> right motor
    
    Pins 16 and 18 -> left sonar
    Pins 29 and 31 -> right sonar
    
    Pin 8 -> left IR 
    Pin 37 -> right IR
    
    Pin 36 -> central IR
    
    Pin 12 -> servo
    
    Pin 3 and 5 -> compass (SDA and SCL)
'''
 
# set used pins
usedPins = [7, 11, 13, 15, 16, 18, 29, 31, 8, 37, 36, 12]
 
# reset used PINs
for pin in usedPins :
    GPIO.setup(pin, GPIO.OUT)       # set as output
    GPIO.output(pin, False)         # deactivate
 
# clean and exit
GPIO.cleanup()
