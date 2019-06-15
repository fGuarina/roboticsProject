# import the libraries
import RPi.GPIO as GPIO
import time

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

try:
    while True :
        time.sleep(2)

        trigOff()
        trigModule()

        while GPIO.input(ECHO) == 0 :
            pulse_start = time.time()

        while GPIO.input(ECHO) == 1 :
            pulse_end = time.time()

        distance = distanceCalculation(pulse_start, pulse_end)
        print ("Distanza dall'ostacolo: ", distance, " cm")

        if (distance < 5):
            print ("Distanza minore di 5c m!")
            break
        
except KeyboardInterrupt:
    print("Misurazione interrotta dall'utente")  

GPIO.cleanup()  



        
    
    
    
    

