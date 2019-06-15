# coding=utf-8

'''
	Simple code for IR sensor.
	Uscita:
	- livello logico alto se non c'è nessun rilevamento
	- livello logico basso se c'è un rilevamento
	La distanza può essere regolata tramite il trimmer, 
	ruotando in senso orario la distanza aumenta; 
	ruotando in senso antiorario la distanza di rilevamento diminuisce.
	
	Da esperimenti si è osservato che la distanza massima rilevata è:
	- 1.5 / 2 cm da un oggetto nero
	- 6 / 6.5 cm da un oggetto bianco
''' 

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
IR_PIN = 37 
GPIO.setup(IR_PIN, GPIO.IN)

def main() :	
	while True :
		booleanValue = GPIO.input(IR_PIN)
		if booleanValue :
			print("Nessun rilevamento!")
		else :
			print("Qualcosa è stato rilevato!")
		time.sleep(0.5)
	GPIO.cleanup()

# main function
main()
  
  
