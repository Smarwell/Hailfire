import RPi.GPIO as GPIO
import threading

#Set Pins for Output
PIN1 = 1
PIN2 = 2
PIN3 = 3
PIN4 = 4
#Throttles (dynamic-values)
throt = [0.0, 0.0, 0.0, 0.0]
#Frequency
FREQ = 1000	#Hz
#Time Multiplier (for HIGH/LOW period duration)
#	where 1/100 is T = 1sec
TM = 1/100
def workerThread(throt, pin):
	'''
	workerThread:
		- Sets the pin to high and low, depending on current throttle,
			to designated pin.
		- Duration of HIGH, LOW can be modified through TM
	'''
	while throt[4]:
		up = throt[pin]
		GPIO.output(pin, 1)
		time.sleep(up * TM)
		GPIO.output(pin, 0)
		time.sleep((100-up)*TM)
	return

class ThrottleToPinGenerator():
	'''
	ThrottleToPinGenerator:
		- Generates an output to each pin specified (PIN1,2,3,4)
		- ONLY 1 can be instantiated
		- Set throt indices 
	'''
	def __init__(self):
		#Throttles (dynamic-values)
		#	- 5th value to tell when to stop
		#	- throttles w/i [0.0, 100.0]
		self.throt = [0.0, 0.0, 0.0, 0.0, 1]
		self.threads = []
		#Setup Pin Modes
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(PIN1, GPIO.OUT)
		GPIO.setup(PIN2, GPIO.OUT)
		GPIO.setup(PIN3, GPIO.OUT)
		GPIO.setup(PIN4, GPIO.OUT)
		#Create Threads
		for i in range(0,4):
			t = threading.Thread(target=workerThread, args=(self.throt, i,))
			threads.append(t)
			t.start()
	def exit(self);
		self.throt[4] = 0
		GPIO.cleanup();
