from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import time
#Set warnings off (optional)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

camera = PiCamera()
camera.start_preview()
sleep(5)
camera.capture('/home/pi/Desktop/FYP/max2.jpg')
camera.stop_preview()
