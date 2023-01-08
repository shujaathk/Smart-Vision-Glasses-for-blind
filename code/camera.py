from picamera import PiCamera
import RPi.GPIO as GPIO
from time import sleep
import time
#Set warnings off (optional)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Set Button and LED pins
Button = 16
buzzer = 23
GPIO_TRIGGER = 18
GPIO_ECHO = 24

#Setup Button and LED
GPIO.setup(Button,GPIO.IN,pull_up_down=GPIO.PUD_UP)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
GPIO.setup(buzzer, GPIO.OUT)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)
    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    StartTime = time.time()
    StopTime = time.time()
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2
    return distance

if __name__ == '__main__':
    try:
        while True:
            button_state = GPIO.input(Button)
            dist = distance()
            if dist < 20:
                GPIO.output(buzzer, GPIO.HIGH)
                sleep(0.5)
                print ("Measured Distance = %.1f cm" % dist)
                time.sleep(1)
            else:
                print ("Measured Distance = %.1f cm" % dist)
                GPIO.output(buzzer, GPIO.LOW)
                sleep(0.5)
           
            if button_state == GPIO.LOW:
                print("BUTTON IS PRESSED")
                camera = PiCamera()
                camera.start_preview()
                sleep(5)
                camera.capture('/home/aswah/Desktop/FYP/he3.jpg')
                camera.stop_preview()
                 
            else:
                print("BUTTON IS NOT PRESSED so no image")
                sleep(1)
           
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()