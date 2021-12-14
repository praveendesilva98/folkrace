 
import RPi.GPIO as GPIO
from VL53L0X_rasp_python.python.VL53L0X import *
import keyboard
import csv

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Variables
pinServo, pinPwm, pinIna, pinInb = 12, 18, 24, 23

# Lidar
tof = VL53L0X()
tof.start_ranging(VL53L0X_BETTER_ACCURACY_MODE)

#RC Motor
GPIO.setup(pinPwm, GPIO.OUT) #PWM
GPIO.setup(pinInb, GPIO.OUT) #INB
GPIO.setup(pinIna, GPIO.OUT) #INA

# Servo
GPIO.setup(pinServo, GPIO.OUT)

# RC Motor
pwm = GPIO.PWM(pinPwm, 50)
pwm.start(0)

#Servo
servo = GPIO.PWM(pinServo, 50)
servo.start(0)


def saveCsv(lidar, servoAngle, motorDC):
    header = ['Lidar', 'Stearing Angle', 'Motor dutycycle']
    data = [lidar, servoAngle, motorDC]
    with open('dataset.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerow(data)


def setAngle(angle):
    duty = angle / 18 + 2
    GPIO.output(pinServo, True)
    servo.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(pinServo, False)
    servo.ChangeDutyCycle(0)


def clockwise():
    GPIO.output(pinInb, GPIO.HIGH)
    GPIO.output(pinIna, GPIO.LOW)


def antiClockwise():
    GPIO.output(pinIna, GPIO.HIGH)
    GPIO.output(pinInb, GPIO.LOW)


def run():
    if keyboard.is_pressed("up arrow"):
        for pwmM in range(0, 100, 10):
            pwm.ChangeDutyCycle(pwmM)
    elif keyboard.is_pressed("right arrow"):
        angleS = 50
        setAngle(angleS)
    elif keyboard.is_pressed("left arrow"):
        angleS = 130
        setAngle(angleS)
    else:
        pwmM = 0
        pwm.ChangeDutyCycle(pwmM)
        angleS = 90
        setAngle(angleS)


clockwise()


try:
    while True:
        #pwmM, angleS = 0, 90
        #run()
        distance = tof.get_distance()
        print("%d mm" % (distance))
        #saveCsv(distance, run[1], run[0])
except KeyboardInterrupt:
        pwm.stop()
        servo.stop()
        GPIO.cleanup()
        tof.stop_ranging()

