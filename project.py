 
import RPi.GPIO as GPIO
from VL53L0X_rasp_python.python.VL53L0X import *
import keyboard
import csv

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Variables
pinServo, pinPwm, pinIna, pinInb = 12, 18, 24, 23
rc = 0 #Motor PWM
st = 0 #Steering rotating angle

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


def motorSpeed():
    if keyboard.read_key() == "u":
        global rc
        rc += 5
    elif keyboard.read_key() == "b":
        rc += -5
    
    if rc>100:
        rc=100
    elif rc<0:
        rc=0
    pwm.ChangeDutyCycle(rc)
    return rc


def steerAngle():
    if keyboard.read_key() == "k":
        global st
        st += 10
    elif keyboard.read_key() == "h":
        st += -10
    else:
        st = 90
    
    if st>130:
        st=130
    elif st<50:
        st=50
    setAngle(st)
    return st


def writeHeaderCsv():
    with open('dataset.csv', 'w', encoding='UTF8') as csv_file:
            csv_writer = csv.writer(csv_file)
            header = ['Lidar', 'Stearing Angle', 'Motor dutycycle']
            csv_writer.writerow(header)


def readCsv():
    try:
        csv_file = open('dataset.csv')
        csv_reader = csv.reader(csv_file)
        count = len(list(csv_reader))
        return count
    except IOError:
        print("File doen't exist!")
        writeHeaderCsv()


def writeCsv(lidar, servoAngle, motorDC):
    count = readCsv()
    data = [lidar, servoAngle, motorDC]
    with open('dataset.csv', 'a', encoding='UTF8') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(data)
    csv_file.close()


clockwise()


try:
    while True:
        motorSpeed()
        steerAngle()
        distance = tof.get_distance()
        writeCsv(distance, st, rc)
except KeyboardInterrupt:
        pwm.stop()
        servo.stop()
        GPIO.cleanup()
        tof.stop_ranging()

