 
 
import RPi.GPIO as GPIO
from VL53L0X_rasp_python.python.VL53L0X import *
import numpy as np
import pandas as pd
from sklearn.preprocessing import PolynomialFeatures
from sklearn.linear_model import LinearRegression
from sklearn.model_selection import train_test_split 
import time


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


def predict(distance):
    dataset = pd.read_csv('dataset.csv')
    X = dataset.iloc[:, 0].values
    y = dataset.iloc[:, 1].values
    z = dataset.iloc[:, 2].values
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.4, random_state=0)
    X_train, X_test, z_train, z_test = train_test_split(X, z, test_size=0.4, random_state=0)
    poly_reg = PolynomialFeatures(degree=4)
    X_poly = poly_reg.fit_transform(X.reshape(-1, 1))
    pol_reg_angle = LinearRegression()
    pol_reg_angle.fit(X_poly, y)
    pol_reg_pwm = LinearRegression()
    pol_reg_pwm.fit(X_poly, z)
    return pol_reg_angle.predict(poly_reg.fit_transform([[distance]])), pol_reg_pwm.predict(poly_reg.fit_transform([[distance]]))


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

    

clockwise()
setAngle(90)


try:
    while True:
        st = time.time()
        gain = 1
        distance = tof.get_distance()
        prediction = predict(distance)
        servoAngle = prediction[0]
        motorPwm = prediction[1]
        pwm.ChangeDutyCycle(gain*motorPwm)
        setAngle(servoAngle)
        print("Distance: %d mm | Angle: %d° | PWM: %d | Exec time: %f" % ((distance, servoAngle, gain*motorPwm, time.time()-st)))
except KeyboardInterrupt:
        pwm.stop()
        servo.stop()
        GPIO.cleanup()
        tof.stop_ranging()


