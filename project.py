 
import RPi.GPIO as GPIO
import time
import smbus
from VL53L0X_rasp_python.python.VL53L0X import *
import keyboard
import csv

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Variables
pinServo, pinPwm, pinIna, pinInb = 12, 18, 24, 23

# IMU
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


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



def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

    #Write to power management register
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

    #Write to Configuration register
    bus.write_byte_data(Device_Address, CONFIG, 0)

    #Write to Gyro configuration register
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

    #Write to interrupt enable register
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
    

def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)

    #to get signed value from mpu6050
    if(value > 32768):
            value = value - 65536
    return value

def saveCsv(lidarL, lidarR, servoAngle, motorDC):
    header = ['Left Lidar', 'Rght Lidar', 'Stearing Angle', 'Motor dutycycle']
    data = [lidarL, lidarR, servoAngle, motorDC]
    
    with open('datasheet.csv', 'w', encoding='UTF8') as f:
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


def controlMotor():
    if keyboard.is_pressed("u"):
        for x in range(0, 100, 10):
            pwm.ChangeDutyCycle(x)
    else:
        x = 0
        pwm.ChangeDutyCycle(x)
    return x


def controlServo():
    if keyboard.is_pressed("h"):
        angle = 50
        setAngle(angle)
    elif keyboard.is_pressed("k"):
        angle = 140
        setAngle(angle)
    else:
        angle = 90
        setAngle(angle)
    return angle
        

clockwise()
bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()


try:
    while True:
        #RC Motor DutyCycle
        motorSpeed = controlMotor()
        
        #IMU val
        #Read Accelerometer raw value
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)

        #Read Gyroscope raw value
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        #Full scale range +/- 250 degree/C as per sensitivity scale factor
        Ax = acc_x/16384.0
        Ay = acc_y/16384.0
        Az = acc_z/16384.0

        Gx = gyro_x/131.0
        Gy = gyro_y/131.0
        Gz = gyro_z/131.0


        #print ("Gx=%.2f" %Gx,"Gy=%.2f" %Gy,"Gz=%.2f" %Gz,"Ax=%.2f g" %Ax,"Ay=%.2f g" %Ay,"Az=%.2f g" %Az)
        
        #Lidar val
        distance = tof.get_distance()
        #print("%d mm" % (distance))

        # Servo
        angle = controlServo()
        saveCsv(distance1, distance2, angle, motorSpeed)


except KeyboardInterrupt:
        pwm.stop()
        servo.stop()
        GPIO.cleanup()
        tof.stop_ranging()

    
