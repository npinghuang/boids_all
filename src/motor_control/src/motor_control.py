#!/usr/bin/env python2
#Libraries
import rospy
import time 
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
from time import sleep

in1 = 6
in2 = 5
ena = 13
in3 = 25
in4 = 16
enb = 12
speed = 0
angle = 0
stop = 0
sight = 0
speed_parameter = 3

temp1=1

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(ena,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
pwm_one=GPIO.PWM(ena,1000)
pwm_one.start(0)
GPIO.setup(in3,GPIO.OUT)
GPIO.setup(in4,GPIO.OUT)
GPIO.setup(enb,GPIO.OUT)
GPIO.output(in3,GPIO.LOW)
GPIO.output(in4,GPIO.LOW)
pwm_two=GPIO.PWM(enb,1000)
pwm_two.start(0)

def callback(data):
    global speed,angle,stop,sight
    speed = data.data[0]
    angle = data.data[1]
    stop = data.data[2]
    sight = data.data[3]
    motorcontrol()
    #rospy.loginfo('%.2f %.2f %.0f %.0f', data.data[0],data.data[1],data.data[2],data.data[3])
    rospy.loginfo('%.2f %.2f %.0f %.0f', speed,angle,stop,sight)

def listener():

    rospy.init_node('motor_control', anonymous=True)

    rospy.Subscriber('RobotData', Float32MultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def motorcontrol():
    global speed,angle,stop,sight,speed_parameter
    
    if stop == 1:
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.LOW)
        #print ("stopped")

    elif sight == 1:
        pwm_one.ChangeDutyCycle(20)
        pwm_two.ChangeDutyCycle(20)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.HIGH)
        GPIO.output(in3,GPIO.HIGH)
        GPIO.output(in4,GPIO.LOW)
    elif sight == 2:
        pwm_one.ChangeDutyCycle(20)
        pwm_two.ChangeDutyCycle(20)
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        GPIO.output(in3,GPIO.LOW)
        GPIO.output(in4,GPIO.HIGH)
    else:
        
        if -0.3<angle<0.3:
            pwm_one.ChangeDutyCycle(speed)
            pwm_two.ChangeDutyCycle(speed+1) #right
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif 0.3<=angle<1.5:
            pwm_one.ChangeDutyCycle(speed+1*speed_parameter)
            pwm_two.ChangeDutyCycle(speed-1*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif 1.5<=angle<2.5:
            pwm_one.ChangeDutyCycle(speed+2*speed_parameter)
            pwm_two.ChangeDutyCycle(speed-2*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif 2.5<=angle<3.5:
            pwm_one.ChangeDutyCycle(speed+3*speed_parameter)
            pwm_two.ChangeDutyCycle(speed-3*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
        
        elif 3.5<=angle<4.5:
            pwm_one.ChangeDutyCycle(speed+4*speed_parameter)
            pwm_two.ChangeDutyCycle(speed-4*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
        
        elif 4.5<=angle<=5:
            pwm_one.ChangeDutyCycle(speed+5*speed_parameter)
            pwm_two.ChangeDutyCycle(speed-5*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif -1.5<=angle<=-0.3:
            pwm_one.ChangeDutyCycle(speed-1*speed_parameter)
            pwm_two.ChangeDutyCycle(speed+1*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif -2.5<=angle<-1.5:
            pwm_one.ChangeDutyCycle(speed-2*speed_parameter)
            pwm_two.ChangeDutyCycle(speed+2*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)

        elif -3.5<=angle<-2.5:
            pwm_one.ChangeDutyCycle(speed-3*speed_parameter)
            pwm_two.ChangeDutyCycle(speed+3*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
        
        elif -4.5<=angle<-3.5:
            pwm_one.ChangeDutyCycle(speed-4*speed_parameter)
            pwm_two.ChangeDutyCycle(speed+4*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
        
        elif -5<=angle<-4.5:
            pwm_one.ChangeDutyCycle(speed-5*speed_parameter)
            pwm_two.ChangeDutyCycle(speed+5*speed_parameter)
            GPIO.output(in1,GPIO.LOW)
            GPIO.output(in2,GPIO.HIGH)
            GPIO.output(in3,GPIO.LOW)
            GPIO.output(in4,GPIO.HIGH)
        
if __name__ == '__main__':
    try:
        listener()
        
    except rospy.ROSInterruptException:
        pass
