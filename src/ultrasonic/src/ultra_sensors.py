#!/usr/bin/env python2
#Libraries
import rospy
from std_msgs.msg import Float32MultiArray
import RPi.GPIO as GPIO
import time
from collections import deque

q1 = deque([0,0,0,0,0])
q2 = deque([0,0,0,0,0])
q3 = deque([0,0,0,0,0])
q4 = deque([0,0,0,0,0])
q5 = deque([0,0,0,0,0])
 
#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)
 
#set GPIO Pins
trig_one = 17
echo_one = 18
trig_two = 23
echo_two = 24
trig_three = 27
echo_three = 22
trig_four = 19
echo_four = 26
trig_five = 20
echo_five = 21
 
#set GPIO direction (IN / OUT)
GPIO.setup(trig_one, GPIO.OUT)
GPIO.setup(echo_one, GPIO.IN)
GPIO.setup(trig_two, GPIO.OUT)
GPIO.setup(echo_two, GPIO.IN)
GPIO.setup(trig_three, GPIO.OUT)
GPIO.setup(echo_three, GPIO.IN)
GPIO.setup(trig_four, GPIO.OUT)
GPIO.setup(echo_four, GPIO.IN)
GPIO.setup(trig_five, GPIO.OUT)
GPIO.setup(echo_five, GPIO.IN)
 
def distance_one():

    # set Trigger to HIGH , after 0.01ms to LOW
    GPIO.output(trig_one, True)
    time.sleep(0.00001)
    GPIO.output(trig_one, False)
 
    StartTime = time.time()
    StopTime = time.time()
    t=0
    tt=0
    while GPIO.input(echo_one) == 0:
        StartTime = time.time()
        tt = StartTime - StopTime
        if tt > 0.0115309038:      
            break
 
    while GPIO.input(echo_one) == 1:
        StopTime = time.time()
        t = StopTime - StartTime
        if t > 0.0115309038:
            break

    TimeElapsed = StopTime - StartTime
    distance = (TimeElapsed * 34300) / 2
 
    if distance>0:
        q1.popleft()
        q1.append(distance)
        c = q1
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median
    else:
        c = q1
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median


def distance_two():

    GPIO.output(trig_two, True)
    time.sleep(0.00001)
    GPIO.output(trig_two, False)
 
    StartTime_two = time.time()
    StopTime_two = time.time()
    t=0
    tt=0
    while GPIO.input(echo_two) == 0:
        StartTime_two = time.time()
        tt = StartTime_two - StopTime_two
        if tt > 0.0115309038:      
            break
 
    while GPIO.input(echo_two) == 1:
        StopTime_two = time.time()
        t = StopTime_two - StartTime_two
        if t > 0.0115309038:
            break

    TimeElapsed_two = StopTime_two - StartTime_two
    distance_two = (TimeElapsed_two * 34300) / 2
 
    if distance_two>0:
        q2.popleft()
        q2.append(distance_two)
        c = q2
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median
    else:
        c = q2
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median

def distance_three():

    GPIO.output(trig_three, True)
    time.sleep(0.00001)
    GPIO.output(trig_three, False)
 
    StartTime_three = time.time()
    StopTime_three = time.time()
    t=0
    tt=0
    while GPIO.input(echo_three) == 0:
        StartTime_three = time.time()
        tt = StartTime_three - StopTime_three
        if tt > 0.0115309038:      
            break
 
    while GPIO.input(echo_three) == 1:
        StopTime_three = time.time()
        t = StopTime_three - StartTime_three
        if t > 0.0115309038:
            break

    TimeElapsed_three = StopTime_three - StartTime_three
    distance_three = (TimeElapsed_three * 34300) / 2
 
    if distance_three>0:
        q3.popleft()
        q3.append(distance_three)
        c = q3
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median
    else:
        c = q3
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median

def distance_four():

    GPIO.output(trig_four, True)
    time.sleep(0.00001)
    GPIO.output(trig_four, False)
 
    StartTime_four = time.time()
    StopTime_four = time.time()
    t=0
    tt=0
    while GPIO.input(echo_four) == 0:
        StartTime_four = time.time()
        tt = StartTime_four - StopTime_four
        if tt > 0.0115309038:      
            break
 
    while GPIO.input(echo_four) == 1:
        StopTime_four = time.time()
        t = StopTime_four - StartTime_four
        if t > 0.0115309038:
            break

    TimeElapsed_four = StopTime_four - StartTime_four
    distance_four = (TimeElapsed_four * 34300) / 2
 
    if distance_four>0:
        q4.popleft()
        q4.append(distance_four)
        c = q4
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median
    else:
        c = q4
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median

def distance_five():

    GPIO.output(trig_five, True)
    time.sleep(0.00001)
    GPIO.output(trig_five, False)
 
    StartTime_five = time.time()
    StopTime_five = time.time()
    t=0
    tt=0
    while GPIO.input(echo_five) == 0:
        StartTime_five = time.time()
        tt = StartTime_five - StopTime_five
        if tt > 0.0115309038:      
            break
 
    while GPIO.input(echo_five) == 1:
        StopTime_five = time.time()
        t = StopTime_five - StartTime_five
        if t > 0.0115309038:
            break

    TimeElapsed_five = StopTime_five - StartTime_five
    distance_five = (TimeElapsed_five * 34300) / 2
    #distance_five = 100
 
    if distance_five>0:
        q5.popleft()
        q5.append(distance_five)
        c = q5
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median
    else:
        c = q5
        c = sorted(c)
        size = len(c)
        median = c[(size-1)//2]
        return median

def ultra_sensors():
    pub = rospy.Publisher('ultra_sensors',Float32MultiArray, queue_size=1)
    rospy.init_node('ultra_sensors', anonymous=True)
    rate = rospy.Rate(10) 
    #array = [23.0, 22.3, 3.2, 55.6, 12.4]

    while not rospy.is_shutdown():
        #print("in while")
        d_one = distance_one()
        #print("after distance 1")
        d_two = distance_two()
        #print("after distance 2")
        d_three = distance_three()
        #print("after distance 3")
        d_four = distance_four()
        #print("after distance 4")
        d_five = distance_five()
        #print("after distance 5")
        array = [d_one, d_two, d_three, d_four, d_five]
        ultra_str = Float32MultiArray(data = array)
        #print("after ultra")
        rospy.loginfo(ultra_str)
        pub.publish(ultra_str)
        #print("after publish")
        rate.sleep()
    if rospy.is_shutdown() == 1:
        #print("rospy is shutdown")
        GPIO.cleanup()
 
if __name__ == '__main__':
    try:
        print("init")
        ultra_sensors()

    except rospy.ROSInterruptException:
        pass

