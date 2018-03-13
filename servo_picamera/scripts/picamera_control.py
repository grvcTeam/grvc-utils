#!/usr/bin/env python
# license removed for brevity

# GRVC Servo Controller using Raspberry Pi
# Depedencies
# $ sudo apt-get install python-setuptools
# $ sudo easy_install -U RPIO
# Source https://www.nociones.de/controlar-un-servo-con-rasperry-pi-usando-rpio-pwm-y-dma/

import rospy, tf
import RPIO import PWM
from mavros_msgs.msg import RCIn

global servo

def callback(data):
    newDC = data.channels[8]
    ##
    # Process new DC to be set between 2500 (-90º) and 600(90º)
    ##
    servo.set_servo(12,newDC)  # GPIO12


def talker():
    global servo
    servo = PWM.Servo()
    servo.set_servo(12,1500) # 0º

    rospy.init_node('picamera_control', anonymous=True)
    rospy.Subscriber("mavros/rc/in", RCIn, callback)

    rate = rospy.Rate(1) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
