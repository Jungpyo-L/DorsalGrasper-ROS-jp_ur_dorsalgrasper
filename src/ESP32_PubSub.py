#!/usr/bin/env python

## JP's ESP32 experimentation in ROS environment

import rospy
import numpy as np
import os, sys
from jp_ur_dorsalgrasper.msg import SensorPacket

# serial communication
import serial
from serial import Serial


def main():
    rospy.init_node('ESP32')
    
    #Sensor reading is published to topic 'SensorPacket'
    pub = rospy.Publisher('SensorPacket', SensorPacket, queue_size=10)
    msg = SensorPacket()
    msg.data = [0.0, 0.0, 0.0, 0.0] 

    ser = serial.Serial("/dev/ttyUSB0", baudrate=115200, timeout=1, write_timeout=1)
    ser.flushInput()
 
    while not rospy.is_shutdown():
        ser_bytes = ser.readline().decode("utf-8")
        split_data = ser_bytes.split(',')
        first_val = split_data[0]
        second_val = split_data[1]
        # print('first: ', first_val)
        # print('second: ', second_val)
        msg.data[0] = float(first_val)
        msg.data[1] = float(second_val)
        msg.data[2] = 0.0
        msg.data[3] = 0.0
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
