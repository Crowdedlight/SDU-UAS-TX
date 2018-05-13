#!/usr/bin/env python
import roslib
import sys
import rospy
from rospy.impl import init
import time
import csv
import serial
import json

import cv2


cv2.aruco


from remote_control.msg import set_controller

class remote_control:
    def __init__(self):
        # Parameters depending on camera

        # Service handler

        # parameters
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)
        self.output_range = 700

        # Subscribe to get height
        self.sub_control = rospy.Subscriber("/remote_control/set_controller", set_controller, self.callback)

    def clampPercentage(self, data):

        # clamp every channel. All but thrust have same rules of going from -100 to 100
        if data.thrust < 0: data.thrust = 0
        if data.thrust > 100: data.thrust = 100

        if data.roll < -100: data.roll = -100
        if data.roll > 100: data.roll = 100

        if data.yaw < -100: data.yaw = -100
        if data.yaw > 100: data.yaw = 100

        if data.pitch < -100: data.pitch = -100
        if data.pitch > 100: data.pitch = 100

        return data

    def Percent_to_ppm(self, percent, type=""):
        # RPY    => 3.50*procent+1500
        # Thrust => 7*procent+1150

        if type == "thrust":
            return 7.0 * percent + 1150
        else:
            return self.output_range / 100 * percent + 1500

    def callback(self, msg):
        # rospy.loginfo(msg)
        data = self.clampPercentage(msg)

        # change msg to ppm values
        # thrust = 0-100%
        # roll   = -100-100%
        # pitch  = -100-100%
        # yaw    = -100-100%
        # 100% => 1850
        # 0% => 1500
        # -100% => 1150

        thrust = self.Percent_to_ppm(data.thrust, "thrust")
        roll = self.Percent_to_ppm(data.roll)
        pitch = self.Percent_to_ppm(data.pitch)
        yaw = self.Percent_to_ppm(data.yaw)

        jsonMsg = {"t": thrust,
                   "r": roll,
                   "p": pitch,
                   "y": yaw}

        self.sendMsgToTransmitter(jsonMsg)


    def sendMsgToTransmitter(self, msg):
        # json stringify for transmit
        jsonMsg = json.dumps(msg)
        self.ser.write(jsonMsg + " \n")
        # rospy.loginfo(jsonMsg)

    def shutdownHandler(self):

        # shutdown services
        # self.s_start_log.shutdown()
        # self.s_stop_log.shutdown()
        self.ser.close()
        print("Shutting down")


def main():
    rospy.init_node('remote_control', anonymous=True)
    rospy.sleep(1)

    rc = remote_control()
    rospy.on_shutdown(rc.shutdownHandler)

    while not rospy.is_shutdown():
        line = rc.ser.readline()
        rospy.loginfo(line)

    rospy.spin()

if __name__ == '__main__':
    main()
