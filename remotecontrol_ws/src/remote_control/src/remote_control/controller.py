#!/usr/bin/env python
import roslib
import sys
import rospy
from rospy.impl import init
import time
import csv
import serial
import json

from remote_control.msg import set_controller

class remote_control:
    def __init__(self):
        # Parameters depending on camera

        # Service handler

        # parameters
        self.ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200)

        # Subscribe to get height
        self.sub_control = rospy.Subscriber("/remote_control/set_controller", set_controller, self.callback)

    def clampProcentage(self, data):

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

    def Procent_to_ppm(self, procent, type=""):
        # RPY    => 3.50*procent+1500
        # Thrust => 7*procent+1150

        if type == "thrust":
            return 7.0*procent+1150
        else:
            return 3.50*procent+1500

    def callback(self, msg):
        # rospy.loginfo(msg)
        data = self.clampProcentage(msg)

        # change msg to ppm values
        # thrust = 0-100%
        # roll   = -100-100%
        # pitch  = -100-100%
        # yaw    = -100-100%
        # 100% => 1850
        # 0% => 1500
        # -100% => 1150

        thrust = self.Procent_to_ppm(data.thrust, "thrust")
        roll = self.Procent_to_ppm(data.roll)
        pitch = self.Procent_to_ppm(data.pitch)
        yaw = self.Procent_to_ppm(data.yaw)

        jsonMsg = {"thrust": thrust,
                   "roll": roll,
                   "pitch": pitch,
                   "yaw": yaw}

        self.sendMsgToTransmitter(jsonMsg)


    def sendMsgToTransmitter(self, msg):
        # json stringify for transmit
        jsonMsg = json.dumps(msg)
        self.ser.write(jsonMsg + "\n")
        rospy.loginfo(jsonMsg)

    def shutdownHandler(self):

        # shutdown services
        # self.s_start_log.shutdown()
        # self.s_stop_log.shutdown()
        self.ser.close()
        print("Shutting down")


def main():
    rospy.init_node('droneinfo', anonymous=True)
    rospy.sleep(1)

    rc = remote_control()
    rospy.on_shutdown(rc.shutdownHandler)

    rospy.spin()

if __name__ == '__main__':
    main()
