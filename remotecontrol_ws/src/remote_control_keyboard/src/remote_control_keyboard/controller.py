#!/usr/bin/env python
import sys
import pygame
import rospy
from pygame.locals import *
from remote_control.msg import set_controller

def main():
    rospy.init_node("remote_key")
    pub = rospy.Publisher("/remote_control/set_controller", set_controller, queue_size=10)

    roll = 0
    pitch = 0
    thrust = 0
    yaw = 0

    movementAmount = 1

    pygame.init()

    BLACK = (0, 0, 0)
    WIDTH = 128
    HEIGHT = 102
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

    windowSurface.fill(BLACK)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        # get key current state
        keys = pygame.key.get_pressed()
        if keys[K_LEFT]:
            roll -= movementAmount
            if roll < -100: roll = -100
        elif keys[K_RIGHT]:
            roll += movementAmount
            if roll > 100: roll = 100
        if keys[K_DOWN]:
            pitch -= movementAmount
            if pitch < -100: pitch = -100
        elif keys[K_UP]:
            pitch += movementAmount
            if pitch > 100: pitch = 100
        if keys[K_s]:
            thrust -= movementAmount
            if thrust < 0: thrust = 0
        elif keys[K_w]:
            thrust += movementAmount
            if thrust > 100: thrust = 100
        if keys[K_a]:
            yaw -= movementAmount
            if yaw < -100: yaw = -100
        elif keys[K_d]:
            yaw += movementAmount
            if yaw > 100: yaw = 100

        if keys[K_SPACE]:
            thrust = 0
            pitch = 0
            roll = 0
            yaw = 0

        # rospy.loginfo(thrust)

        msg = set_controller(thrust, roll, pitch, yaw)
        pub.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
