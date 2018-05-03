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

    pygame.init()

    BLACK = (0, 0, 0)
    WIDTH = 128
    HEIGHT = 102
    windowSurface = pygame.display.set_mode((WIDTH, HEIGHT), 0, 32)

    windowSurface.fill(BLACK)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

        # get key current state
        keys = pygame.key.get_pressed()
        if keys[K_LEFT]:
            roll -= 1
        elif keys[K_RIGHT]:
            roll += 1
        if keys[K_DOWN]:
            pitch -= 1
        elif keys[K_UP]:
            pitch += 1
        if keys[K_s]:
            thrust -= 1
        elif keys[K_w]:
            thrust += 1
        if keys[K_a]:
            yaw -= 1
        elif keys[K_d]:
            yaw += 1

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
