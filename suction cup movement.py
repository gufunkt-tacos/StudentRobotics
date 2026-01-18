from sr.robot3 import Robot
import math
import numpy as np
from math import cos, sin, pi, sqrt, atan2 # i have no idea if we need these, what even is atan2? arctan squared? arctan 2? is it even arctan?
robot = Robot()

def suction_cup(power):
    uhhhhh_idk_how_this_works = True
    #i'm not sure what to do here

motorboard_serial = "xxxxxx" # idk the serial
arm_motorboard = robot.motor_boards[motorboard_serial]
box_detected = False
detection_dist = 100 # idk how far we need to see from

while True:

    markers = robot.camera.see()

    for marker in markers:
        if(marker.position.distance < detection_dist and -pi/12 < marker.position.horizontal_angle < pi/12): # if it's the right distance away and within 30deg (tolerance may be too high here)
            box_detected = True
        else:
            box_detected = False

    if(box_detected == True):

        arm_motorboard.motor[0].power = 1
        robot.sleep(2)
        arm_motorboard.motor[0].power = 0
        suction_cup(1)
        arm_motorboard.motor[0].power = -1
        robot.sleep(2)
        arm_motorboard.motor[0].power = 0
        suction_cup(-1)
     
    robot.sleep(0.25)




