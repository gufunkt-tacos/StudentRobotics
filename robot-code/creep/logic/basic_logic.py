import time

from ..machine import CreepRobot
from asyncio import *

def move_forward(creep: CreepRobot):
    creep.drive_sync(16, 0)
    time.sleep(1)
    creep.motor_stop()

def pick_up_box(creep: CreepRobot):
    # Move arm to pick up position
    creep.Arm_tilt_up()
    creep.Arm_Extend(1)
    time.sleep(6)
    creep.VacValve("GRIP")
    creep.VacPump(1)
    creep.Arm_tilt_down()
    time.sleep(1) # Wait for suction to take effect

    # Move arm back to initial position after picking up cube
    while creep.sucker_gripping() == False:
        time.sleep(0.1) # Wait until the cube is securely gripped
    creep.Arm_tilt_up()

    creep.Arm_Retract(1)
    time.sleep(7)
    creep.VacPump(0)
    creep.VacValve("VENT")
    time.sleep(0.1)
    creep.VacValve("GRIP")

    # Pull cube into robot
    creep.Arm_tilt_up()
    creep.Arm_Extend(1)
    time.sleep(3)
    creep.Arm_tilt_down()
    time.sleep(1)
    creep.Arm_Retract(1)
    time.sleep(3)

    # Return arm to initial position
    creep.Arm_tilt_up()
    time.sleep(1)