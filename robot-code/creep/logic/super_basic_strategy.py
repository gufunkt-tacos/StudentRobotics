from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *

def strategy_base(creep: CreepRobot):
    #look for the closest box; get it
    get_token(creep, ObjectType.BASE, "floor")

    creep.drive_speed_distance(-40, 120)


    return

    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.