from creep.machine import *
import creep.logic.robot_subroutines as rs

def strategy_base(creep: CreepRobot):

    print(str(creep.find_objects(ObjectType.ANY)))
    rs.get_ledge_token(creep, ObjectType.BASE, 0)
        



    #if there's time, turn round the corner and  get more boxes
    #once the time reaches a threshold, go home.