from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *

def strategy_base(creep: CreepRobot):
    #look for the closest box; get it
    get_token(creep, ObjectType.BASE, "floor")

    #go to the red box but dont get it; push out the way
    go_to_closest_token(creep, ObjectType.ACID, find_closest_token(creep, ObjectType.ACID, 0), False) # False prevents the doors from opening

    #get the one on the ledge
    get_ledge_token(creep, ObjectType.BASE,90)

    #if we see the next blue box, get it. otherwise, go to where it would be
    floor_token_seen = get_token(creep, ObjectType.BASE, "floor")
    if(floor_token_seen != True):
        creep.drive_speed_distance(16, 100)



    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.