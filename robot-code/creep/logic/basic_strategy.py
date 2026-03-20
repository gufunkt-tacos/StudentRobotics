from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *

def strategy_base(creep: CreepRobot):
    #look for the closest box; get it
    get_token(creep, ObjectType.BASE, "floor")

    #go to the red box but dont get it; push out the way
    go_to_closest_token(creep, ObjectType.ACID, find_closest_token(creep, ObjectType.ACID, 0), False) # False prevents the doors from opening

    #if we see the next blue box, get it. otherwise, turn and get the box on the ledge
    floor_token_seen = get_token(creep, ObjectType.BASE, "floor")
    if(floor_token_seen != True):
        #logic for getting a legde token
        temp_var_to_prevent_error = 676767676767676767


    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.