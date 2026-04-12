from creep.machine import *
import creep.logic.basic_logic as bl
import creep.logic.robot_subroutines as rs
import creep.logic.bsearch as bsearch

def strategy_base(creep: CreepRobot):

    creep.Doors_open()

    creep.drive_speed_distance(40, 130) # THIS MIGHT NEED TO CHANGED

    creep.Doors_wedge()

    closest_acid = rs.find_closest_token(creep, ObjectType.ACID, 0)
    if closest_acid is not None and closest_acid.position < 100:
        rs.go_to_closest_token(creep, ObjectType.ACID, closest_acid, False)
    else:
        creep.drive_speed_distance(-40, 60)
        creep.turn_speed_angle(16, 165)
        creep.Doors_open()
        creep.drive_speed_distance(-40, 60)
        creep.Doors_close()
        creep.turn_speed_angle(16, 165)
        return bsearch.strategy_base(creep)

    
    creep.turn_speed_angle(-16, 85)
    creep.drive_speed_distance(40, 60)
    creep.turn_speed_angle(16, 165)

    if not rs.get_ledge_token(creep, ObjectType.BASE, 0):
        return bsearch.strategy_base(creep)

    creep.turn_speed_angle(-16, 85)
    
    closest_basic = rs.find_closest_token(creep, ObjectType.BASE, 0)
    if closest_basic is not None:
        rs.go_to_closest_token(creep, ObjectType.BASE, closest_basic, False)
        creep.turn_speed_angle(16, 165)
        return
    else:
        creep.turn_speed_angle(16, 165)
        return



    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.