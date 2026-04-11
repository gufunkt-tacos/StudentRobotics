from creep.machine import *
import creep.logic.basic_logic as bl
import creep.logic.robot_subroutines as rs

def strategy_base(creep: CreepRobot):
    #look for the closest box; get it
    rs.get_token(creep, ObjectType.BASE, "floor")

    if not creep.can_continue():
        return

    #go to the red box but dont get it; push out the way
    # if go_to_closest_token(creep, ObjectType.ACID, find_closest_token(creep, ObjectType.ACID, 0), False): # False prevents the doors from opening
    #     print("Pushed red box out the way")
    # else:
    #     print("No red box found, moving on")

    creep.drive_speed_distance(-40, 55)
    creep.turn_speed_angle(16, 165)
    creep.Doors_open()
    creep.drive_speed_distance(-40, 60)
    creep.Doors_wedge()
    creep.turn_speed_angle(-16, 85)

    bases = creep.find_objects(ObjectType.BASE)
    sorted_bases = sorted(bases, key=lambda d: d.position)
    
    target = sorted_bases[-1]

    rs.go_to_closest_token(creep, ObjectType.BASE, target, True)

    creep.turn_speed_angle(16, 165)

    return

    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.