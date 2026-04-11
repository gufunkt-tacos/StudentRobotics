from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *

def strategy_base(creep: CreepRobot):
    #look for the closest box; get it
    get_token(creep, ObjectType.BASE, "floor")

    if not creep.can_continue():
        return

    #go to the red box but dont get it; push out the way
    # if go_to_closest_token(creep, ObjectType.ACID, find_closest_token(creep, ObjectType.ACID, 0), False): # False prevents the doors from opening
    #     print("Pushed red box out the way")
    # else:
    #     print("No red box found, moving on")
    creep.drive_speed_distance(40, 82 + 13)

    creep.turn_speed_angle(-16, 90)

    obj = find_closest_token(creep, ObjectType.ARENA_MARKER)
    distance, turn_angle, final_angle = get_normal_to_token(creep, obj, 50)

    creep.turn_speed_angle(16 * sign(turn_angle), abs(turn_angle))
    creep.drive_speed_distance(40, distance)
    creep.turn_speed_angle(16 * sign(final_angle), abs(final_angle))
    creep.turn_speed_angle(16, 180)


    if not creep.can_continue():
        return

    #get the one on the ledge
    get_ledge_token(creep, ObjectType.BASE,90)

    if not creep.can_continue():
        return

    #if we see the next blue box, get it. otherwise, go to where it would be
    # get_token(creep, ObjectType.BASE, "floor")

    creep.turn_speed_angle(16, 180)

    return

    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.