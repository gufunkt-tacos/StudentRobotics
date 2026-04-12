from creep.machine import *
import creep.logic.basic_logic as bl
import creep.logic.robot_subroutines as rs
import creep.logic.bsearch as bsearch

def strategy_base(creep: CreepRobot):

    creep.Doors_wedge()

    base = rs.find_closest_token(creep, ObjectType.BASE, 0)
    if base is not None:
        rs.go_to_closest_token(creep, ObjectType.BASE, base, False, True)
        base = rs.find_closest_token(creep, ObjectType.BASE, 0)
        if base is None:
            rs.recovery(creep)
        if rs.is_on_floor(creep, base):
            creep.Doors_open()
            creep.drive_speed_distance(40, 50)
            creep.Doors_wedge()
        else:
            distance, turn_angle, final_angle =rs.get_normal_to_token(creep, base, 50)
            creep.turn_speed_angle(16*rs.sign(turn_angle), abs(turn_angle))
            creep.drive_speed_distance(40, distance)
            creep.turn_speed_angle(16*rs.sign(final_angle), abs(final_angle))
            rs.get_ledge_token(creep, ObjectType.BASE, 0)
    else:
        time_elapsed = (time.time() - creep.time_started_game)
        if time_elapsed >= creep.go_home_time:
            raise Exception("Time to go NOW!") 
        
        strategy_base(creep)






    #if there's time, turn round the corner and get more boxes
    #once the time reaches a threshold, go home.