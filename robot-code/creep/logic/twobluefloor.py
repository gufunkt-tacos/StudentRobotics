from creep.machine import *
import creep.logic.basic_logic as bl
import creep.logic.robot_subroutines as rs
from creep.logic.stategy_base import BaseStrategy
import creep.logic.bsearch as bsearch

class TwoBlueFloor(BaseStrategy):
    def strategy_base(self, creep: CreepRobot):
    
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

        bases = creep.find_objects(creep, ObjectType.BASE)
        if not bases:
            if not rs.search_for_boxes(creep, ObjectType.BASE):
                rs.align_to_wall(creep)
                return bsearch.strategy_base(creep)
            bases = creep.find_objects(creep, ObjectType.BASE)

        sorted_bases = sorted(bases, key=lambda d: d.position)
        if not sorted_bases:
            return bsearch.strategy_base(creep)
        target = sorted_bases[-1]

        rs.go_to_closest_token(creep, ObjectType.BASE, target, True)

        creep.turn_speed_angle(16, 165)

        return

        #if there's time, turn round the corner and get more boxes
        #once the time reaches a threshold, go home.
