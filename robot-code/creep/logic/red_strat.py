from creep.machine import CreepRobot, ObjectType
import creep.logic.robot_subroutines as rs
import creep.logic.basic_logic as bl
from creep.logic.stategy_base import BaseStrategy

class RedStrat(BaseStrategy):

    def strategy_acid(self, creep: CreepRobot) -> bool:
        #look for the closest box; get it
        rs.get_token(creep, ObjectType.ACID, "floor")

        if not creep.can_continue():
            return False
        
        creep.turn_speed_angle(16, 30)
        # Look to see if base is there, if so, push it into the opponents side
        bases = creep.find_objects(ObjectType.BASE)
        if bases and len(bases) > 0:
            closest_base = min(bases, key=lambda b: b.position)
            if closest_base.position > 90:
                creep.turn_speed_angle(16, 180)
            else:
                rs.go_to_closest_token(creep, ObjectType.BASE, closest_base, 0)
                # Find closest arena token and push the box towards it
                arena_markers = creep.find_objects(ObjectType.ARENA_MARKER)
                if arena_markers and len(arena_markers) > 0:
                    closest_marker = min(arena_markers, key=lambda m: m.position)
                    rs.go_to_closest_token(creep, ObjectType.ARENA_MARKER, closest_marker, 0)

                # Back up a little
                creep.drive_speed_distance(-40, 20)
                # Turn around and go to the blue cube back at base to orient towards the red cube on the shelf
                creep.turn_speed_angle(16, 165)
                rs.go_to_closest_token(creep, ObjectType.BASE, closest_base, 0, stop_short=True)
                creep.turn_speed_angle(16, 85)

        else:
            creep.turn_speed_angle(16, 180)

        # Now facing the ledge look for the closest red cube and get it from the ledge
        acid_tokens = creep.find_objects(ObjectType.ACID)
        if acid_tokens and len(acid_tokens) > 0:
            closest_acid = min(acid_tokens, key=lambda a: a.position)
            rs.get_in_position(creep, target_token_id=closest_acid.id)
            bl.pick_up_box(creep)

            return True # Go home
        
        return False # Go home