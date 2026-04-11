from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
import creep.logic.super_basic_strategy as sbs
import creep.logic.basic_strategy as bs

def main():
    creep = CreepRobot()
    try:
        creep.initialise()
        bs.strategy_base(creep)
    except Exception as e:
        print("An error occurred:", e)
        creep.error_jingle()
    finally:
        # go_home_norm(creep)
        home_coords = creep.get_home_coords()
        go_to_coords(creep, int(home_coords[0]), int(home_coords[1]))
        creep.startup_jingle()

if __name__ == "__main__":
    main()
