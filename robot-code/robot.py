from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
from creep.logic.super_basic_strategy import *

def main():
    creep = CreepRobot()
    try:
        creep.initialise()
        strategy_base(creep)
    except Exception as e:
        print("An error occurred:", e)
        creep.error_jingle()
    finally:
        # go_home_norm(creep)
        return

if __name__ == "__main__":
    main()
