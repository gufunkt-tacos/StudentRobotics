from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
from creep.logic.basic_strategy import *

def main():
    creep = CreepRobot()
    try:
        creep.initialise()
        raise Exception("Test exception to check error handling") # Remove this line after confirming error handling works
        strategy_base(creep)
    except Exception as e:
        print("An error occurred:", e)
        creep.error_jingle()
    finally:
        go_home_norm(creep)

if __name__ == "__main__":
    main()
