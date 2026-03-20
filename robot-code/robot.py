from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
from creep.logic.basic_strategy import *

def main():
    creep = CreepRobot()
    creep.initialise()
    strategy_base(creep)

if __name__ == "__main__":
    main()
