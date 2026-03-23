from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
import time

def main():
    creep = CreepRobot()
    creep.initialise()
    collect_box_from_ledge(creep)

if __name__ == "__main__":
    main()
