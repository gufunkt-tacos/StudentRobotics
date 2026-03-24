from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
import time

def main():
    creep = CreepRobot()
    creep.initialise()
    obj = None
    while not obj:
        obj = find_closest_token(creep, ObjectType.TOKEN)

    collect_box_from_ledge(creep, obj)


if __name__ == "__main__":
    main()
