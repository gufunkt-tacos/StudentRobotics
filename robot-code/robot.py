from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
import time

def main():
    creep = CreepRobot()
    creep.initialise()
    obj = find_closest_token(creep, ObjectType.TOKEN)
    print(square_to_ledge())


if __name__ == "__main__":
    main()
