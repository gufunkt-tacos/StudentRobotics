from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
import time

def main():
    creep = CreepRobot()
    creep.initialise()
    obj = None
    while not obj and creep.can_continue():
        obj = find_closest_token(creep, ObjectType.TOKEN)

    if not creep.can_continue():
        print("Failed to find token within time limit")
        return
    
    square_to_ledge(creep, obj)

if __name__ == "__main__":
    main()
