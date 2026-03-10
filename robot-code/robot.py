from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import move_forward, pick_up_box
from creep.logic.robot_subroutines import get_floor_token, go_home

def main():
    creep = CreepRobot()
    creep.initialise()
    go_home(creep)

if __name__ == "__main__":
    main()
