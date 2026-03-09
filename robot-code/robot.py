from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import move_forward, pick_up_box
from creep.logic.robot_subroutines import get_floor_token

def main():
    creep = CreepRobot()
    creep.initialise()
    get_floor_token(creep, ObjectType.BASE)

if __name__ == "__main__":
    main()
