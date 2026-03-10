from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import move_forward, pick_up_box, go_to_coords
from creep.logic.robot_subroutines import get_floor_token, go_home

def main():
    creep = CreepRobot()
    creep.initialise()
    go_to_coords(creep, 0, 0)  # Example coordinates for the box

if __name__ == "__main__":
    main()
