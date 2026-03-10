from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import move_forward, pick_up_box
from creep.logic.robot_subroutines import get_floor_token
import time

def main():
    creep = CreepRobot()
    creep.initialise()

    while True:
        print("New Sonar Distance = " + str(CreepRobot.right_front_sonar()))
        print("Old Sonar Distance = " + str(CreepRobot.right_front_sonar_old()))
        time.sleep(0.5)
        

if __name__ == "__main__":
    main()
