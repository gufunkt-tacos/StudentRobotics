from creep.machine import CreepRobot
from creep.logic.basic_logic import move_forward, pick_up_box

print(f"__name__ in robot.py is {__name__}")

def main():
    creep = CreepRobot()
    creep.initialise()
    pick_up_box(creep)

if __name__ == "__main__":
    main()
