import time

from ..machine import CreepRobot
from threading import Thread

def move_forward(creep: CreepRobot):
    creep.drive_sync(16, 0)
    time.sleep(1)
    creep.motor_stop()

def pick_up_box(creep: CreepRobot) -> bool:
    # Move arm to pick up position
    creep.Arm_tilt_up()
    creep.Arm_Extend(1)
    time.sleep(6)
    creep.VacValve("GRIP")
    creep.VacPump(1)
    creep.Arm_tilt_down()

    # Move arm back to initial position after picking up cube
    cutoff_time = time.time() + 4 # Set a cutoff time to prevent infinite loop
    success = False
    while time.time() < cutoff_time:
        time.sleep(0.1) # Wait until the cube is securely gripped
        if creep.sucker_gripping():
            success = True
            break
    
    print("Gripping cube:", "Success" if success else "Failed")
    if not success:
        # Asynchronously return lift arm and turn off pump
        def async_cleanup(creep: CreepRobot):
            creep.VacPump(0)
            creep.Arm_tilt_up()
        
        cleanup_thread = Thread(target=async_cleanup, args=(creep,))
        cleanup_thread.start()
        return False # Failed to grip cube within time limit
    
    def async_cleanup(creep: CreepRobot):
        # Return cube to robot
        creep.Arm_tilt_up()
        time.sleep(4)
        creep.Arm_Retract(1)
        time.sleep(7)
        creep.VacPump(0)
        creep.VacValve("VENT")
        time.sleep(0.1)
        creep.VacValve("GRIP")

        pull_cube_into_robot(creep)

    cleanup_thread = Thread(target=async_cleanup, args=(creep,))
    cleanup_thread.start()

    return True

def pull_cube_into_robot(creep: CreepRobot):
    # Pull cube into robot - can do this while moving forward to save time
    creep.Arm_tilt_up()
    creep.Arm_Extend(1)
    time.sleep(3)
    creep.Arm_tilt_down()
    time.sleep(1)
    creep.Arm_Retract(1)
    time.sleep(3)

    # Return arm to initial position
    creep.Arm_tilt_up()
    time.sleep(1)