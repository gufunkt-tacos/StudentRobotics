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
    time.sleep(1) # Wait for suction to take effect

    # Move arm back to initial position after picking up cube
    cutoff_time = time.time() + 3 # Set a cutoff time to prevent infinite loop
    while not creep.sucker_gripping() and time.time() < cutoff_time:
        time.sleep(0.1) # Wait until the cube is securely gripped
    
    if time.time() >= cutoff_time:
        # Asynchronously return the arm and stop the pump
        def async_cleanup(creep: CreepRobot):
            creep.Arm_Retract(1)
            time.sleep(7)
            creep.VacPump(0)
        
        cleanup_thread = Thread(target=async_cleanup, args=(creep,))
        cleanup_thread.start()
        return False # Failed to grip cube within time limit
    
    def async_cleanup(creep: CreepRobot):
        creep.Arm_tilt_up()

        creep.Arm_Retract(1)
        time.sleep(7)
        creep.VacPump(0)
        creep.VacValve("VENT")
        time.sleep(0.1)
        creep.VacValve("GRIP")

    cleanup_thread = Thread(target=async_cleanup, args=(creep,))
    cleanup_thread.start()

    return True

async def pull_cube_into_robot(creep: CreepRobot):
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