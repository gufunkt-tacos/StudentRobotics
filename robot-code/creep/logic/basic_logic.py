import time

from ..machine import CreepRobot, ObjectType, Object, ARENA_MARKER_COORS, get_marker_coords
from math import atan2, cos, degrees, radians, sin
from threading import Thread

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

def get_current_estimated_position(creep: CreepRobot) -> tuple | None:
    # Read any available arena markers to find the current position of the robot
    markers = creep.find_objects(ObjectType.ARENA_MARKER)
    if markers:
        marker = markers[0]  # Assuming the first marker is the one we want
        marker_coords = get_marker_coords(marker.id)
        marker_distance = marker.position
        marker_angle = marker.h_angle
        # Calculate the robot's position based on the marker's position and angle
        robot_x = marker_coords[0] - marker_distance * cos(radians(marker_angle))
        robot_y = marker_coords[1] - marker_distance * sin(radians(marker_angle))
        return (robot_x, robot_y)
    else:
        return None  # No markers found, position cannot be estimated


def go_to_coords(creep: CreepRobot, x: int, y: int) -> bool:
    """ Go and collect a box at a location and avoid obstacles on the way

    Args:
        creep (CreepRobot): the robot instance to control
        x (int): the x coordinate of the box
        y (int): the y coordinate of the box
    """

    # Point the robot towards where the box should be
    robot_pos = get_current_estimated_position(creep)
    print(f"Estimated robot position: {robot_pos}")
    if robot_pos is not None:
        robot_x, robot_y = robot_pos
        angle_to_coords = atan2(y - robot_y, x - robot_x)
        angle_to_coords_degrees = degrees(angle_to_coords)
        print(f"Angle to box: {angle_to_coords_degrees} degrees")
        creep.turn_speed_angle(round(5 * (angle_to_coords_degrees/abs(angle_to_coords_degrees))), abs(angle_to_coords_degrees))
        # Move towards the box
        distance_to_coords = ((x - robot_x) ** 2 + (y - robot_y) ** 2) ** 0.5
        print(f"Distance to box: {distance_to_coords}")
        creep.drive_speed_distance(30, distance_to_coords)
        return True
    
    return False
    