from ..machine import CreepRobot, Object, ObjectType
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from statistics import mean


def sign(x):
    if(x>=0):
        return 1
    else:
        return -1


def find_closest_token(creep: CreepRobot, type: ObjectType, angle_offset: float = 0.0) -> Object | None:
    valid_tokens = []
    valid_tokens = creep.find_objects(type)
    if valid_tokens:
        closest_token = valid_tokens[0]
        for i in range(0,len(valid_tokens)):
            if(valid_tokens[i].position < closest_token.position):
                closest_token = valid_tokens[i]

        print("angle seen = " + str(closest_token.h_angle))

        closest_token.h_angle -= angle_offset

        return closest_token
    else:
        return None

    
def go_to_closest_token(creep: CreepRobot, type: ObjectType, closest_token: Object) -> None:

    scaling = 1
    creep.camera_pan(0)

    closest_token_dist = closest_token.position
    token_angle = closest_token.h_angle
   
    print(closest_token_dist)
    creep.turn_speed_angle(10*sign(token_angle), abs(token_angle)/scaling)

    # Try to find the token again after turning, to get another reading
    new_closest_token = find_closest_token(creep, type, 0)
    if new_closest_token:
        closest_token_dist = new_closest_token.position
        token_angle = new_closest_token.h_angle   
    
    creep.turn_speed_angle(5*sign(token_angle), (abs(token_angle)/scaling)/2)

    creep.drive_speed_distance(40, closest_token_dist)
    creep.Doors_open()
    creep.drive_speed_distance(40, 60)
    creep.Doors_close()
    creep.Doors_wedge()


def get_floor_token(creep: CreepRobot, type: ObjectType) -> bool:
    # Find a token in front of the robot
    closest_token = find_closest_token(creep, type)
    if closest_token:
        go_to_closest_token(creep, type, closest_token)
        return True
    
    # If no token is found, pan the camera to the left and right to try and find one
    angle_offset = 45
    creep.camera_pan(angle_offset)
    closest_token = find_closest_token(creep, type, angle_offset)

    if closest_token:
        go_to_closest_token(creep, type, closest_token)
        return True
    
    # If still no token is found, pan the camera to the right and try again
    angle_offset = -45
    creep.camera_pan(angle_offset)
    closest_token = find_closest_token(creep, type, angle_offset)

    if closest_token:
        go_to_closest_token(creep, type, closest_token)
        return True

    # If no token is found after panning, return the camera to the center and return False
    creep.camera_pan(0)
    return False


def get_token(creep: CreepRobot, type: ObjectType, level: str) -> bool:
    creep.camera_pan(0)
    creep.Doors_wedge()
    if (level == "floor"):
        return get_floor_token(creep, type)
    return False

    
def next_arena_token(start: int, step: int, direction: int) -> int:
    token = start + (step*direction)
    if(token > 19):
        token-=20
    if(token < 0):
        token+=20
    return token
    

"""
Dylan to Dan - 
    Not sure what this code is supposed to do so I have tried to refactor  
    but it may be wrong
"""
def go_home(creep: CreepRobot):
    
    creep.Doors_wedge()
    closest_token_dist = 8192
    closest_token_angle = 361
    marker_found = False

    arena_markers = creep.find_objects(ObjectType.ARENA_MARKER)
    if (arena_markers):
        for i in range(len(arena_markers)):
            for j in range(0,2):
                if(arena_markers[i].position == creep.my_lab[j]):
                    marker_found = True
                    if(arena_markers[i].position < closest_token_dist):
                        closest_token_dist = arena_markers[i].position
                        closest_token_angle = arena_markers[i].h_angle
            print(str(arena_markers[i]))
    
    if marker_found:
        creep.turn_speed_angle(5, closest_token_angle)
        creep.drive_speed_distance(30, closest_token_dist)
        return

    # If no arena marker is found, try to find the next one in the sequence
    for step in range(1,10):
        if (arena_markers):
            for i in range(len(arena_markers)):
                if(arena_markers[i].position == next_arena_token(creep.my_lab[0],step,-1) or arena_markers[i].position == next_arena_token(creep.my_lab[2],step,1)):
                    marker_found = True
                    if(arena_markers[i].position < closest_token_dist):
                        closest_token_dist = arena_markers[i].position
                        closest_token_angle = arena_markers[i].h_angle
    
    if marker_found:
        creep.turn_speed_angle(5, closest_token_angle)
        creep.drive_speed_distance(30, closest_token_dist)
        creep.turn_speed_angle(5,90*sign(closest_token_angle))
    else:
        creep.drive_speed_distance(30,25)
    
    go_home(creep)


def navigate_obstacle(creep: CreepRobot):
    creep.Doors_wedge()
    right_sonar = creep.right_front_sonar()
    left_sonar = creep.left_front_sonar()
    if(right_sonar < 30 or left_sonar < 30):
        valid_tokens = creep.find_objects(ObjectType.TOKEN)
        print(valid_tokens)

        closest_token_dist = float('inf') # in case valid tokens is empty
        if valid_tokens:
            closest_token_dist = valid_tokens[0].position
            print(len(valid_tokens))
            for i in range(len(valid_tokens)):
                if(valid_tokens[i].position < closest_token_dist):
                    closest_token_dist = valid_tokens[i].position
            
        if (closest_token_dist > 50):
            creep.turn_speed_angle(32*sign(right_sonar-left_sonar),90)
            creep.drive_speed_distance(20,50)
            creep.turn_speed_angle(32*-sign(right_sonar-left_sonar),90)


@dataclass
class LedgeConfig:
    """
    All tunable parameters for the ledge-pickup sequence in one place.
 
    Distances are in centimetres, angles in degrees, times in seconds.
    """
 
    # APPROACH/ALIGNMENT
 
    # How far from the ledge (measured by the front sonars) we want to stop
    target_dist_to_ledge: float = 5.0
    
    initial_dist_to_ledge: float = 1000

    target_distance_in_front_of_box: float = 25

    cam_step_distance = 5
 
    # Both sonars must agree to within this many cm before we call ourselves square to the ledge
    distance_alignment_tolerance: float = 5.0

    angle_alignment_tolerance: float = 5.0

    center_advance_cm: float = 5.0
 
    # Physical distance between the two front sonar sensors on the robot
    sonar_separation: float = 37.9
 
    # Drive / turn speed used during the alignment phase
    alignment_drive_speed: int = 15
    alignment_turn_speed:  int = 10
 
    # Maximum wall-clock time we'll spend trying to square up before giving up.
    alignment_timeout: float = 20.0
 
    # How many sonar samples to average during alignment (more = slower but
    # steadier readings).
    sonar_samples: int = 5

 
    # ARM EXTENSION
 
    # Motor current threshold below which we consider the linear actuator to have reached its hard stop (fully extended or retracted).
    arm_stall_current: float = 0.01
 
    # Maximum time to wait for the arm to finish extending or retracting.
    arm_timeout: float = 10.0

 
    # GRIPPING
 
    # Box height (IR sensor reading) below which we assume the sucker is directly over a box surface.
    box_height_threshold: float = 7.5
 
    # How long to attempt to achieve suction before declaring grip failure
    grip_timeout: float = 8.0
 
    # How long the grip must remain stable before we trust it
    grip_confirmation_dwell: float = 0.5

 
    # WIGGLE SEACH
 
    # Speed used to rotate in place while scanning for the box
    wiggle_speed: int = 15
 
    # Duration of each wiggle half-swing.
    wiggle_duration: float = 0.5
 
    # Number of full left-right-centre cycles before giving up the box search
    wiggle_retries: int = 3

 
    # RETREAT
 
    # How far to reverse after completing (or failing) a pickup attempt
    retreat_distance: float = 50.0
 
    # Speed at which to reverse.
    retreat_speed: int = -30
    wiggle_center_delay: float = 0.15 
 
 
 

DEFAULT_LEDGE_CONFIG = LedgeConfig()

class LedgePickupResult(Enum):
    """
    Describes the outcome of a full ledge-pickup sequence.
 
    """
    SUCCESS              = auto()  # Box gripped and retracted cleanly
    NO_BOX_FOUND         = auto()  # Arm extended but IR sensor never saw a box
    GRIP_FAILED          = auto()  # IR sensor saw a box but suction never held
    GRIP_LOST_ON_RETRACT = auto()  # Grip was good but lost during arm retraction
    ALIGNMENT_TIMEOUT    = auto()  # Could not square up to the ledge in time
    NAV_FAILED           = auto()  # Could not navigate to the ledge at all

def navigate_to_ledge_position(
    creep: CreepRobot,
    obj: Object,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Drive from the robot's current position to a point directly in front of
    the ledge (d cm along the ledge's outward normal), ready for alignment.
    Coordinate frame: x = right, y = forward (robot-centric, top-down).
    obj.h_angle: degrees, 0 = ahead, positive = right.
    obj.yaw:     degrees, 0 = marker facing robot square-on,
                 positive = clockwise from marker's outward perspective
                 (= counterclockwise from robot's view).
    Returns True if all manoeuvres completed, False if any timed out.
    """

    creep.Arm_Extend(1)
    
    turn_angle  = obj.h_angle
    drive_dist  = obj.position - cfg.initial_dist_to_ledge

    print(
        f"  turn_to_P={turn_angle:.1f}°  drive={drive_dist:.1f} cm  "
    )
    creep.Doors_close()
    creep.Arm_tilt_up()
    # Step 1 — turn to face the approach point
    if abs(turn_angle) > 1.0:
        ok = creep.turn_speed_angle(30 * sign(turn_angle), abs(turn_angle))
        if not ok:
            print("[navigate_to_ledge_position] initial turn timed out")
            return False
    # Step 2 — drive to approach point
    if drive_dist > 0.5:
        ok = creep.drive_speed_distance(30, drive_dist)
        if not ok:
            print("[navigate_to_ledge_position] drive timed out")
            return False
    # Step 3 — turn to face the ledge squarely
    print("[navigate_to_ledge_position] complete")
    return True

def get_pos_with_sonar(creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG):
    L = creep.left_front_sonar(cfg.sonar_samples) - cfg.target_dist_to_ledge
    R = creep.left_front_sonar(cfg.sonar_samples) - cfg.target_dist_to_ledge
    speed1 = speed2 = 0
    if abs(L) >= cfg.distance_alignment_tolerance:
        speed1 = cfg.alignment_drive_speed * sign(L)
        in_position = False
    if abs(R) >= cfg.distance_alignment_tolerance:
        speed2 = cfg.alignment_drive_speed * sign(R)
        in_position = False
    
    turn_angle = math.atan2(L - R, cfg.sonar_separation)
    if abs(turn_angle) >= cfg.angle_alignment_tolerance:
        speed1 += turn_angle * 0.5
        speed2 -= turn_angle * 0.5
        in_position = False

    return (speed1, speed2, in_position)


def square_to_ledge(
    creep: CreepRobot,
    obj: Object,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    """
    creep.Doors_close()
    print("[square_to_ledge] phase 1 — camera-guided approach")
    markers = creep.find_objects(obj.type)
    current = None
    
    if markers is None:
        return False

    for marker in markers:
        if marker.id == obj.id:
            current = marker

    if current is None:
        return False
            

    L = creep.left_front_sonar(cfg.sonar_samples)
    R = creep.right_front_sonar(cfg.sonar_samples)
    spread = L - R

    a = math.atan2(spread, cfg.sonar_separation)

    r = cfg.target_distance_in_front_of_box
    d = current.position
    h = math.degrees(current.h_angle)

    distance_to_move = d**2 + r**2 - 2 * d * r * math.cos(a - h)

    angle_to_move = math.sin(a - h) * r / distance_to_move

    creep.turn_speed_angle(15, angle_to_move)
    creep.drive_speed_distance(30 , distance_to_move)

    final_turn_angle = 180 - (a - h) - angle_to_move

    creep.turn_speed_angle(15, final_turn_angle)


    markers = creep.find_objects(obj.type)
    current = None

    if markers is not None:
        for marker in markers:
            if marker.id == obj.id:
                current = marker

    while not in_position:
        in_position = True

        if current is None:
            data_from_sonar = get_pos_with_sonar(creep, cfg)
            in_position = data_from_sonar[2]
            speed1 = data_from_sonar[0]
            speed2 = data_from_sonar[1]

        else:
            turn_angle = current.h_angle
            drive_dist = current.position - cfg.target_dist_to_ledge

            if abs(drive_dist) >= cfg.distance_alignment_tolerance:
                speed1 = speed2 = cfg.alignment_drive_speed * sign(drive_dist)
                in_position = False
            else:
                speed1 = speed2 = 0
            

            if abs(turn_angle) >= cfg.angle_alignment_tolerance:
                speed1 += turn_angle * 0.5
                speed2 -= turn_angle * 0.5
                in_position = False



        creep.drive_both(round(speed1), round(speed2))

        # IR check after every movement
        if box_is_present(creep, cfg):
            print("[square_to_ledge] IR triggered in camera phase — centering")
            creep.drive_speed_distance(cfg.alignment_drive_speed,
                                       cfg.target_dist_to_ledge)
            return True
        
        
        markers = creep.find_objects(obj.type)
        current = None

        if markers is not None:
            for marker in markers:
                if marker.id == obj.id:
                    current = marker

    # ── Phase 3: wiggle fallback ──────────────────────────────────────────────
    print("[square_to_ledge] phase 3 — wiggle fallback")
    return scan_for_box_on_ledge(creep, cfg)



def scan_for_box_on_ledge(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Search for a box on the ledge by wiggling left and right.
    Pattern: centre → right → left (double) → centre, repeated cfg.wiggle_retries times.
    When the IR sensor triggers, the robot continues moving for cfg.wiggle_center_delay
    seconds before stopping so the arm centre lands on the box.
    """
    print(f"[scan_for_box_on_ledge] searching with {cfg.wiggle_retries} retry cycles")
    if box_is_present(creep, cfg):
        print("[scan_for_box_on_ledge] box found at centre immediately")
        time.sleep(cfg.wiggle_center_delay)
        creep.motor_stop()
        return True
    for attempt in range(cfg.wiggle_retries):
        print(f"  [attempt {attempt + 1}/{cfg.wiggle_retries}]")
        # WIGGLE RIGHT
        t0 = time.time()
        creep.drive_both(cfg.wiggle_speed, -cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration:
            if box_is_present(creep, cfg):
                time.sleep(cfg.wiggle_center_delay)   # let arm reach centre
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found during right wiggle")
                return True
        creep.motor_stop()
        # WIGGLE LEFT (double duration to sweep back through centre)
        t0 = time.time()
        creep.drive_both(-cfg.wiggle_speed, cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration * 2:
            if box_is_present(creep, cfg):
                time.sleep(cfg.wiggle_center_delay)   # let arm reach centre
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found during left wiggle")
                return True
        creep.motor_stop()
        # RETURN TO CENTRE
        t0 = time.time()
        creep.drive_both(cfg.wiggle_speed, -cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration:
            if box_is_present(creep, cfg):
                time.sleep(cfg.wiggle_center_delay)   # let arm reach centre
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found returning to centre")
                return True
        creep.motor_stop()
        if box_is_present(creep, cfg):
            print("[scan_for_box_on_ledge] box found at centre after wiggle cycle")
            time.sleep(cfg.wiggle_center_delay)
            creep.motor_stop()
            return True
    print("[scan_for_box_on_ledge] box not found after all retries")
    return False


def box_is_present(creep: CreepRobot, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    Return True if the IR height sensor indicates a box is beneath the sucker
    """
    height = creep.sucker_height()
    print(f"[box_is_present] IR height = {height:.2f} cm (threshold {cfg.box_height_threshold})")
    return height <= cfg.box_height_threshold


def grip_box(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Attempt to grip the box that is currently beneath the sucker.
 
    Sequence:
      1. Activate vacuum pump and valve.
      2. Tilt arm down so sucker makes contact.
      3. Wait for suction to build and hold (confirm_grip).
      4. On success: leave vacuum running and return True.
      5. On failure: stop vacuum, tilt back up and return False.
    """
    print("[grip_box] activating vacuum and lowering arm…")
 
    creep.VacValve("GRIP")
    creep.VacPump(1)
    creep.Arm_tilt_down()
 
    # Give a short dwell for the arm to reach its down position and for the sucker to make firm contact before we start checking pressure.
    time.sleep(0.3)
 
    if confirm_grip(creep, cfg):
        print("[grip_box] box gripped")
        return True
    else:
        # Abort: vent, tilt back up, stop vacuum
        print("[grip_box] grip failed, aborting")
        creep.VacPump(0)
        creep.VacValve("VENT")
        time.sleep(0.2)
        creep.VacValve("GRIP")
        creep.Arm_tilt_up()
        return False
    

def confirm_grip(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Wait up to *cfg.grip_timeout* seconds for the sucker to achieve and
    hold a stable grip.
 
    Returns True if grip was confirmed, False on timeout.
    """
    t_start    = time.time()
    grip_since = None   # time when grip was first detected this cycle
 
    while time.time() - t_start < cfg.grip_timeout:
        if creep.sucker_gripping():
            if grip_since is None:
                grip_since = time.time()
                print("[confirm_grip] grip detected waiting for dwell…")
            elif time.time() - grip_since >= cfg.grip_confirmation_dwell:
                print("[confirm_grip] grip confirmed")
                return True
        else:
            if grip_since is not None:
                print("[confirm_grip] grip dropped resetting dwell timer")
            grip_since = None   # lost grip reset dwell timer
        time.sleep(0.05)
 
    print(f"[confirm_grip] timed out after {cfg.grip_timeout:.1f}s: no grip")
    return False

def retract_with_box(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Retract the arm with the box held under suction, periodically verifying
    that the grip has not been lost.
 
    Sequence:
      1. Tilt arm up (lifts box clear of the ledge surface).
      2. Retract arm while monitoring grip.
      3. Wait for arm to finish (stall current).
      4. Final grip check.
 
    Returns True if retraction completed with the grip intact.
    Returns False if grip was lost mid-retraction (caller should check whether
    the box just dropped or was deposited somewhere usable).
 
    """
    print("[retract_with_box] tilting arm up and retracting…")
 
    # tilt up first to lift the box off the shelf
    creep.Arm_tilt_up()
    time.sleep(0.4)  
 
    # start retracting
    creep.Arm_Retract(1)
 
    # Poll grip and arm-finished together
    t_start = time.time()
    while not has_arm_finished(creep, cfg):
        if time.time() - t_start > cfg.arm_timeout:
            print("[retract_with_box] retraction timed out")
            creep.Arm_Stop()
            # Still check the grip, we may have the box even if we timed out
            break
 
        if not creep.sucker_gripping():
            # Grip lost during retraction.  Stop and report.
            print("[retract_with_box] grip lost during retraction!")
            creep.Arm_Stop()
            creep.VacPump(0)
            creep.VacValve("VENT")
            time.sleep(0.2)
            creep.VacValve("GRIP")
            return False
 
        time.sleep(0.05)
 
    creep.Arm_Stop()
    time.sleep(0.2)
 
    # FINAL GRIP CHECK
    if creep.sucker_gripping():
        print("[retract_with_box] arm retracted with box secured")
        return True
    else:
        print("[retract_with_box] final grip check failed: box may have been lost")
        creep.VacPump(0)
        creep.VacValve("VENT")
        time.sleep(0.2)
        creep.VacValve("GRIP")
        return False
    

def has_arm_finished(creep: CreepRobot, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    Return True when the arm linear actuator has reached its hard stop.
 
    The SR motor board reports the current through the motor.  When the
    actuator hits a mechanical limit the motor stalls and its current drops
    toward zero.  We use a threshold slightly above zero to account for
    measurement noise.
    """
    current = creep.robot.motor_boards["SR0VJ1K"].motors[1].current
    return current <= cfg.arm_stall_current


def retreat_from_ledge(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> None:
    """
    Reverse away from the ledge after a pickup attempt (successful or not).
 
    Always called regardless of outcome so the robot is in a safe position
    for the next action.
    """
    print(f"[retreat_from_ledge] reversing {cfg.retreat_distance:.0f} cm")
    creep.drive_speed_distance(-abs(cfg.retreat_speed), cfg.retreat_distance)


def clamp(number: float) -> float:
    """
    Sets min and max of a number at -1 and 1 respectively
    """
    return max(min(number, 1), -1)


def collect_box_from_ledge(creep: CreepRobot, obj: Object, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> LedgePickupResult:
    """
    Master entry-point for all ledge related functions.

    Call this function once a ledged box has been found and needs to be collected.

    The robot will then retreat *cfg.retreat_distance* from the centre pillion, ready for further collection attempts.
    """
    if not navigate_to_ledge_position(creep, obj, cfg):
        return LedgePickupResult.NAV_FAILED
    # square_to_ledge now handles camera → sonar → wiggle internally.
    # Returns True only when the box is found and the arm is centred.
    if not square_to_ledge(creep, cfg):
        return LedgePickupResult.NO_BOX_FOUND      # ← was ALIGNMENT_TIMEOUT
    # ← REMOVE the scan_for_box_on_ledge call that was here
    if not grip_box(creep, cfg):
        return LedgePickupResult.GRIP_FAILED
    if not retract_with_box(creep, cfg):           # ← bug 3 fixed
        return LedgePickupResult.GRIP_LOST_ON_RETRACT
    retreat_from_ledge(creep, cfg)                 # ← bug 4 fixed
    print("Box successfully collected from ledge.")
    return LedgePickupResult.SUCCESS


    



