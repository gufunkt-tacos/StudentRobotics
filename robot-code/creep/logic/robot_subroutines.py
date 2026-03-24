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
 
    # Both sonars must agree to within this many cm before we call ourselves square to the ledge
    alignment_tolerance: float = 1.0
 
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
 
    # RETRY LOGIC
 
    # How many full ledge-pickup attempts to make before returning failure.
    max_attempts: int = 1
 
    # After a failed attempt, rotate by this many degrees to search for the box at a slightly different position before retrying.
    retry_lateral_search_angle: float = 10.0
 
 

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
    the ledge, ready for the alignment phase.
 
    Returns True if the drive completed without timeout, False otherwise.
    """
    print("[navigate_to_ledge_position] computing approach geometry…")
    print(f"  obj.position={obj.position:.1f}  obj.yaw={obj.yaw:.1f}°  "
          f"target_dist={cfg.target_dist_to_ledge:.1f}")
 
    d = cfg.target_dist_to_ledge
    r = obj.position
    yaw_rad = math.radians(obj.yaw)
 
    # cosine rule: distance to the target approach point
    distance_to_move = math.sqrt(
        d**2 + r**2 - 2 * d * r * math.cos(yaw_rad)
    )
 
    # heading angle to turn through
    # guard against asin domain error from noisy inputs.
    sin_arg = (d * math.sin(yaw_rad)) / distance_to_move
    sin_arg = clamp(sin_arg, -1.0, 1.0)
    angle_to_move = math.degrees(math.asin(sin_arg))
 
    print(f"turn {angle_to_move:.1f}°  then drive {distance_to_move:.1f} cm")
 
    creep.Doors_close()     # protect the arm during driving
    creep.Arm_tilt_up()     # keep arm clear of obstacles
 
    # Turn
    if abs(angle_to_move) > 1.0:
        ok = creep.turn_speed_angle(
            30 * sign(angle_to_move), abs(angle_to_move))
        if not ok:
            print("[navigate_to_ledge_position] turn timed out")
            return False
 
    # Drive
    ok = creep.drive_speed_distance(30, distance_to_move)
    if not ok:
        print("[navigate_to_ledge_position] drive timed out")
        return False
 
    print("[navigate_to_ledge_position] complete")
    return True



def square_to_ledge(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Use both front sonars to align the robot squarely with the ledge and
    bring it to *cfg.target_dist_to_ledge* cm from it.

    """
    print(f"[square_to_ledge] aligning, target={cfg.target_dist_to_ledge} cm  "
          f"tol=±{cfg.alignment_tolerance} cm")
 
 
    # Arm should already be extended from the caller and doors closed
    creep.Doors_close()
 
    t_start = time.time()
    iteration = 0
 
    while True:
        iteration += 1
        elapsed = time.time() - t_start
 
        if elapsed > cfg.alignment_timeout:
            print(f"[square_to_ledge] timed out after {elapsed:.1f}s "
                  f"({iteration} iterations)")
            creep.motor_stop()
            return False
 
        # ── 1. Read sonars ───────────────────────────────────────────────────
        left_dist  = creep.left_front_sonar(samples=cfg.sonar_samples)
        right_dist = creep.right_front_sonar(samples=cfg.sonar_samples)
 
        print(f"  [iter {iteration}] L={left_dist:.1f}  R={right_dist:.1f}  "
              f"elapsed={elapsed:.1f}s")
 
        # sanity-check: sonars must be roughly consistent
        # if the difference is larger than the physical sensor separation then the ledge is at a very extreme angle, or one sensor is seeing somethingelse entirely
        # bail out rather than making things worse...
        if abs(left_dist - right_dist) > cfg.sonar_separation:
            print(f"[square_to_ledge] sonar spread too large "
                  f"({abs(left_dist - right_dist):.1f} > {cfg.sonar_separation:.1f}) "
                  f"– ledge may not be in front of the robot")
            creep.motor_stop()
            return False
 
        # compute angular error
        # arcsin domain is [-1, 1]; clamp to defend against floating-point noise.
        sin_arg   = clamp((left_dist - right_dist) / cfg.sonar_separation, -1.0, 1.0)
        angle_err = math.degrees(math.asin(sin_arg))
 
        # ── 5. Compute range error ───────────────────────────────────────────
        mean_dist = mean([left_dist, right_dist])
        range_err = mean_dist - cfg.target_dist_to_ledge
 
        angle_ok = abs(angle_err) <= cfg.alignment_tolerance / 2
        range_ok = abs(range_err) <= cfg.alignment_tolerance
 
        print(f"  angle_err={angle_err:.2f}°  range_err={range_err:.2f} cm  "
              f"angle_ok={angle_ok}  range_ok={range_ok}")
 
        if angle_ok and range_ok:
            creep.motor_stop()
            print(f"[square_to_ledge] aligned in {elapsed:.2f}s ✓")
            return True
 
        # correct angular error first and only when we are sure about it do we do the range correction
        if not angle_ok:
            turn_speed = cfg.alignment_turn_speed * sign(angle_err)
            # turn by the full angle error as the next iteration will re-measure.
            creep.turn_speed_angle(turn_speed, abs(angle_err))
 
        # correct the range error finally
        elif not range_ok:
            # drive forward (positive range_err = too far) or backward.
            drive_speed = cfg.alignment_drive_speed * sign(range_err)
            creep.drive_speed_distance(drive_speed, abs(range_err))
 
        # small pause to let the robot settle - is this necessary?????
        time.sleep(0.1)



def scan_for_box_on_ledge(
    creep: CreepRobot,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,) -> bool:
    """
    Search for a box on the ledge by wiggling left and right in front of it.
 
    The search pattern is:
        centre check → wiggle right → wiggle left → back to centre
    repeated up to *cfg.wiggle_retries* times.
 
    Returns True as soon as box_is_present() is detected
    Returns False if the box is never found within all retries
    """
    print(f"[scan_for_box_on_ledge] searching with "
          f"{cfg.wiggle_retries} retry cycles…")

 
    # always check the centre position first
    if box_is_present(creep, cfg):
        print("[scan_for_box_on_ledge] box found at centre position immediately")
        return True
 
    for attempt in range(cfg.wiggle_retries):
        print(f"  [scan attempt {attempt + 1}/{cfg.wiggle_retries}]")
 
        # WIGGLE RIGHT
        # We use drive_both to spin in place; drive_both does encoder reset
        # internally so we check the sensor in a timed loop.
        t0 = time.time()
        creep.drive_both(cfg.wiggle_speed, -cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration:
            if box_is_present(creep, cfg):
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found during right wiggle")
                return True
        creep.motor_stop()
 
        # WIGGLE LEFT (double duration to cross through centre)
        t0 = time.time()
        creep.drive_both(-cfg.wiggle_speed, cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration * 2:
            if box_is_present(creep, cfg):
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found during left wiggle")
                return True
        creep.motor_stop()
 
        # RETURN TO CENTRE
        t0 = time.time()
        creep.drive_both(cfg.wiggle_speed, -cfg.wiggle_speed)
        while time.time() - t0 < cfg.wiggle_duration:
            if box_is_present(creep, cfg):
                creep.motor_stop()
                print("[scan_for_box_on_ledge] box found returning to centre")
                return True
        creep.motor_stop()
 
        # ── re-check centre ──────────────────────────────────────────────────
        if box_is_present(creep, cfg):
            print("[scan_for_box_on_ledge] box found at centre after wiggle cycle")
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
    return max(min(number, 1), 0)


def collect_box_from_ledge(creep: CreepRobot, obj: Object, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    Master startpoint for all ledge related functions.

    Call this function once a ledged box has been found and needs to be collected.

    The robot will then retreat *cfg.retreat_distance* from the centre pillion, ready for further collection attempts.
    """
    if not navigate_to_ledge_position(creep, obj, cfg):
        return LedgePickupResult.NAV_FAILED
    
    if not square_to_ledge(creep, cfg):
        return LedgePickupResult.ALIGNMENT_TIMEOUT
    
    if not scan_for_box_on_ledge(creep, cfg):
        return LedgePickupResult.NO_BOX_FOUND
    
    if not grip_box(creep, cfg):
        return LedgePickupResult.GRIP_FAILED
    
    if not retract_with_box():
        return LedgePickupResult.GRIP_LOST_ON_RETRACT
    
    retreat_from_ledge()
    return LedgePickupResult.SUCCESS

    



