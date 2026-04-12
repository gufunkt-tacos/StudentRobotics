from ..machine import *
from creep.logic.basic_logic import *
import math
import time
from dataclasses import dataclass, field
from enum import Enum, auto
from statistics import mean
from .basic_logic import pick_up_box
import creep.logic.basic_strategy as bs



@dataclass
class LedgeConfig:
    """
    All tunable parameters for the ledge-pickup sequence in one place.
 
    Distances are in centimetres, angles in degrees, times in seconds.
    """
 
    # APPROACH/ALIGNMENT
 
    # How far from the ledge (measured by the front sonars) we want to stop
    target_dist_to_ledge: float = 25
    
    initial_dist_to_ledge: float = 1000

    target_distance_in_front_of_box: float = 10
 
    # Both sonars must agree to within this many cm before we call ourselves square to the ledge
    distance_alignment_tolerance: float = 5

    angle_alignment_tolerance: float = 2
 
    # Physical distance between the two front sonar sensors on the robot
    sonar_separation: float = 50
 
    # Drive / turn speed used during the alignment phase
    Kd = 0.8
    Ka = 0.5
 
    # Maximum wall-clock time we'll spend trying to square up before giving up.
    alignment_timeout: float = 10.0
 
    # How many sonar samples to average during alignment (more = slower but
    # steadier readings).
    sonar_samples: int = 1

    # ARM EXTENSION
 
    # Motor current threshold below which we consider the linear actuator to have reached its hard stop (fully extended or retracted).
    arm_stall_current: float = 0.01
 
    # Maximum time to wait for the arm to finish extending or retracting.
    arm_timeout: float = 10.0
 
    # GRIPPING
 
    # Box height (IR sensor reading) below which we assume the sucker is directly over a box surface.
    box_height_threshold: float = 10
 
    # How long to attempt to achieve suction before declaring grip failure
    grip_timeout: float = 8.0
 
    # How long the grip must remain stable before we trust it
    grip_confirmation_dwell: float = 0.5

 
    # WIGGLE SEACH
 
    # Speed used to rotate in place while scanning for the box
    wiggle_speed: int = 10
 
    # Duration of each wiggle half-swing.
    wiggle_duration: float = 0.1
 
    # Number of full left-right-centre cycles before giving up the box search
    wiggle_retries: int = 3
      
    wiggle_center_delay: float = 0.02 

 
    # RETREAT
 
    # How far to reverse after completing (or failing) a pickup attempt
    retreat_distance: float = 50.0
 
    # Speed at which to reverse.
    retreat_speed: int = -30
    
 
 
 

DEFAULT_LEDGE_CONFIG = LedgeConfig()


def sign(x):
    if(x>=0):
        return 1
    else:
        return -1


def find_closest_token(creep: CreepRobot, type: ObjectType, angle_offset: float = 0.0) -> Object | None:
    valid_tokens = creep.find_objects(type)
    if valid_tokens is not None and len(valid_tokens) > 0:
        closest_token = valid_tokens[0]
        for i in range(0,len(valid_tokens)):
            if(valid_tokens[i].position < closest_token.position):
                closest_token = valid_tokens[i]

        print("angle seen = " + str(closest_token.h_angle))

        closest_token.h_angle -= int(angle_offset)

        same_id_tokens = [token for token in valid_tokens if token.id == closest_token.id]
        virtual_token = Object(id=closest_token.id,position=mean([t.position for t in same_id_tokens]),h_angle=mean([t.h_angle for t in same_id_tokens]),v_angle=mean([t.v_angle for t in same_id_tokens]), yaw=mean([t.yaw for t in same_id_tokens]), pitch=mean([t.pitch for t in same_id_tokens]), roll=mean([t.roll for t in same_id_tokens]))
        return virtual_token
    else:
        return None
    
# def find_closest_token(creep: CreepRobot, type: ObjectType, angle_offset: float = 0.0) -> Object | None:
#     valid_tokens = creep.find_objects(type)
#     if valid_tokens is not None and len(valid_tokens) > 0:
#         closest_token = valid_tokens[0]
#         for i in range(0,len(valid_tokens)):
#             if(valid_tokens[i].position < closest_token.position):
#                 closest_token = valid_tokens[i]

#         print("angle seen = " + str(closest_token.h_angle))

#         closest_token.h_angle -= int(angle_offset)

#         # find_objects now returns one Object per physical cube, so same_id_tokens
#         # will almost always be a single element.  We keep the averaging for
#         # safety but, critically, also copy the pre-computed on_floor and
#         # has_top_face attributes so callers don't lose that information.
#         same_id_tokens = [token for token in valid_tokens if token.id == closest_token.id]
#         virtual_token = Object(
#             id       = closest_token.id,
#             position = mean([t.position for t in same_id_tokens]),
#             h_angle  = mean([t.h_angle  for t in same_id_tokens]),
#             v_angle  = mean([t.v_angle  for t in same_id_tokens]),
#             yaw      = mean([t.yaw      for t in same_id_tokens]),
#             pitch    = mean([t.pitch    for t in same_id_tokens]),
#             roll     = mean([t.roll     for t in same_id_tokens]),
#         )
#         # Preserve the floor-classification computed inside find_objects
#         virtual_token.on_floor     = closest_token.on_floor
#         virtual_token.has_top_face = closest_token.has_top_face
#         return virtual_token
#     else:
#         return None

    
def go_to_closest_token(creep: CreepRobot, type: ObjectType, closest_token: Object, open_doors: bool, approach_only: bool = False) -> bool:
    start_time = time.time()
    if not closest_token:
        return False

    # if closest_token.position > 82+13:
    #     creep.drive_speed_distance(40, 82)
    #     return


    scaling = 1
    creep.camera_pan(0)
    print(f"Time after camera pan: {time.time() - start_time:.2f} seconds")

    closest_token_dist = closest_token.position
    token_angle = closest_token.h_angle
   
    print(closest_token_dist)
    if not creep.turn_speed_angle(10*sign(token_angle), abs(token_angle)/scaling):
        return False
    
    print(f"Time after turn: {time.time() - start_time:.2f} seconds")

    # Try to find the token again after turning, to get another reading
    new_closest_token = find_closest_token(creep, type, 0)
    if new_closest_token is not None:
        closest_token_dist = new_closest_token.position
        token_angle = new_closest_token.h_angle 

    print(f"Time after finding token again: {time.time() - start_time:.2f} seconds")
    
    if not creep.turn_speed_angle(5*sign(token_angle), (abs(token_angle)/scaling)/2):
        return False

    if not creep.drive_speed_distance(40, closest_token_dist - 30):
        return False
    
    if open_doors:
        open_door_thread = Thread(target=creep.Doors_open)
        open_door_thread.start()
        if not creep.drive_speed_distance(40, 50):
            return False
        creep.Doors_close()
        creep.Doors_wedge()
        creep.floor_tokens_collected += 1
    else:
        creep.Doors_wedge()
        if not creep.drive_speed_distance(40, 50):
            return False

    return True

def get_in_position(creep: CreepRobot, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> None:
    """
    Final adjustments before collecting box from ledge (uses sonar)
    """
    TARGET_DIST = cfg.target_distance_in_front_of_box   # cm from wall
    DIST_TOL = cfg.distance_alignment_tolerance      # acceptable error (deg)
    ANGLE_TOL = cfg.angle_alignment_tolerance     # cm difference between sensors
    SONAR_SEPARATION = cfg.sonar_separation

    creep.Doors_close()

    while True:
        L = creep.left_front_sonar()
        R = creep.right_front_sonar()

        # distance
        dist_error = min(L, R) - TARGET_DIST

        # angle
        markers = creep.find_objects(ObjectType.ANY)
        current = None
        if markers is not None and len(markers) > 0:
            current = min(markers, key=lambda m: m.position)

        if current is None:
            angle_error = 0
        else:
            angle_error = SONAR_SEPARATION * math.tan(math.radians(current.h_angle))

        # check if done
        if abs(dist_error) < DIST_TOL and abs(angle_error) < ANGLE_TOL:
            creep.motor_stop()
            break

        # might need tuning
        Kd = 0.5   # distance gain
        Ka = 0.5   # angle gain

        speed = Kd * dist_error
        turn  = Ka * angle_error

        # convert to motor speeds
        speed1 = speed + turn
        speed2 = speed - turn

        # --- Clamp speeds ---
        speed1 = max(min(speed1, 10), -10)
        speed2 = max(min(speed2, 10), -10)

        creep.drive_both(int(speed1), int(speed2))
    

def collect_ledge_token(creep: CreepRobot):
#this is copied from keith; idk how it works AT ALL

    height = creep.sucker_height()
    print("height = ",height, " cms")
    while height > 10:
        height = creep.sucker_height()
        print("height = ",height, " cms")
        # extend arm
        print("extending arm")
        #robot.motor_boards["SR0VJ1K"].motors[1].power = -1.0
        creep.Arm_Extend(1)
    print("Token detected, stop arm extending")
    # robot.motor_boards["SR0VJ1K"].motors[1].power = 0  
    creep.Arm_Stop()  
    #vacuum on
    creep.VacValve("GRIP") # Allows suction
    creep.VacPump(1)
    time.sleep(1)
    #tilt arm down
    vacPressure = int(creep.robot.arduino.command("z"))
    creep.Arm_tilt_down()
    while (vacPressure > 50):
        vacPressure = int(creep.robot.arduino.command("z"))
        print("Vac Pressure = ",vacPressure)
    time.sleep(.5)   
    #tilt arm up
    creep.Arm_tilt_up()
    time.sleep(.5)
    #retract arm
    #robot.motor_boards["SR0VJ1K"].motors[1].power = 1.0
    creep.Arm_Retract(1) # full speed. Takes around 7 seconds 
    time.sleep(7)
    
    #vacuum off
    #robot.motor_boards["SR0VJ1K"].motors[0].power = 0
    creep.VacValve("VENT") # releases vacuum
    creep.VacPump(0)

    creep.Arm_tilt_up()
    height = creep.sucker_height()
    print("height = ",height, " cms")
    while height < 10:
        height = creep.sucker_height()
        print("height = ",height, " cms")
        # extend arm
        print("extending arm")
        #robot.motor_boards["SR0VJ1K"].motors[1].power = -1.0
        creep.Arm_Extend(1)
    creep.Arm_tilt_down()
    creep.Arm_Retract(1)    
    time.sleep(3)

def get_ledge_token(creep: CreepRobot, type: ObjectType, angle_to_ledge: int):
    #turn towards platform
    creep.turn_speed_angle(16 * sign(angle_to_ledge), abs(angle_to_ledge))
    #close door so we can get closer
    creep.Doors_close()
    #reverse away from platform - why do we do this?
    # creep.drive_speed_distance(-16,70)

    closest_token = find_closest_token(creep, type, 0)
    if closest_token is None:
        return False
    closest_token_dist = closest_token.position
    token_angle = closest_token.h_angle

    print(closest_token_dist)
    creep.turn_speed_angle(10*sign(token_angle), abs(token_angle))

    # Try to find the token again after turning, to get another reading
    new_closest_token = find_closest_token(creep, type, 0)
    if new_closest_token is not None:
        closest_token_dist = new_closest_token.position
        new_token_angle = new_closest_token.h_angle
    else:
        closest_token_dist = 0
        new_token_angle = 0
    
    creep.turn_speed_angle(5*sign(new_token_angle), abs(new_token_angle)/2)

    creep.drive_speed_distance_objchk(24,120, 40)
    creep.motor_stop()
    # collect_ledge_token(creep)

    # token_to_square = new_closest_token if closest_token is None else closest_token
    # if token_to_square is not None:
    #     square_to_ledge(creep, token_to_square)
    # else:
    #     square_to_ledge(creep, None)

    get_in_position(creep)
    if pick_up_box(creep):
        creep.ledge_tokens_collected += 1


    # add clearance for doors
    creep.drive_speed_distance(-16, 10)
    creep.Doors_wedge()

    creep.drive_speed_distance_objchk(-16,50,20)
    creep.turn_speed_angle(-16*sign(angle_to_ledge),abs(angle_to_ledge))

def recovery(creep: CreepRobot) -> None:
    """
    Sonar-based recovery sequence.

    Phase 1 — Unstick
    ------------------
    Reads the two front sonars and the rear sonar to decide how the robot
    is trapped, then manoeuvres to free itself:

      Front + rear blocked  → spin in 90° steps until a clear gap appears,
                              then drive through it.
      Front only blocked    → reverse away, turn toward the side with more
                              clearance, drive forward.
      Rear only blocked     → drive forward to escape whatever is behind,
                              then turn 45° to change heading.
      Nothing obviously     → robot is probably just lost (no wall contact);
        blocked               nudge forward and reorient 45° to break out of
                              any invisible loop.

    Phase 2 — Post-recovery decision
    ----------------------------------
    Uses the token inventory to choose the safest next action:

      floor_tokens ≥ 1     → go home, deposit all boxes, reset the inventory,
                              turn back into the arena and keep searching.
      ledge_tokens ≥ 1     → go home and stay there (ledge tokens are
      (floor == 0)           valuable but fragile to carry; don't risk losing
                              them chasing more).
      no tokens at all     → just keep searching — nothing to lose.
    """

    STUCK_CM   = 30   # sonar reading below this means "blocked by a wall/obstacle"
    CLEAR_CM   = 60   # sonar reading above this counts as a usable gap
    BACK_DIST  = 45   # cm to reverse when front is blocked
    FWD_DIST   = 45   # cm to drive forward when rear is blocked or after a spin

    print("[recovery] reading sonars…")
    L    = creep.left_front_sonar()
    R    = creep.right_front_sonar()
    rear = creep.rear_sonar()

    front_blocked = L < STUCK_CM or R < STUCK_CM
    rear_blocked  = rear < STUCK_CM

    print(f"[recovery] L={L:.0f}cm  R={R:.0f}cm  rear={rear:.0f}cm  "
          f"front_blocked={front_blocked}  rear_blocked={rear_blocked}")


    if front_blocked and rear_blocked:
        # Completely boxed in — spin 90° at a time looking for a gap
        print("[recovery] boxed in — spinning to find gap")
        freed = False
        for _ in range(4):
            creep.turn_speed_angle(16, 90)
            L = creep.left_front_sonar()
            R = creep.right_front_sonar()
            if L > CLEAR_CM and R > CLEAR_CM:
                print("[recovery] gap found — driving through")
                creep.drive_speed_distance(20, FWD_DIST)
                freed = True
                break
        if not freed:
            # Last resort: drive toward whichever front sonar had most clearance
            print("[recovery] no clean gap found — forcing through clearest direction")
            if L > R:
                creep.turn_speed_angle(-16, 45)   # lean left
            else:
                creep.turn_speed_angle(16, 45)    # lean right
            creep.drive_speed_distance(20, FWD_DIST)

    elif front_blocked:
        # Back up, then turn toward the side with more clearance
        print(f"[recovery] front blocked (L={L:.0f}, R={R:.0f}) — reversing then turning")
        creep.drive_speed_distance(-20, BACK_DIST)
        if L > R:
            # More room on the left → turn left (negative = anti-clockwise)
            print("[recovery] more clearance on left — turning left")
            creep.turn_speed_angle(-16, 60)
        else:
            print("[recovery] more clearance on right — turning right")
            creep.turn_speed_angle(16, 60)
        creep.drive_speed_distance(20, FWD_DIST)

    elif rear_blocked:
        # Something caught behind us — drive forward to escape it
        print(f"[recovery] rear blocked (rear={rear:.0f}) — driving forward")
        creep.drive_speed_distance(20, FWD_DIST)
        creep.turn_speed_angle(16, 45)

    else:
        # Robot isn't touching walls it's lost or spinning in place
        print("[recovery] no walls detected — robot is lost, nudging and reorienting")
        creep.drive_speed_distance(20, 40)
        creep.turn_speed_angle(16, 45)

    # ── Phase 2: Post-recovery decision ────────────────────────────────────

    floor_count = getattr(creep, "floor_tokens_collected", 0)
    ledge_count = getattr(creep, "ledge_tokens_collected", 0)

    print(f"[recovery] inventory: floor={floor_count}  ledge={ledge_count}")

    if floor_count >= 1:
        # We're carrying floor boxes — go home, drop them off, then come back
        print("[recovery] carrying floor token(s) — depositing then resuming search")
        deposit_and_resume(creep)

    elif ledge_count >= 1:
        # Only a ledge (upper) token on board — valuable but fragile; stay home
        print("[recovery] carrying ledge token only — going home to stay safe")
        raise Exception("GO HOME NOW WE NEED THIS 1 LEDGE TOKEN") 
        # Don't recurse back into strategy; let robot.py finally-block handle exit

    else:
        # Nothing collected yet — just keep searching
        print("[recovery] no tokens collected — resuming search")
        return bs.strategy_base(creep)


def deposit_and_resume(creep: CreepRobot) -> None:
    """
    Drive home, deposit whatever boxes are wedged in the front of the robot,
    then turn around and hand back to bsearch for another collection run.
    """
    if not go_home_norm(creep):
        print("[deposit] could not reach home — aborting deposit")
        return

    # Drop boxes into the lab zone: open doors, reverse out so they stay inside
    print("[deposit] opening doors and reversing to release boxes")
    creep.Doors_open()
    creep.drive_speed_distance(-30, 70)
    creep.Doors_wedge()

    # Reset the inventory — boxes are now in the lab
    creep.floor_tokens_collected = 0
    print("[deposit] inventory reset — resuming search")

    # Turn 180° to face back into the arena and carry on
    creep.turn_speed_angle(16, 180)
    bs.strategy_base(creep)


def get_floor_token(creep: CreepRobot, type: ObjectType) -> bool:
    search_time = time.time()
    # Find a token in front of the robot
    closest_token = find_closest_token(creep, type)
    print(f"Initial token search took {time.time() - search_time:.2f} seconds")
    if closest_token:
        return go_to_closest_token(creep, type, closest_token, True)
        
    
    # If no token is found, pan the camera to the left and right to try and find one
    angle_offset = 45
    creep.camera_pan(angle_offset)
    closest_token = find_closest_token(creep, type, angle_offset)

    if closest_token:
        return go_to_closest_token(creep, type, closest_token, True)
        
    
    # If still no token is found, pan the camera to the right and try again
    angle_offset = -45
    creep.camera_pan(angle_offset)
    closest_token = find_closest_token(creep, type, angle_offset)

    if closest_token:
        return go_to_closest_token(creep, type, closest_token, True)
        

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
    
def get_normal_to_token(creep: CreepRobot, obj: Object, distance_away: float) -> tuple[float, float, float]:
    """
    This function returns the position and horizontal angle a set distance away from the token.
    
    Args:
        creep: CreepRobot instance
        obj: Object to calculate normal from
        distance_away: Distance to maintain from the object
        
    Returns:
        tuple[float, float, float]: (distance_to_move, angle_to_move, final_turn)
    """
    r = distance_away 
    d = obj.position
    y = obj.yaw
    h = obj.h_angle

    # apply cosine rule to get distance
    distance_to_move = math.sqrt(d**2 + r**2 - 2 * d * r * math.cos(math.radians(y)))

    # apply sine rule to get angle (signed)
    angle_to_move = math.degrees(math.asin(math.sin(math.radians(y)) * r / distance_to_move)) + h

    final_turn = (h - angle_to_move) - y

    return (distance_to_move, -angle_to_move, -final_turn)

def get_normal_to_ledge(creep: CreepRobot, obj: Object, distance_away: float, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> tuple[float, float, float] | None:
    """
    This function returns the position and horizontal angle a set distance away from the ledge.
    Note that this is not accurate as it uses the sonars and needs to be checked as it can RETURN NONE!
    
    Args:
        creep: CreepRobot instance
        obj: Object to calculate normal from
        distance_away: Distance to maintain from the object
        
    Returns:
        tuple[float, float, float]: (distance_to_move, angle_to_move, final_turn)
        or None
    """ 
    SONAR_SEPARATION = cfg.sonar_separation
    
    L = creep.left_front_sonar()
    R = creep.right_front_sonar()
    spread = L - R
    
    if spread >= cfg.sonar_separation:
      return None

    r = distance_away 
    d = obj.position if obj is not None else (L + R) / 2
    y = math.degrees(math.atan2(spread, SONAR_SEPARATION))
    h = obj.h_angle if obj is not None else 0
 
    # apply cosine rule to get distance
    distance_to_move = math.sqrt(d**2 + r**2 - 2 * d * r * math.cos(math.radians(h - y)))

    # apply sine rule to get angle (signed)
    angle_to_move = h - math.degrees(math.asin(math.sin(math.radians(y)) * r / distance_to_move))

    final_turn = (h - angle_to_move) - y

    return (distance_to_move, angle_to_move, final_turn)

def is_on_floor(creep: CreepRobot, obj: Object) -> bool:
    """
    Determine if a detected object is on the floor vs. on an elevated surface.
    
    Uses the object's vertical angle and distance to estimate its height.
    Objects below the threshold are considered floor tokens.
    """
    THRESHOLD = 5
    CAMERA_HEIGHT = 44.5

    d = obj.position
    a = obj.v_angle

    d_slant = math.sqrt(obj.position**2 + CAMERA_HEIGHT**2)

    h = math.sin(math.radians(a)) * d_slant + CAMERA_HEIGHT

    if h >= THRESHOLD:
        creep.LED_A_blue()
        return False

    creep.LED_B_green()
    return True

# def is_on_floor(creep: CreepRobot, obj: Object) -> bool:
#     """
#     Return True if the object is on the arena floor rather than on the raised
#     central platform / ledge.

#     Uses the ``on_floor`` attribute set by ``find_objects()`` (which averages
#     side-face readings and excludes the unreliable top face before estimating
#     height).  Falls back to the original geometric formula for any Object that
#     was constructed without going through ``find_objects()``.
#     """
#     # find_objects sets on_floor from a proper multi-face averaged height.
#     # The attribute defaults to False in __init__, so we can trust it if the
#     # position is non-zero (a zero position means the Object was never
#     # populated by a real camera reading).
#     result = obj.on_floor
#     if result:
#         creep.LED_B_green()
#     else:
#         creep.LED_A_blue()
#     return result


def find_home_marker(creep: CreepRobot) -> Object | None:
    """
    Try to find the closest home marker by panning the camera, then rotating.
    Returns the closest home marker Object or None if not found.
    """
    # Pan left, centre, right before moving
    for pan_angle in [0, 30, -30]:
        creep.camera_pan(pan_angle)
        time.sleep(1)
        markers = creep.find_objects(ObjectType.ARENA_MARKER)
        print(str(markers))
        if markers:
            home_markers = [m for m in markers if m.id in creep.my_lab]
            if home_markers:
                creep.camera_pan(0)
                if pan_angle != 0:
                    creep.turn_speed_angle(-16 * sign(pan_angle), abs(pan_angle))
                    markers = creep.find_objects(ObjectType.ARENA_MARKER)
                    if markers:
                        home_markers = [m for m in markers if m.id in creep.my_lab]
                        if not home_markers:
                            creep.drive_speed_distance(-16, 50)
                            return find_home_marker(creep)
                    else:
                        return find_home_marker(creep)
                return min(home_markers, key=lambda m: m.position)

    creep.camera_pan(0)

    # Rotate in place in 45° steps, full 360°
    for _ in range(5):
        creep.turn_speed_angle(16, 60)
        for pan_angle in [0, 30, -30]:
            creep.camera_pan(pan_angle)
            time.sleep(1)
            markers = creep.find_objects(ObjectType.ARENA_MARKER)
            print(str(markers))
            if markers:
                home_markers = [m for m in markers if m.id in creep.my_lab]
                if home_markers:
                    creep.camera_pan(0)
                    if pan_angle != 0:
                        creep.turn_speed_angle(-16 * sign(pan_angle), abs(pan_angle))
                        markers = creep.find_objects(ObjectType.ARENA_MARKER)
                        if markers:
                            home_markers = [m for m in markers if m.id in creep.my_lab]
                            if not home_markers:
                                creep.drive_speed_distance(-16, 50)
                                return find_home_marker(creep)
                        else:
                            return find_home_marker(creep)
                    return min(home_markers, key=lambda m: m.position)

    creep.camera_pan(0)
    return None


def search_for_boxes(
    creep: CreepRobot,
    box_type: ObjectType = ObjectType.BASE,
    sweep_step_deg: int   = 30
) -> Object | None:

    print(f"[search_for_boxes] starting 360° sweep (step={sweep_step_deg}°)")

    best_any:    Object | None = None   # closest token of any height

    steps = 360 // sweep_step_deg

    for step_idx in range(steps):
        for pan_angle in (0, 30, -30):
            creep.camera_pan(pan_angle)
            candidates = creep.find_objects(box_type)
            if not candidates:
                continue

            for obj in candidates:
                if best_any is None or obj.position < best_any.position:
                    best_any = obj

        # Stop early if we have a floor token (preferred)
        if best_any is not None:
            break

        # Rotate to next sweep position
        if step_idx < steps - 1:
            creep.turn_speed_angle(16, sweep_step_deg)

    creep.camera_pan(0)


    if best_any is not None:
        print(f"[search_for_boxes] no floor token; returning nearest token "
              f"id={best_any.id} dist={best_any.position:.0f}cm")
        return best_any
    
    
    return None

def read_front_sonars(creep: CreepRobot) -> tuple[float, float]:
    """Return (left_cm, right_cm) front sonar readings."""
    return creep.left_front_sonar(), creep.right_front_sonar()

def go_home_norm(creep: CreepRobot, norm_dist: float = 20.0) -> bool:
    """
    Navigate to a position norm_dist cm in front of the closest home marker,
    perpendicular to the wall.

    Returns True if successfully reached the home normal, False if lost.
    """
    creep.Doors_wedge()

    # --- Attempt 1: find home marker directly ---
    home_marker = find_home_marker(creep)

    while home_marker is None:
        # --- Attempt 2: navigate to a nearby arena marker and look again ---
        print("[go_home] home not visible, navigating to intermediate vantage point")
        if not navigate_via_arena_marker(creep, norm_dist):
            print("[go_home] no arena markers visible at all, giving up")
            return False

        home_marker = find_home_marker(creep)

    # --- Drive to the normal point in front of home ---
    print(f"[go_home] found home marker {home_marker.id} at {home_marker.position:.1f} cm")
    dist, angle, final_turn = get_normal_to_token(creep, home_marker, norm_dist)

    creep.turn_speed_angle(16 * sign(angle), abs(angle))
    creep.drive_speed_distance(30, dist)
    creep.drive_speed_distance(30, 50)
    creep.turn_speed_angle(16 * sign(final_turn), abs(final_turn))

    print("[go_home] reached home position")
    return True


def navigate_via_arena_marker(creep: CreepRobot, norm_dist: float) -> bool:
    """
    Check if any preferred markers are visible (sorted by proximity to home).
    If yes, go to the best one's normal facing the wall.
    If no, go to any other visible marker's normal facing 90 degrees to it.
    """
    markers = creep.find_objects(ObjectType.ARENA_MARKER)

    if markers:
        visible_ids = {m.id: m for m in markers}

        # --- Preferred path: best visible marker from precomputed ranking ---
        best = None
        for marker_id in creep.preferred_nav_markers:
            if marker_id in visible_ids:
                best = visible_ids[marker_id]
                break

        if best is not None:
            print(f"[navigate_via_arena_marker] using preferred marker {best.id}")
            dist, angle, final_turn = get_normal_to_token(creep, best, norm_dist)
            creep.turn_speed_angle(16 * sign(angle), abs(angle))
            creep.drive_speed_distance(30, dist)
            creep.turn_speed_angle(16 * sign(final_turn), abs(final_turn))
            return True

        # --- Fallback: any non-home visible marker, stop 90 degrees to its normal ---
        non_preferred = [m for m in markers if m.id not in creep.preferred_nav_markers
                                            and m.id not in creep.my_lab]
        if non_preferred:
            fallback = min(non_preferred, key=lambda m: m.position)
            print(f"[navigate_via_arena_marker] fallback marker {fallback.id} — stopping 90° to normal")
            dist, angle, final_turn = get_normal_to_token(creep, fallback, norm_dist)
            creep.turn_speed_angle(16 * sign(angle), abs(angle))
            creep.drive_speed_distance(30, dist)
            # Turn to face 90 degrees to the wall normal instead of towards it
            # This orients the robot along the wall, looking across the arena towards home
            sideways_turn = final_turn - 90
            creep.turn_speed_angle(16 * sign(sideways_turn), abs(sideways_turn))
            return True

    print("[navigate_via_arena_marker] no markers visible at all")
    return False


def go_home(creep: CreepRobot):
    
    creep.Doors_wedge()
    closest_token_dist = 8192
    closest_token_angle = 361
    marker_found = False

    arena_markers = creep.find_objects(ObjectType.ARENA_MARKER)
    if (arena_markers):
        for i in range(len(arena_markers)):
            for j in range(len(creep.my_lab)):
                print(f"current marker = {arena_markers[i].id} lab token checking for = {creep.my_lab[j]}")
                if(arena_markers[i].id == creep.my_lab[j]):
                    marker_found = True
                    if(arena_markers[i].position < closest_token_dist):
                        closest_token_dist = arena_markers[i].position
                        closest_token_angle = arena_markers[i].h_angle
            print(str(arena_markers[i]))
    

    if marker_found:
        print("moving" + str(closest_token_dist))
        creep.turn_speed_angle(5, closest_token_angle)
        creep.drive_speed_distance(30, closest_token_dist)
        return

    # If no arena marker is found, try to find the next one in the sequence
    for step in range(1,10):
        if (arena_markers):
            for i in range(len(arena_markers)):
                if(arena_markers[i].id == next_arena_token(creep.my_lab[0],step,-1) or arena_markers[i].id == next_arena_token(creep.my_lab[2],step,1)):
                    marker_found = True
                    if(arena_markers[i].position < closest_token_dist):
                        marker_id = arena_markers[i].id
                        closest_token_dist = arena_markers[i].position
                        closest_token_angle = arena_markers[i].h_angle
    
    if marker_found:
        
        marker_in_corner = False

        for marker in creep.corner_markers:
            if marker == marker_id:
                marker_in_corner = True


        marker_coords = get_marker_coords(marker_id)
        lab_wall = get_marker_wall(creep.my_lab[2])
        wall_facing = get_marker_wall(marker_id)
        wall_zero = lab_wall
        wall_one = (lab_wall + 1)%4
        wall_two = (lab_wall + 2)%4
        wall_three = (lab_wall + 3)%4
        print(lab_wall)
        print(wall_zero, wall_one, wall_two, wall_three)
        print(marker_id)
        print(get_marker_wall(marker_id))
        print("wall facing: " + str(wall_facing))
        print("distance: " + str(closest_token_dist))

        if(wall_facing == wall_zero):
             print("turning from wall zero")
             if (closest_token_dist>75 and marker_in_corner == False):
                creep.turn_speed_angle(16,180)
                print("turning 180deg")
             else:
                creep.drive_speed_distance(32, closest_token_dist/1)
                creep.turn_speed_angle(-16, 90)
                
        elif(wall_facing == wall_one):
            print("turning from wall one")
            if (closest_token_dist>75 and marker_in_corner == False):
                creep.turn_speed_angle(16,180)
                print("turning 180deg")
            else:
                creep.turn_speed_angle(-16, 90)
            
        elif(wall_facing == wall_two):
            print("turning from wall two")
            if (closest_token_dist>75 and marker_in_corner == False):
                creep.turn_speed_angle(16,180)
                print("turning 180deg")
            else:
                creep.turn_speed_angle(16, 90)
        elif(wall_facing == wall_three):
            print("turning from wall three")
            if (closest_token_dist>75 and marker_in_corner == False):
                creep.turn_speed_angle(16,180)
                print("turning 180deg")
            else:
                creep.drive_speed_distance(32, closest_token_dist/1)
                creep.turn_speed_angle(16, 90)


        else:
            print("can't find which wall i'm at") #should never reach this   
        
    else:
        print("no token found")
        creep.turn_speed_angle(16,45) # if it can't see anything, turn
    
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
            creep.turn_speed_angle(16*sign(right_sonar-left_sonar),90)
            creep.drive_speed_distance(20,50)
            creep.turn_speed_angle(16*-sign(right_sonar-left_sonar),90)
            
        



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

def navigate_to_initial_ledge_position(
    creep: CreepRobot,
    obj: Object,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    Drive from the robot's current position to a point that is near enough where sonar might be working
    """
    
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


def square_to_ledge(
    creep: CreepRobot,
    obj: Object,
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    """
    creep.Doors_close()
    result = get_normal_to_ledge(creep, obj, cfg.target_dist_to_ledge)
    if result is None and obj is None:
        return False
    if result is None:
        result = get_normal_to_token(creep, obj, cfg.target_dist_to_ledge)
   
    distance_to_move = result[0]

    angle_to_move = result[1]
    
    final_turn_angle = result[2]


    creep.turn_speed_angle(16 * sign(angle_to_move), abs(angle_to_move))
    creep.drive_speed_distance(30 , distance_to_move)
    creep.turn_speed_angle(16 * sign(angle_to_move), abs(final_turn_angle))

    get_in_position(creep)
    return True




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
    creep.Arm_Extend(1)
    
    while has_arm_finished(creep) == False:
      if box_is_present(creep, cfg):
          print("[scan_for_box_on_ledge] box found at centre immediately")
          creep.Arm_Extend(0)
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
 
    if confirm_grip(creep):
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
    cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG,):
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
    creep.VacValve("VENT")
    creep.VacPump(0)
    
    # pull box back in
    creep.Arm_tilt_up()
    while not has_arm_finished(creep) and box_is_present(creep):
        creep.Arm_Extend(1)
    creep.Arm_tilt_down()
    creep.Arm_Retract(1)
    
 
    
    

def has_arm_finished(creep: CreepRobot, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> bool:
    """
    Return True when the arm linear actuator has reached its hard stop.
 
    The SR motor board reports the current through the motor.  When the
    actuator hits a mechanical limit the motor stalls and its current drops
    toward zero.  We use a threshold slightly above zero to account for
    measurement noise.
    """
    current = creep.robot.motor_boards["SR0VJ1K"].motors[1].current
    print(current)
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


def collect_box_from_ledge(creep: CreepRobot, obj: Object | None, cfg: LedgeConfig = DEFAULT_LEDGE_CONFIG) -> LedgePickupResult:
    """
    Master entry-point for all ledge related functions.

    Call this function once a ledged box has been found and needs to be collected.

    The robot will then retreat *cfg.retreat_distance* from the centre pillion, ready for further collection attempts.
    """
    if obj is None:
        return LedgePickupResult.NAV_FAILED

    if not navigate_to_initial_ledge_position(creep, obj, cfg):
        return LedgePickupResult.NAV_FAILED
    # square_to_ledge now handles camera → sonar → wiggle internally.
    # Returns True only when the box is found and the arm is centred.
    if not square_to_ledge(creep, obj):
        return LedgePickupResult.ALIGNMENT_TIMEOUT      # ← was ALIGNMENT_TIMEOUT
    # ← REMOVE the scan_for_box_on_ledge call that was here
    if not scan_for_box_on_ledge(creep, cfg):
        return LedgePickupResult.NO_BOX_FOUND
    if not pick_up_box(creep):
        return LedgePickupResult.GRIP_FAILED
    # if not retract_with_box(creep, cfg):          =
    #     return LedgePickupResult.GRIP_LOST_ON_RETRACT
    retreat_from_ledge(creep, cfg)                 # ← bug 4 fixed
    print("Box successfully collected from ledge.")
    return LedgePickupResult.SUCCESS




# ── Tunable constants ─────────────────────────────────────────────────────────

WALL_STANDOFF         : float = 80.0   # cm to cruise at from the wall
SCAN_STEP             : float = 50.0   # cm per driving segment between scans
MAX_CARRY             : int   = 3      # go home after this many floor tokens
HOME_BUFFER           : float = 30.0  # seconds before go_home_time to head back
SONAR_WARN_CM         : float = 35.0  # soft obstacle threshold (cm)
CUBE_APPROACH_CLEARANCE: float = 30.0 # stop this far short when approaching cube

corner_ids = [0, 19, 14, 15, 10, 9, 4, 5]

def time_remaining(creep: CreepRobot) -> float:
    """Seconds left before the robot should head home."""
    elapsed = time.time() - creep.time_started_game
    return creep.go_home_time - elapsed


def should_go_home(creep: CreepRobot) -> bool:
    """True if it's time to head home (time-based or carrying enough cubes)."""
    floor_count = getattr(creep, "floor_tokens_collected", 0)
    if floor_count >= MAX_CARRY:
        print(f"[circler] carrying {floor_count} tokens — heading home")
        return True
    if time_remaining(creep) <= HOME_BUFFER:
        print(f"[circler] only {time_remaining(creep):.1f}s left — heading home")
        return True
    return False


def find_floor_base_cubes(creep: CreepRobot) -> list[Object]:
    """Return all visible BASE cubes classified as on_floor=True, sorted by distance."""
    creep.camera_pan(0)
    all_base = creep.find_objects(ObjectType.BASE)
    if not all_base:
        return []
    floor_cubes = [o for o in all_base if o.on_floor]
    return sorted(floor_cubes, key=lambda o: o.position)


def sonar_obstacle_ahead(creep: CreepRobot) -> bool:
    """Soft pre-step sonar check — returns True if anything is within SONAR_WARN_CM."""
    L = creep.left_front_sonar()
    R = creep.right_front_sonar()
    blocked = L < SONAR_WARN_CM or R < SONAR_WARN_CM
    if blocked:
        print(f"[circler] soft obstacle: L={L:.0f}cm R={R:.0f}cm")
    return blocked


def align_to_wall(creep: CreepRobot) -> bool:
    """
    Find the nearest arena marker, drive to WALL_STANDOFF cm in front of it
    (perpendicular, facing the wall), then turn 90° clockwise so the wall is
    on the left.

    Returns True on success, False if no arena marker is visible.
    """
    creep.Doors_wedge()
    candidates = []

    # Pan left, centre, right to maximise chance of seeing a marker
    best_marker: Object | None = None
    for pan in (0, 30, -30):
        creep.camera_pan(pan)
        markers = creep.find_objects(ObjectType.ARENA_MARKER)
        if markers:
            # Pick the closest one

            for marker in markers:
                for corner_id in corner_ids:
                    if marker.id == corner_id:
                        candidates.append(marker)
            
            if candidates:
                candidate = min(candidates, key=lambda m: m.position)
            else:
                candidate = min(markers, key=lambda m: m.position) 

            if best_marker is None or candidate.position < best_marker.position:
                best_marker = candidate
    creep.camera_pan(0)

    if best_marker is None:
        print("[circler] no arena marker visible — cannot align to wall")
        return False

    print(f"[circler] aligning to marker {best_marker.id} "
          f"(wall {get_marker_wall(best_marker.id)}) "
          f"at {best_marker.position:.0f}cm, h={best_marker.h_angle:.1f}°")

    # Drive to the normal point WALL_STANDOFF cm in front of the marker
    dist, angle, final_turn = get_normal_to_token(creep, best_marker, WALL_STANDOFF)

    creep.turn_speed_angle(sign(angle) * 16, abs(angle))
    creep.drive_speed_distance(30, dist)
    # final_turn orients the robot perpendicular to the wall (facing the wall)
    creep.turn_speed_angle(sign(final_turn) * 16, abs(final_turn))

    # Now the robot faces the wall.  Turn 90° clockwise to travel along it.
    # Clockwise from above = positive turn direction in the motor convention.
    print("[circler] turning 90° clockwise along wall")
    creep.turn_speed_angle(16, 90)

    return True


def collect_cube(creep: CreepRobot, cube: Object) -> bool:
    """
    Approach and collect a floor cube.  Uses the two-pass alignment in
    go_to_closest_token (turn, re-read, drive, open doors, close doors).

    Returns True if the cube was successfully collected.
    """
    print(f"[circler] collecting cube id={cube.id} "
          f"at {cube.position:.0f}cm h={cube.h_angle:.1f}°")


    return go_to_closest_token(creep, ObjectType.BASE, cube, open_doors=True)


def deposit_and_realign(creep: CreepRobot) -> None:
    """
    Go home, drop boxes into the lab, reset inventory, then realign to a wall
    marker ready for another circling pass.
    """
    print("[circler] going home to deposit")
    if not go_home_norm(creep):
        print("[circler] could not reach home — continuing without depositing")
        return

    # Drop boxes: open doors, reverse so boxes stay in the lab zone
    creep.Doors_open()
    creep.drive_speed_distance(-30, 70)
    creep.Doors_wedge()

    creep.floor_tokens_collected = 0
    print("[circler] deposit complete — resuming circuit")

    # Turn back into the arena (180°) and realign to the nearest wall
    creep.turn_speed_angle(16, 180)
    align_to_wall(creep)
