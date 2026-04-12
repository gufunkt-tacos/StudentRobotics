"""
circler.py — Arena-circling floor-cube collection strategy
==========================================================

Behaviour summary
-----------------
1. Grab the first floor-level BASE cube visible at startup.
2. Find the nearest arena wall marker, position WALL_STANDOFF cm in front of
   it (perpendicular to the wall), then turn 90° clockwise so the wall is on
   the robot's left.  This gives a consistent clockwise traversal:
       wall 0 (top)  → wall 1 (right) → wall 2 (bottom) → wall 3 (left) → …
3. Drive SCAN_STEP cm at a time, checking sonars for obstacles and calling
   find_objects() after each step.  Objects with on_floor=True are targeted.
4. On finding a floor BASE cube: approach and collect it (open doors), then
   realign to the nearest wall marker and continue circling.
5. Go home and deposit when carrying MAX_CARRY cubes or when fewer than
   HOME_BUFFER seconds remain.  After depositing, resume circling.

Collision avoidance
-------------------
The existing drive_speed_distance() already calls recovery() when the
machine's collision flags are set (sonar threshold ~10 cm).  circler adds an
extra *soft* pre-check at SONAR_WARN_CM before each step so the robot slows
and steers around obstacles without waiting for a hard collision.

Floor-vs-ledge filtering
-------------------------
Uses the new Object.on_floor attribute (set by the updated find_objects()).
Only cubes with on_floor=True are targeted; ledge cubes are ignored so the
circling run does not accidentally drive into the raised platform.

Tunable constants — adjust to match your arena / robot
-------------------------------------------------------
  WALL_STANDOFF   : cruising distance from the wall (cm)
  SCAN_STEP       : distance between successive scans (cm)
  MAX_CARRY       : maximum floor tokens before depositing
  HOME_BUFFER     : seconds before go_home_time to trigger return
  SONAR_WARN_CM   : soft sonar threshold for pre-step obstacle check
  CUBE_APPROACH_CLEARANCE : stop this far short when approaching a cube (cm)
"""

from creep.machine import CreepRobot, ObjectType, Object, get_marker_wall
import creep.logic.robot_subroutines as rs
import time
import math

# ── Tunable constants ─────────────────────────────────────────────────────────

WALL_STANDOFF         : float = 80.0   # cm to cruise at from the wall
SCAN_STEP             : float = 50.0   # cm per driving segment between scans
MAX_CARRY             : int   = 3      # go home after this many floor tokens
HOME_BUFFER           : float = 30.0  # seconds before go_home_time to head back
SONAR_WARN_CM         : float = 35.0  # soft obstacle threshold (cm)
CUBE_APPROACH_CLEARANCE: float = 30.0 # stop this far short when approaching cube

corner_ids = [0, 19, 14, 15, 10, 9, 4, 5]

# ── Internal helpers ──────────────────────────────────────────────────────────

def _time_remaining(creep: CreepRobot) -> float:
    """Seconds left before the robot should head home."""
    elapsed = time.time() - creep.time_started_game
    return creep.go_home_time - elapsed


def _should_go_home(creep: CreepRobot) -> bool:
    """True if it's time to head home (time-based or carrying enough cubes)."""
    floor_count = getattr(creep, "floor_tokens_collected", 0)
    if floor_count >= MAX_CARRY:
        print(f"[circler] carrying {floor_count} tokens — heading home")
        return True
    if _time_remaining(creep) <= HOME_BUFFER:
        print(f"[circler] only {_time_remaining(creep):.1f}s left — heading home")
        return True
    return False


def _find_floor_base_cubes(creep: CreepRobot) -> list[Object]:
    """Return all visible BASE cubes classified as on_floor=True, sorted by distance."""
    creep.camera_pan(0)
    all_base = creep.find_objects(ObjectType.BASE)
    if not all_base:
        return []
    floor_cubes = [o for o in all_base if o.on_floor]
    return sorted(floor_cubes, key=lambda o: o.position)


def _sonar_obstacle_ahead(creep: CreepRobot) -> bool:
    """Soft pre-step sonar check — returns True if anything is within SONAR_WARN_CM."""
    L = creep.left_front_sonar()
    R = creep.right_front_sonar()
    blocked = L < SONAR_WARN_CM or R < SONAR_WARN_CM
    if blocked:
        print(f"[circler] soft obstacle: L={L:.0f}cm R={R:.0f}cm")
    return blocked


def _align_to_wall(creep: CreepRobot) -> bool:
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
            
            candidate = min(candidates, key=lambda m: m.position)

            if candidate is None:
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
    dist, angle, final_turn = rs.get_normal_to_token(creep, best_marker, WALL_STANDOFF)

    creep.turn_speed_angle(rs.sign(angle) * 16, abs(angle))
    creep.drive_speed_distance(30, dist)
    # final_turn orients the robot perpendicular to the wall (facing the wall)
    creep.turn_speed_angle(rs.sign(final_turn) * 16, abs(final_turn))

    # Now the robot faces the wall.  Turn 90° clockwise to travel along it.
    # Clockwise from above = positive turn direction in the motor convention.
    print("[circler] turning 90° clockwise along wall")
    creep.turn_speed_angle(16, 90)

    return True


def _collect_cube(creep: CreepRobot, cube: Object) -> bool:
    """
    Approach and collect a floor cube.  Uses the two-pass alignment in
    go_to_closest_token (turn, re-read, drive, open doors, close doors).

    Returns True if the cube was successfully collected.
    """
    print(f"[circler] collecting cube id={cube.id} "
          f"at {cube.position:.0f}cm h={cube.h_angle:.1f}°")


    return rs.go_to_closest_token(creep, ObjectType.BASE, cube, open_doors=True)


def _deposit_and_realign(creep: CreepRobot) -> None:
    """
    Go home, drop boxes into the lab, reset inventory, then realign to a wall
    marker ready for another circling pass.
    """
    print("[circler] going home to deposit")
    if not rs.go_home_norm(creep):
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
    _align_to_wall(creep)


# ── Main strategy entry point ─────────────────────────────────────────────────

def strategy_base(creep: CreepRobot) -> None:
    """
    Entry point called from robot.py.

    Phase 0 — Startup grab
    ----------------------
    Scan for the nearest floor BASE cube and collect it immediately before
    starting the circuit.  This mirrors faststart's opening move without the
    hardcoded 150 cm blast.

    Phase 1 — Circuit loop
    ----------------------
    Align → drive in SCAN_STEP segments → collect any floor cubes seen →
    deposit when full or time is short → repeat.
    """

    print("[circler] ── strategy start ──")

    print("[circler] Phase 0: startup grab")
    creep.Doors_wedge()
    floor_cubes = _find_floor_base_cubes(creep)

    if floor_cubes:
        print(f"[circler] startup: {len(floor_cubes)} floor cube(s) visible")
        _collect_cube(creep, floor_cubes[0])
    else:
        # Wide camera sweep before giving up
        found_at_startup = False
        for pan in (30, -30):
            creep.camera_pan(pan)
            all_base = creep.find_objects(ObjectType.BASE)
            if all_base:
                floor_only = [o for o in all_base if o.on_floor]
                if floor_only:
                    best = min(floor_only, key=lambda o: o.position)
                    # Correct for camera pan: robot needs to turn toward the cube
                    best.h_angle += pan   # approximate correction
                    creep.camera_pan(0)
                    _collect_cube(creep, best)
                    found_at_startup = True
                    break
        if not found_at_startup:
            creep.camera_pan(0)
            print("[circler] no floor cube at startup — going straight to circuit")

    floor_cubes = _find_floor_base_cubes(creep)

    while floor_cubes is not None:
        _collect_cube(creep, floor_cubes[0])
        floor_cubes = _find_floor_base_cubes(creep)
        

    # ── Phase 1: align to wall before starting circuit ────────────────────
    print("[circler] Phase 1: initial wall alignment")
    if not _align_to_wall(creep):
        # No markers visible — spin in 45° increments until one appears
        for _ in range(8):
            creep.turn_speed_angle(16, 45)
            if _align_to_wall(creep):
                break
        else:
            rs.recovery(creep)

    # ── Phase 2: main circuit loop ────────────────────────────────────────
    print("[circler] Phase 2: entering circuit loop")

    while not _should_go_home(creep):


        print(f"[circler] driving {SCAN_STEP:.0f}cm along wall")
        creep.drive_speed_distance(30, SCAN_STEP)


        floor_cubes = _find_floor_base_cubes(creep)

        while floor_cubes is not None:
            # Take the nearest floor cube
            target = floor_cubes[0]
            print(f"[circler] {len(floor_cubes)} floor cube(s) visible; "
                  f"targeting id={target.id} at {target.position:.0f}cm")

            collected = _collect_cube(creep, target)

            if collected:
                floor_count = getattr(creep, "floor_tokens_collected", 0)
                print(f"[circler] collected! total floor={floor_count}")
            else:
                print("[circler] collection failed — continuing circuit")

            # Re-align to wall after detour
            floor_cubes = _find_floor_base_cubes(creep)
        _align_to_wall(creep)

        # candidates = []
        # best_marker: Object | None = None
        # creep.camera_pan(0)
        # markers = creep.find_objects(ObjectType.ARENA_MARKER)
        # if markers:

        #     for marker in markers:
        #         for corner_id in corner_ids:
        #             if marker.id == corner_id:
        #                 candidates.append(marker)
            
        #     candidate = min(candidates, key=lambda m: m.position)

        #     if candidate is None:
        #         candidate = min(markers, key=lambda m: m.position)

        #     if best_marker is None or candidate.position < best_marker.position:
        #         best_marker = candidate

        # ── Periodic realignment ──────────────────────────────────────────
        # Look for a wall marker to check drift.  If one is unusually close or
        # at a sharp angle we've started rounding a corner — turn and realign.
        creep.camera_pan(0)
        wall_markers = creep.find_objects(ObjectType.ARENA_MARKER)
        if wall_markers:
            # The nearest marker seen while cruising parallel to the wall
            # should be roughly perpendicular (h_angle ≈ ±90° or behind us).
            # If a marker is nearly dead ahead (|h_angle| < 30°), we're
            # approaching a corner — realign to stay on track.
            ahead = [m for m in wall_markers if abs(m.h_angle) < 30
                     and m.position < WALL_STANDOFF * 2]
            if ahead:
                print(f"[circler] corner detected (marker id={ahead[0].id} "
                      f"at {ahead[0].position:.0f}cm, h={ahead[0].h_angle:.1f}°) "
                      "— realigning")
                _align_to_wall(creep)

    # ── Phase 3: go home ──────────────────────────────────────────────────
    print("[circler] Phase 3: heading home")
    if _time_remaining(creep) <= HOME_BUFFER:
        raise Exception("GO HOME NOW NOW NOW NOW!") 
    _deposit_and_realign(creep)
    strategy_base(creep)

