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
    floor_cubes = rs.find_floor_base_cubes(creep)

    if floor_cubes:
        print(f"[circler] startup: {len(floor_cubes)} floor cube(s) visible")
        rs.collect_cube(creep, floor_cubes[0])
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
                    rs.collect_cube(creep, best)
                    found_at_startup = True
                    break
        if not found_at_startup:
            creep.camera_pan(0)
            print("[circler] no floor cube at startup — going straight to circuit")

    floor_cubes = rs.find_floor_base_cubes(creep)

    while floor_cubes != []:
        rs.collect_cube(creep, floor_cubes[0])
        floor_cubes = rs.find_floor_base_cubes(creep)
        

    # ── Phase 1: align to wall before starting circuit ────────────────────
    print("[circler] Phase 1: initial wall alignment")
    if not rs.align_to_wall(creep):
        # No markers visible — spin in 45° increments until one appears
        for _ in range(8):
            creep.turn_speed_angle(16, 45)
            if rs.align_to_wall(creep):
                break
        else:
            rs.recovery(creep)

    # ── Phase 2: main circuit loop ────────────────────────────────────────
    print("[circler] Phase 2: entering circuit loop")

    while not rs.should_go_home(creep):


        print(f"[circler] driving {rs.SCAN_STEP:.0f}cm along wall")
        creep.drive_speed_distance(30, rs.SCAN_STEP)


        floor_cubes = rs.find_floor_base_cubes(creep)

        while floor_cubes:
            # Take the nearest floor cube
            target = floor_cubes[0]
            print(f"[circler] {len(floor_cubes)} floor cube(s) visible; "
                  f"targeting id={target.id} at {target.position:.0f}cm")

            collected = rs.collect_cube(creep, target)

            if collected:
                floor_count = getattr(creep, "floor_tokens_collected", 0)
                print(f"[circler] collected! total floor={floor_count}")
            else:
                print("[circler] collection failed — continuing circuit")

            # Re-align to wall after detour
            floor_cubes = rs.find_floor_base_cubes(creep)
        rs.align_to_wall(creep)

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
                     and m.position < rs.WALL_STANDOFF * 2]
            if ahead:
                print(f"[circler] corner detected (marker id={ahead[0].id} "
                      f"at {ahead[0].position:.0f}cm, h={ahead[0].h_angle:.1f}°) "
                      "— realigning")
                rs.align_to_wall(creep)

    # ── Phase 3: go home ──────────────────────────────────────────────────
    print("[circler] Phase 3: heading home")
    if rs.time_remaining(creep) <= rs.HOME_BUFFER:
        raise Exception("GO HOME NOW NOW NOW NOW!") 
    rs.deposit_and_realign(creep)
    strategy_base(creep)

