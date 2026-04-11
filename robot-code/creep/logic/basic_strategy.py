from creep.machine import *
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *


def strategy_base(creep: CreepRobot):
    """Main match strategy."""

    # ── Phase 1: grab the nearest floor box ──────────────────────────────────
    print("[strategy] Phase 1: searching for nearest floor box")
    box = search_for_boxes(creep, ObjectType.BASE)
    if box is not None:
        go_to_closest_token(creep, ObjectType.BASE, box, open_doors=True)
    else:
        # Fallback to old camera-pan-only search
        get_token(creep, ObjectType.BASE, "floor")

    if not creep.can_continue():
        return

    # ── Phase 2: nudge the acid box out of the way ───────────────────────────
    print("[strategy] Phase 2: pushing acid box aside")
    acid = find_closest_token(creep, ObjectType.ACID, 0)
    if go_to_closest_token(creep, ObjectType.ACID, acid, open_doors=False):
        print("[strategy] Pushed acid box out of the way")
    else:
        print("[strategy] No acid box found, moving on")

    if not creep.can_continue():
        return

    # ── Phase 3: ledge box ───────────────────────────────────────────────────
    print("[strategy] Phase 3: collecting ledge box")
    get_ledge_token(creep, ObjectType.BASE, 90)

    if not creep.can_continue():
        return

    # ── Phase 4: second floor box (search + fallback drive) ──────────────────
    print("[strategy] Phase 4: searching for second floor box")
    box2 = search_for_boxes(creep, ObjectType.BASE)
    if box2 is not None:
        go_to_closest_token(creep, ObjectType.BASE, box2, open_doors=True)
    else:
        floor_found = get_token(creep, ObjectType.BASE, "floor")
        if not floor_found:
            print("[strategy] No floor box found — driving blind to known area")
            safe_drive(creep, 16, 100)

    if not creep.can_continue():
        return

    # ── Phase 5: keep collecting if time allows ───────────────────────────────
    print("[strategy] Phase 5: opportunistic extra collection")
    while creep.can_continue():
        box_extra = search_for_boxes(creep, ObjectType.BASE)
        if box_extra is None:
            print("[strategy] No more boxes visible — waiting for timer")
            break
        go_to_closest_token(creep, ObjectType.BASE, box_extra, open_doors=True)

    # strategy_base returns; robot.py finally-block calls go_home_norm
    print("[strategy] Strategy complete — handing over to go-home")
