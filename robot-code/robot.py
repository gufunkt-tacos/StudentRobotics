from creep.machine import CreepRobot, ObjectType
from creep.logic.basic_logic import *
from creep.logic.robot_subroutines import *
from creep.logic.basic_strategy import *
import threading
import time


def main():
    creep = CreepRobot()
    home_timer: threading.Timer | None = None

    try:
        creep.initialise()

        # ── Home Timer ─────────────────────────────────────────────────────────
        # Fires go_home_norm() automatically when go_home_time seconds have
        # elapsed, even if the strategy loop is mid-action.  The flag
        # creep._home_triggered prevents double-firing.
        creep._home_triggered = False

        def _trigger_go_home():
            if creep._home_triggered:
                return
            creep._home_triggered = True
            print("[timer] *** HOME TIMER FIRED — heading home now ***")
            try:
                creep.motor_stop()          # abort whatever motion is running
                go_home_norm(creep)
            except Exception as e:
                print("[timer] Error during timed go-home:", e)

        remaining = max(0.0, creep.go_home_time - (time.time() - creep.time_started_game))
        home_timer = threading.Timer(remaining, _trigger_go_home)
        home_timer.daemon = True            # won't block exit if main finishes first
        home_timer.start()
        print(f"[main] Home timer armed: {remaining:.1f}s")
        # ───────────────────────────────────────────────────────────────────────

        strategy_base(creep)

    except Exception as e:
        print("An error occurred:", e)
        creep.error_jingle()

    finally:
        # Cancel the background timer so it doesn't fire after we're done
        if home_timer is not None:
            home_timer.cancel()

        # Only go home here if the timer hasn't already handled it
        if not getattr(creep, "_home_triggered", False):
            go_home_norm(creep)


if __name__ == "__main__":
    main()
