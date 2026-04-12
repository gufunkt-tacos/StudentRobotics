from creep.machine import CreepRobot, ObjectType
import creep.logic.robot_subroutines as rs
import creep.logic.super_basic_strategy as sbs
import creep.logic.basic_strategy as bs
import creep.logic.circler as circ
import creep.logic.twobluefloor as twobf
import creep.logic.faststart as fs
import threading
import time



def main():
    creep = CreepRobot()

    creep.strat = circ

    home_timer = None
    try:
        creep.initialise()
        creep.home_triggered = False

        def trigger_go_home():
            if creep.home_triggered:
                return
            creep.home_triggered = True
            print("[timer] *** HOME TIMER FIRED — heading home now ***")
            try:
                creep.motor_stop()          # abort whatever motion is running
                rs.go_home_norm(creep)
            except Exception as e:
                print("[timer] Error during timed go-home:", e)

        remaining = max(0.0, creep.go_home_time - (time.time() - creep.time_started_game))
        home_timer = threading.Timer(remaining, trigger_go_home)
        home_timer.daemon = True            # won't block exit if main finishes first
        home_timer.start()
        print(f"[main] Home timer armed: {remaining:.1f}s")

        creep.strat.strategy_base(creep)
        # bs.strategy_base(creep)
    except Exception as e:
        print("An error occurred:", e)
        creep.error_jingle()
    finally:
        if home_timer is not None:
            home_timer.cancel()

        # Only go home here if the timer hasn't already handled it
        if not getattr(creep, "home_triggered", False):
            rs.go_home_norm(creep)



        # home_coords = creep.get_home_coords()
        # go_to_coords(creep, int(home_coords[0]), int(home_coords[1]))
        # creep.drive_speed_distance(-40, 150)
        # rs.go_home_norm(creep)
        creep.drive_speed_distance(40, 50)
        creep.startup_jingle()

if __name__ == "__main__":
    main()
