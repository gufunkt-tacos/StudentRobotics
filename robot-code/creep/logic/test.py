from creep.machine import *

def strategy_base(creep: CreepRobot):

    while True:
        objects = creep.find_objects(ObjectType.BASE)
        if objects is None:
            continue
        for obj in objects:
            print(f"Object {obj.id} has on_floor={obj.has_top_face}")
            if obj.on_floor:
                creep.LED_C_red()
            else:
                creep.LED_C_blue()
        



    #if there's time, turn round the corner and  get more boxes
    #once the time reaches a threshold, go home.