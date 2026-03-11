from ..machine import *

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
        creep.drive_speed_distance(32, closest_token_dist/1.05)
        lab_wall = get_marker_wall(creep.my_lab[2])

        wall_facing = get_marker_wall(marker_id)
        wall_zero = lab_wall
        wall_one = (lab_wall + 1)%4
        wall_two = (lab_wall + 2)%4
        wall_three = (lab_wall + 3)%4
        print(wall_zero, wall_one, wall_two, wall_three)
        print(marker_id)
        print(get_marker_wall(marker_id))
        print("wall facting: " + str(wall_facing))

        match wall_facing:
            case int(wall_zero):
                print("turning from wall zero")
                creep.turn_speed_angle(-16, 90)
            case int(wall_one):
                print("turning from wall one")
                creep.turn_speed_angle(-16, 90)
                print(creep.left_front_sonar(), creep.right_front_sonar)
                if(creep.left_front_sonar() < 50 or creep.right_front_sonar() < 50):
                    print("sonar used")
                    creep.turn_speed_angle(-16, 90)
            case int(wall_two):
                print("turning from wall two")
                creep.turn_speed_angle(16, 90)
                print(creep.left_front_sonar(), creep.right_front_sonar)
                if(creep.left_front_sonar() < 50 or creep.right_front_sonar() < 50):
                    print("sonar used")
                    creep.turn_speed_angle(16, 90)
            case int(wall_three):
                print("turning from wall three")
                creep.turn_speed_angle(16, 90)
            case _:
                print("can't find which wall i'm at") #should never reach this
        
        
                    

    else:
        print("no token found")
        creep.turn_speed_angle(-16,90) # if it can't see anything, turn
    
    go_home(creep)


def navigate_obstacle(creep: CreepRobot):
    creep.Doors_wedge()
    right_sonar = creep.right_front_sonar()
    left_sonar = creep.left_front_sonar()
    if(right_sonar < 30 or left_sonar < 30):
        valid_tokens = creep.find_objects(ObjectType.TOKEN)
        print(valid_tokens)
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
