from ..machine import CreepRobot, Object, ObjectType
import math
import time

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
        None

    
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


global speedfactor # only temporary
global maxspeed
maxspeed = 30
speedfactor = 1


def approach_ledge(creep: CreepRobot, targetDistance = 10, tolerance = 5):
    creep.Doors_close()
    creep.Arm_Extend(1)

    success = False

    while not success:
        success = True

        right_distance = creep.right_front_sonar() - targetDistance
        left_distance = creep.left_front_sonar() - targetDistance
        left_speed = 0
        right_speed = 0

        if abs(right_distance) > tolerance:
            success = False
            right_speed = min(int(right_distance * speedfactor), sign(right_distance) * maxspeed)   
            print(right_speed)
        if abs(left_distance) > tolerance:
            success = False
            left_speed = min(int(left_distance * speedfactor), sign(left_distance) * maxspeed)
            print(left_distance)

        print()

        creep.drive_both(left_speed, right_speed)
    
    creep.motor_stop()

    print("Success!")

global sucker_timeout       # temp change later
global required_box_height
global time_for_wiggle
required_box_height = 5
sucker_timeout = 10
time_for_wiggle = 0.5

def box_detected(creep):
    return creep.sucker_height() <= required_box_height

def look_for_box_on_ledge(creep: CreepRobot):
    global required_box_height
    global sucker_timeout

    if box_detected(creep):
        get_box(creep)
    else:
        start_time = time.time()
        creep.drive_both(moving_speed,-moving_speed)

        while time.time() - start_time < time_for_wiggle:
            if box_detected(creep):
                return True
            
        start_time = time.time()
        creep.drive_both(-moving_speed,moving_speed)

        while time.time() - start_time < 2 * time_for_wiggle:
            if box_detected(creep):
                return True

        start_time = time.time()
        creep.drive_both(moving_speed, - moving_speed)

        while time.time() - start_time < time_for_wiggle:
            if box_detected(creep):
                return True
    
    return False
        
global ledge_collection_success_flag
ledge_collection_success_flag = False


        
def get_box(creep: CreepRobot):
        global ledge_collection_success_flag
        creep.VacPump(1)
        creep.VacValve("GRIP")
        creep.Arm_tilt_down()
        start_time = time.time()
        while time.time() - start_time < sucker_timeout:
            if creep.sucker_gripping():
                ledge_collection_success_flag = True
                break
            ledge_collection_success_flag = False
        return



def clean_up_after_collecting_from_ledge(creep: CreepRobot):
    global distance_from_ledge
    creep.drive_speed_distance(moving_speed, distance_from_ledge)
    return

global distance_from_ledge  # THIS NEEDS CHANGING ONLY FOR TESTING 
distance_from_ledge = 50

# def calculate_optimal_ledge_collection(creep: CreepRobot, obj: Object):
#     global distance_from_ledge

#     # This is just the cosine rule dont worry
#     distance_to_move = math.sqrt(
#         distance_from_ledge**2 + obj.position**2
#         - 2 * distance_from_ledge * obj.position * math.cos(math.radians(obj.yaw))
#     )

#     # This is just the sine rule OK
#     angle_to_move = math.degrees(
#         math.asin(distance_from_ledge * math.sin(math.radians(obj.yaw)) / distance_to_move)
#     )

#     return distance_to_move, angle_to_move

global angle_speed
angle_speed = 30
global moving_speed
moving_speed = 30

def collect_box_from_ledge(creep: CreepRobot, obj: Object):
    global distance_from_ledge
    global ledge_collection_success_flag

    distance_to_move = math.sqrt(
        distance_from_ledge**2 + obj.position**2
        - 2 * distance_from_ledge * obj.position * math.cos(math.radians(obj.yaw))
    )

    angle_to_move = math.degrees(
        math.asin(distance_from_ledge * math.sin(math.radians(obj.yaw)) / distance_to_move)
    )

    creep.turn_speed_angle(angle_speed, angle_to_move)

    creep.drive_speed_distance(moving_speed, distance_to_move)

    approach_ledge(creep)

    if look_for_box_on_ledge(creep):
        get_box(creep)

    clean_up_after_collecting_from_ledge(creep)

    if ledge_collection_success_flag:
        print("SEQUENCE COMPLETED SUCCESSFULYL AND BOX WAS PICKED UP")
    else:
        print("SEQUENCE FAILED AND NO BOX WAS PICKED UP")







