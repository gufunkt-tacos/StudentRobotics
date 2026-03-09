from ..machine import CreepRobot

def sign(x):
    if(x>=0):
        return 1
    else:
        return -1
    
def find_closest_token(type, angle_offset, creep: CreepRobot):

    valid_tokens = []
    valid_tokens = creep.find_objects(type)
    found = valid_tokens[0]
    if found == True:
        print(valid_tokens)
        closest_token_dist = int(valid_tokens[2][0])
        token_angle = valid_tokens[3][0]
        print(len(valid_tokens[2]))
        for i in range(0,len(valid_tokens[2])):
            if(valid_tokens[2][i] < closest_token_dist):
                closest_token_dist = valid_tokens[2][i]
                token_angle = valid_tokens[3][i]
                print(i)
                print(valid_tokens[2][i])
                print(valid_tokens[3][i])
        
        print("angle seen = " + str(token_angle))

        token_angle-=angle_offset

        return [closest_token_dist, token_angle, found]
    else:
        return [0,0, False]

    
def go_to_closest_token(type: str, closest_token: list, creep: CreepRobot):

    scaling = 1
    creep.camera_pan(0)

    closest_token_dist = closest_token[0]
    token_angle = closest_token[1]
   
    print(closest_token_dist)
    creep.turn_speed_angle(10*sign(token_angle), abs(token_angle)/scaling)

    token_angle = find_closest_token(type, 0)[1]
    
    creep.turn_speed_angle(5*sign(token_angle), (abs(token_angle)/scaling)/2)

    creep.drive_speed_distance(40, closest_token_dist)
    creep.Doors_open()
    creep.drive_speed_distance(40, 60)
    creep.Doors_close()
    creep.Doors_wedge()

def get_floor_token(type, creep: CreepRobot):

    angle_offset = 0

    closest_token = find_closest_token(type, angle_offset, creep)
    if(closest_token[2] == True):
        go_to_closest_token(type, closest_token, creep)
        return True
    else:
        creep.camera_pan(45)
        angle_offset = 45
        closest_token = find_closest_token(type, angle_offset, creep)

        if(closest_token[2] == True):
            go_to_closest_token(type, closest_token, creep)
            return True
        else:
            creep.camera_pan(-45)
            angle_offset = -45
            closest_token = find_closest_token(type, angle_offset, creep)

            if(closest_token[2] == True):
                go_to_closest_token(type, closest_token, creep)
                return True
            else:
                creep.camera_pan(0)
                return False


    

def get_token(type, level, creep: CreepRobot):
     creep.camera_pan(0)
     angle_offset = 0
     creep.Doors_wedge()
     if (level == "floor"):
        get_floor_token(type, creep)
     
        


def next_arena_token(start, step, direction):
    token = start + (step*direction)
    if(token > 19):
        token-=20
    if(token < 0):
        token+=20
    return token
    

def go_home(creep: CreepRobot):
    
    creep.Doors_wedge()
    closest_token_dist = 8192
    closest_token_angle = 361
    marker_found = False

    arena_markers = creep.find_objects("arena_marker")
    for i in range(0,len(arena_markers[1])):
        for j in range(0,2):
            if(arena_markers[0][i] == my_lab[j]):
                marker_found = True
                if(arena_markers[2][i] < closest_token_dist):
                    closest_token_dist = arena_markers[2][i]
                    closest_token_angle = arena_markers[3][i]
    
    if(marker_found == True):
        creep.turn_speed_angle(5, closest_token_angle)
        creep.drive_speed_distance(30, closest_token_dist)
    else:
        for step in range(1,10):
            for i in range(0,len(arena_markers[1])):
                if(arena_markers[0][i] == next_arena_token(my_lab[0],step,-1)):
                    marker_found = True
                    if(arena_markers[2][i] < closest_token_dist):
                        closest_token_dist = arena_markers[2][i]
                        closest_token_angle = arena_markers[3][i]
                        
                if(arena_markers[0][i] == next_arena_token(my_lab[2],step,1)):
                    marker_found = True
                    if(arena_markers[2][i] < closest_token_dist):
                        closest_token_dist = arena_markers[2][i]
                        closest_token_angle = arena_markers[3][i]
        
        if(marker_found == True):
            creep.turn_speed_angle(5, closest_token_angle)
            creep.drive_speed_distance(30, closest_token_dist)
            creep.turn_speed_angle(5,90*sign(closest_token_angle))
        else:
            creep.drive_speed_distance(30,25)
        
        go_home()


def navigate_obstacle(creep: CreepRobot):
    creep.Doors_wedge()
    right_sonar = creep.right_front_sonar()
    left_sonar = creep.left_front_sonar()
    if(right_sonar < 30 or left_sonar < 30):
         
         valid_tokens = creep.find_objects("base_tokens")
         print(valid_tokens)
         if (valid_tokens[0] == True):
            closest_token_dist = valid_tokens[2][0]
            print(len(valid_tokens[2]))
            for i in range(0,len(valid_tokens[2])):
                if(valid_tokens[2][i] < closest_token_dist):
                    closest_token_dist = valid_tokens[2][i]
        
         valid_tokens = creep.find_objects("acid_tokens")
         print(valid_tokens)
         if (valid_tokens[0] == True):
            print(len(valid_tokens[2]))
            for i in range(0,len(valid_tokens[2])):
                if(valid_tokens[2][i] < closest_token_dist):
                    closest_token_dist = valid_tokens[2][i]
            
         if (closest_token_dist > 50):
            creep.turn_speed_angle(32*sign(right_sonar-left_sonar),90)
            creep.drive_speed_distance(20,50)
            creep.turn_speed_angle(32*-sign(right_sonar-left_sonar),90)
