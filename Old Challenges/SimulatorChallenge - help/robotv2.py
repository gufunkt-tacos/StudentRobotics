from sr.robot3 import Robot, OUT_H0, INPUT
import time

robot = Robot()
cw = 1
ccw = -1
currentID = -1

homeArena = 0
arena = 1
acidic = 2
basic = 3

leftTrim  = 0.95
rightTrim = 1


motorL = robot.motor_board.motors[0]
motorR = robot.motor_board.motors[1]

robot.arduino.pins[11].mode = INPUT
robot.arduino.pins[12].mode = INPUT



def bigStepForward():
    print("BIG STEP FORWARD")
    power = 0
    for i in range(5):
        power += 0.1
        motorL.power = power * leftTrim
        motorR.power = power * rightTrim
        robot.sleep(0.1)
    motorL.power = 0
    motorR.power = 0
    robot.sleep(0.1)


def mediumStepForward():
    print("SMALL STEP FORWARD")
    motorL.power = 1 * leftTrim
    motorR.power = 1 * rightTrim
    robot.sleep(0.2)
    motorL.power = 0
    motorR.power = 0
    robot.sleep(0.1)

def smallStepForward():
    print("TINY STEP FORWARD")
    motorL.power = 1 * leftTrim
    motorR.power = 1 * rightTrim
    robot.sleep(0.1)
    motorL.power = 0
    motorR.power = 0
    robot.sleep(0.1)


def bigStepAngle(direction):
    print("BIG STEP ANGLE")
    motorL.power = 1 * direction
    motorR.power = -1 * direction
    robot.sleep(0.03)
    motorL.power = 0
    motorR.power = 0
    robot.sleep(0.1)


def smallStepAngle(direction):
    print("SMALL STEP ANGLE")
    motorL.power = 1 * direction
    motorR.power = -1 * direction
    robot.sleep(0.003)
    motorL.power = 0
    motorR.power = 0
    robot.sleep(0.05)


def lowestDistance(markers):
    distances = []
    for marker in markers:
        distances.append(marker.position.distance)
    print(distances)
    minID = distances.index(min(distances))
    return markers[minID]

def search(idType):
    markers = robot.camera.see()
    markersOfInterest = []
    for marker in markers:
        #if marker.position.vertical_angle > 0.5:
        #    continue
        if any(id == marker.id for id in blacklist):
            continue
        if marker.id == 19:
            if idType == homeArena:
                markersOfInterest.append(marker)
        elif marker.id < 18:
            if idType == arena:
                markersOfInterest.append(marker)
        elif marker.id < 140:
            if idType == acidic:
                markersOfInterest.append(marker)
        elif marker.id < 180:
            if idType == basic:
                markersOfInterest.append(marker)
                print("APPENDED")

    print(markersOfInterest)
    if len(markersOfInterest) == 0:
        print("EMPTY")
        bigStepAngle(cw)
        markersOfInterest = search(idType)

    return markersOfInterest
        
def moveTowards(focus, finalDistance = 0):
    
    '''
    found = False
    for marker in markers:
        if marker.id == focusID:
            found = True
            focus = marker
    if found == False:
        print("NOT FOUND")
        focus = lowestDistance(search(basic))
    '''
    angle = focus.position.horizontal_angle
    if abs(angle) > 0.1:
        if angle < 0:
            if angle < -0.26:
                bigStepAngle(ccw)
            else:
                smallStepAngle(ccw)
        if angle > 0:
            if angle > 0.26:
                bigStepAngle(cw)
            else:
                smallStepAngle(cw)
        return False


    distance = focus.position.distance - finalDistance

    
    if distance > 500:
        bigStepForward()
    elif distance > 300:
        mediumStepForward()
    else:
        distance = robot.arduino.ultrasound_measure(2,3) - finalDistance
        while True:
            distance = robot.arduino.ultrasound_measure(2,3) - finalDistance
            smallStepForward()
            robot.sleep(0.05)
            if distance < 50 and finalDistance != 0:
                return True
            if robot.arduino.pins[10].digital_read() == True or robot.arduino.pins[11].digital_read() == True:
                print("FOUND IT")
                return True
    
    return False

def pickup():
    robot.servo_board.servos[0].position = -1
    robot.power_board.outputs[OUT_H0].is_enabled = True
    robot.sleep(1)
    robot.servo_board.servos[0].position = 1
    robot.sleep(1)

def drop():
    robot.servo_board.servos[0].position = -1
    robot.sleep(1)
    robot.power_board.outputs[OUT_H0].is_enabled = False
    robot.servo_board.servos[0].position = 1
    robot.sleep(1)



bigStepForward()
robot.servo_board.servos[0].position = 1
blacklist = []


while True:
    focus = lowestDistance(search(basic))
    if moveTowards(focus, 0) == True:
        pickup()
        blacklist.append(focus.id)
        break


while True:
    focus = lowestDistance(search(homeArena))
    if moveTowards(focus, 300) == True:
        drop()
        break

while True:
    focus = lowestDistance(search(basic))
    if moveTowards(focus, 0) == True:
        blacklist.append(focus.id)
        pickup()
        break


while True:
    focus = lowestDistance(search(homeArena))
    if moveTowards(focus, 300) == True:
        drop()
        break

print("SUCCESS")
robot.sleep(1)


