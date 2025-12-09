from sr.robot3 import Robot
import time

robot = Robot()
cw = 1
ccw = -1
currentID = -1

motorL = robot.motor_board.motors[0]
motorR = robot.motor_board.motors[1]


def bigStepForward():
    print("BIG STEP FORWARD")
    power = 0
    for i in range(5):
        power += 0.1
        motorL.power = power
        motorR.power = power
        robot.sleep(0.1)
    motorL.power = 0
    motorR.power = 0


def mediumStepForward():
    print("SMALL STEP FORWARD")
    motorL.power = 1
    motorR.power = 1
    robot.sleep(0.2)
    motorL.power = 0
    motorR.power = 0

def smallStepForward():
    print("TINY STEP FORWARD")
    motorL.power = 1
    motorR.power = 1
    robot.sleep(0.1)
    motorL.power = 0
    motorR.power = 0


def bigStepAngle(direction):
    print("BIG STEP ANGLE")
    motorL.power = 1 * direction
    motorR.power = -1 * direction
    robot.sleep(0.03)
    motorL.power = 0
    motorR.power = 0


def smallStepAngle(direction):
    print("SMALL STEP ANGLE")
    motorL.power = 1 * direction
    motorR.power = -1 * direction
    robot.sleep(0.003)
    motorL.power = 0
    motorR.power = 0



def filterMarkers(markers):
    markersOfInterest = []
    for marker in markers:
        if marker.id < 180:
            if marker.id >=  140:
                markersOfInterest.append(marker)
    print(markersOfInterest)
    return markersOfInterest

def lowestDistance(markers):
    distances = []
    for marker in markers:
        distances.append(marker.position.distance)
    print(distances)
    minID = distances.index(min(distances))
    return markers[minID]
    

bigStepForward()
robot.servo_board.servos[0].position = 1

while True:

    markers = robot.camera.see()
    focus = lowestDistance(filterMarkers(markers))

    angle = focus.position.horizontal_angle
    if abs(angle) > 0.1:
        if angle < 0:
            if angle < -0.26:
                bigStepAngle(ccw)
            else:
                smallStepAngle(ccw)
        if angle > 0:
            if angle > 0.25:
                bigStepAngle(cw)
            else:
                smallStepAngle(cw)
        
        continue


    distance = focus.position.distance

    
    if distance > 500:
        bigStepForward()
    elif distance > 300:
        mediumStepForward()
    else:
        distance = robot.arduino.ultrasound_measure(2, 3)
        while distance > 50:
            smallStepForward()
            robot.sleep(0.05)
            distance = robot.arduino.ultrasound_measure(2, 3)
        print("FOUND IT")
        robot.servo_board.servos[0].position = -1
        robot.sleep(2)
        robot.servo_board.servos[0].position = 1
        robot.sleep(2)
        break


