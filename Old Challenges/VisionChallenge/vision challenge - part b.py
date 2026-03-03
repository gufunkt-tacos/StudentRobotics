# 18 November 2025
# # Base I2c (8 bit) default address for the MD25 = Blue xB0 (xB1 read), Orange xB2(xB3 read)
# Add the ID codes e.g "SR0HG1B" for your specific SR modules
# Be careful with case, upper or lower and 0's O's etc. Copy from your log file if unsure
# Camera Board  
# Power Board  
# Motor board 
# Motor board 
# Ruggeduino 
# Servo Board 
# USB-ISS adaptor

#You will need a WiFi key to be able to connect
#You can find this inside robot-settings.toml on the USB drive containing your code.

#team_tla = "ZZZ52700"
#usercode_entrypoint = "robot.py"
#wifi_psk = "5f10-c3e8-ea56"
#wifi_region = "GB"
#wifi_enabled = true

# WIFI connection to a laptop/PC or tablet. Search for wifi sources. Use SR password then open the following in a browser
# http://robot.lan   

from sr.robot3 import * # New SR code based on Python 3.11 (I'm using 3.12.2)
import time
import serial   # ignore import error if you get one
import math
import numpy as np
from math import cos, sin, pi, sqrt, atan2
import serial.tools.list_ports

global time_started_robot
time_started_robot = time.time()   # get time when ON/OFF switch pressed

#robot = Robot(wait_for_start=True)   #, no_powerboard=True) # no_powerboard if not using SR kit
robot = Robot(wait_for_start=False)#, no_powerboard=True) # no_powerboard if not using SR kit


#robot.servo_board.servos[0].set_duty_limits(800,2200)
#robot = Robot.ignored_arduinos=["7543535313835170F0B0"]
#robot = Robot(wait_for_start=False, ignored_arduinos=["75830333338351803001"]) # New set-up "7543535313835170F0B0" 75830333338351803001


#""""""""""""""""""""""""""""" variables""""""""""""""""""""""""""""""""""

#These variables need to be defined as global within the functions that use them
wheelspace = 37.9 # 37.9 new robot,31.5 tracks,34.60 for old robot,36.75 for 2020 test base
wheel_diameter = 10.8 #10.8 for new robot, 10 without tyres,5.10 tracks,10.50 for 10cm wheel with tyre
max_encoder = 4294967295 # required when encoder value <0
camera_servo_offset_value=0
#...............................................................................
my_corner = 0 # Will be set by competition dongle with R.zone
# The following variables are used by the MD25 motor drive board
acelrate = 5 # DO NOT SET acelerate to > 5 unless using low speeds. Reset to < 5 after
encoder1 = 0 # Set encoder for motor 1 to zero
encoder2 = 0 # Set encoder for motor 2 to zero
encoder1value = 0
encoder2value = 0
#initialise speed & turn variables to zero
turn = 0        
speed = 0
speed1 = 0
speed2 = 0
drive_timeout_time = 15
drive_time_out = False
turn_timeout_time = 10
turn_timeout = False
#d2r = pi/180

# These coordinates are correct for the 2025-2026 game. List starts at marker '0' to marker 19
marker_coords = \
[[76,0],[153,0],[229,0],[305,0],[381,0] \
,[458,76],[458,153],[458,229],[458,305],[458,381] \
,[381,458],[305,458],[229,458],[153,458],[76,458] \
,[0,381],[0,305],[0,229],[0,153],[0,76]]


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#----------------------------------Defined functions----------------------------
#__________________________________________________________________________

#                         find_ports()
# This routine examines the devices connected to the USB ports of the Pi4
# The name of the device (e.g USB-ISS for the USB to I2C interface adaptor) is passed to the routine
# If the device is found it prints out the tty.device_node, Vendor ID (VID) and Product ID (PID)

def find_ports(portname):
    print ("in find_ports, looking for ",portname)
    ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(ports):
        #print("{}: {} [{}]".format(port, desc, hwid))
        #print("port = ", port)
        #print("desc = ", desc)
        #print("hwid = ", hwid)
        #print("========================================")
        #print("Now checking desc ",portname)
        if desc == portname:
            print(portname, " found, port = ",port)
            #ser = serial.Serial(port,115200)
            print("========================================")
            return port, desc
    return "not detected",""
       
   
#                       DRIVE MOTOR RELATED FUNCTIONS
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#_______________________________________________________________________________
#       dis_2sec_timeout()
# This function disables the MD25 motor controller 2 second timeout
# I2C interface uses 1 byte addressed device mode 0x55
# x55 selects I2C, Blue MD25 base address xB0(write)xB1(read) , command reg x10, 1byte, disable x32
# x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32

def dis_2sec_timeout():
    resp =0
    ser.write( b"\x55\xB2\x10\x01\x32" )  # b required to change unicode to bytes
    n = ser.read(1) #  get acknowledge, doesn't really matter about the format
    resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
    #print("response = ",resp)
    if resp != 0 :
        print ("Motor timeout disabled")
    return
#_______________________________________________________________________________

#        enables_2sec_timeout()
# This function enables the MD25 motor controller 2 second timeout
# x55 selects I2C, Blue MD25 base address xB0, command reg x10, 1byte, enable x33
# x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32

def enables_2sec_timeout():
    ser.write( b"\x55\xB2\x10\x01\x33" )  # b required to change unicode to bytes
    n = ser.read(1) # get acknowledge
    resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
    if resp != 0 :
        print ("Motor timeout enabled")
    return
#_______________________________________________________________________________
#        reset_both_encoders()
# This function resets both wheel encoder values to zero
# x55 selects I2C, Blue MD25base address xB0, command reg x10, 1byte, reset encoders x20
# x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32

def reset_both_encoders():
    resp = 0
    #print ("in reset_both encoders ")
    ser.write(b"\x55\xB2\x10\x01\x20")  # b required to change unicode to bytes
    #time.sleep(.1)
    n = ser.read(1) # get acknowledge
    resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
    if resp == 0 :
        print ("wheel encoders reset failed")
    if resp != 0 :
        print ("wheel encoders reset")
    time.sleep(.1)
    return
#_______________________________________________________________________________

#        encoder_1() encoder(2)
# This function reads the 4 byte value from encoder 1 and converts to an integer
# This value is saved as a variable "encoder1"
# x55 selects I2C, MD25 base address xB0 (B1 to read), encoder data x02-x05, 1byte
# x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , encoder data x02-x05, 1byte
# x02 - 05  are the registers for the four encoder bytes
# Read hibyte first to capture count for lower bytes
# x01 is number of bytes (1) to be read from each register

def encoder_1():
    #print "reading encoder 1"
    #MD25 is a single byte register device. Handles auto-increment of register address
    #select I2C device (x55), register to start read from (x02) and number of bytes (x04)
    ser.write( b"\x55\xB3\x02\x04" )  # b required to change unicode to bytes
    n = ser.read(4) # read four bytes corresponding to encoder1 (4 values)
    Enc1byte3 = (n[0]) # hi byte
    Enc1byte2 = (n[1])
    Enc1byte1 = (n[2])
    Enc1byte0 = (n[3]) # lo byte
    encoder1 = Enc1byte0 + (Enc1byte1 << 8) +(Enc1byte2 << 16) + (Enc1byte3 << 24)
    #print ("encoder1 =", encoder1)
    return encoder1
#_______________________________________________________________________________
# This function reads the 4 byte value from encoder 2 and converts to an integer
# This value is saved as a variable "encoder2"

def encoder_2():
    #print "reading encoder 2"
    #MD25 is a single byte register device. Handles auto-increment of address
    #select I2C device (x55), register to start read from (x06) and number of bytes (x04)
    ser.write( b"\x55\xB3\x06\x04" )  # b required to change unicode to bytes
    n = ser.read(4) # read four bytes corresponding to encoder2 (4 values)
    Enc2byte3 = (n[0])
    Enc2byte2 = (n[1])
    Enc2byte1 = (n[2])
    Enc2byte0 = (n[3])
    encoder2 = Enc2byte0 + (Enc2byte1 << 8) +(Enc2byte2 << 16) + (Enc2byte3 << 24)
    #print ("encoder2 =", encoder2)
    return encoder2
#_______________________________________________________________________________

#   set_acel_rate(acelrate)
# This function sets the motor acceleration rate, range 1 to 10 fastest
# Do not set to higher than 5(default)
# x55 selectsI2C, Blue MD25 base address xB0, command accel rate reg x10, 1byte, accelrate
# x55 selectsI2C, Orange MD25 base address xB2, command accel rate reg x10, 1byte, accelrate
# x03 sets rate to 3 full revers to full forward in 2.1 seconds
def set_acel_rate(acelrate):

    ser.write( b"\x55\xB2\x10\x01\x05" )  # b required to change unicode to bytes
    n = ser.read(1) # get acknowledge
    resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
    time.sleep(.1)
    if resp != 0 :
        print ("Accel rate set at ",acelrate)
    return
#_______________________________________________________________________________

#   drive_both(speed1, speed2)
# Independant control of both motors. Encoders reset at start of routine
# Send a command to write to speed1 and speed 2
# Send speed 1,2 values in the range -128 to +127
# x55 selects I2C, Blue MD25 base address xB0, mode reg x0F, 1bytes, x00 = mode 0
# x55 selects I2C, Orange MD25 base address xB2, mode reg x0F, 1bytes, x00 = mode 0
# last two values are speed values e.g x70, x90. In default mode0:
# 0(0H) = full reverse, 128(80H) stop, 255(FFH) full forward
# but this function converts from -128 reverse 0, stop +127 forward

def drive_both(speed1, speed2):
    reset_both_encoders()
    # Convert passed variables to +ve integers in the range 0 to 255
    # +ve speed = forward, +ve turn = clockwise
    speed1 = speed1 + 128
    speed2 = speed2 + 128
    #Set up mode register (15, x0F) for mode 0,Independant control of motors, no sync
    ser.write( b"\x55\xB2\x0F\x01\x00" )  # b required to change unicode to bytes
    n = ser.read(1) # get acknowledge
    #Write to registers speed 1 (speed1) and speed 2 (speed2)
    ser.write(bytes([0x55, 0xB0, 0x00, 0x02, speed1, speed2]))
    n = ser.read(1) # get acknowledge
    return
#_______________________________________________________________________________

#       motor_stop()
#   Stop both motors
def motor_stop():
    ser.write( b"\x55\xB2\x00\x02\x80\x80" )  # b required to change unicode to bytes
    n = ser.read(1) # get acknowledge
    time.sleep(.1)
    # reset encoders elsewhere if required
    return
#_______________________________________________________________________________

#       drive_sync(speed, turn)
# Synchronised speed on both motors. Resets both encoders at beginning of routine
# First set up mode register (15, x0F) for mode 2
# Send a command to write to speed1 for speed and speed 2 for turn (0 if no turn required)
# x55 selects I2C, MD25 base address xB2, mode reg x0F, 1byte, x02 = mode 2
# Then write to speed 1 and 2 as per drive_both

def drive_sync(speed, turn):
    #print("In drive_sync()")
    # This sets up MD25 register for speed turn. Turn will continue until motor stop commanded
    # Under MD25 control of speed only. Does not use encoders
    # If turn set to 0 then speed selects forward & reverse speeds
    # If speed set to 0 then robot will turn about central axis at rate set by turn value
    # If speed and turn = 0 then robot will stop
    # Set speed in the range 0 to 127 forward, 0 to -128 reverse
    # Set turn for rate of turn, 0 to 127 clockwise, 0 to -128 anticlock
    # Convert passed variables to +ve integers in the range 0 - 255
    # +ve speed = forward, +ve turn = clockwise
    reset_both_encoders() # may not be necessary
    #print ("in drive_sync(speed,turn):speed ",speed, " turn ", turn)
    #reset encoders elsewhere if required. Encoders not used in this mode
    if speed >= 127 :
        speed = 127
    speed = speed + 128
    turn = turn + 128
    #Set up mode register (15, x0F) for mode 2
    #Synchronises speed of both motors
    ser.write( b"\x55\xB2\x0F\x01\x02" )  # b required to change unicode to bytes
    n = ser.read(1) # get acknowledge
    time.sleep(.1)
    #Write to registers speed 1 (speed) and speed 2 (turn)
    #ser.write( "\x55\xB2\x00\x02"  + chr(speed) + chr(turn))
    ser.write(bytes([0x55, 0xB2, 0x00, 0x02, speed, turn]))
    n = ser.read(1) # get acknowledge
    return

#_______________________________________________________________________________

#       drive_speed_distance(speed, distance)
# Resets both encoders at start of routine
# Drive in a straight line at a defined speed (-128 to +127)
# Set speed in the range 0 to 127 forward, 0 to -128 reverse
# Drive for a defined distance in centimetres
# Uses synchronised speed mode
# Drive times out after a calculated time is exceeded (speed, distance based)
# Checks lift limits and stops lift if necessary

def drive_speed_distance(speed, distance):
    global max_encoder
    global wheel_diameter
    global drive_timeout_time
    # nominal speed of robot at speed 32 = 25cm/s (at speed 32)
    # adjust for accel/decel times etc and add a 1 second margin
    if speed != 0 :
        drive_timeout_time = (((distance * 32) / 22) /abs(speed)) +2
    else :
        drive_timeout_time = 15.0 #default drive timeout time
    if distance <= 0: # Only +ve values of distance allowed
        distance = 0
        move = False
        return
    drive_time_out = False # Flag set as True if time-out occurs
    print ("in drive_speed_distance. Will reset both encoders")
    # convert distance to an encoder value
    required_distance_encoder_value = int((distance / (wheel_diameter * 3.142)) * 360)
    time.sleep(.1)
    # start motors
    # drives with 0 turn at speed set by "speed"
    time_started_drive = time.time() # gets time when drive was started
    drive_sync(speed, 0) #start motors
    # THIS DELAY IS ESSENTIAL TO CORRECT OPERATION, OR IS IT ??????
    time.sleep(.1)
    #If speed > 0 (OK but if <0 then need to correct encoder for -ve values)
    move = False
    if speed > 0 :
        while (encoder_1() < required_distance_encoder_value \
        or encoder_2() < required_distance_encoder_value)\
        and ((time.time() - time_started_drive) < drive_timeout_time) : # check drive timeout 15secs
            #print("in drive_speed_distance, encoder1 ",encoder_1())
            move = True
    elif speed < 0 :
        while ((encoder_1()) > (max_encoder - required_distance_encoder_value) \
        or (encoder_2()) > (max_encoder - required_distance_encoder_value))\
        and ((time.time() - time_started_drive) < drive_timeout_time) : # check drive timeout 15secs:
            move = True
    if (time.time() - time_started_drive) > drive_timeout_time :
        move = False
        drive_time_out = True
        print ("Calculated drive time-out = ", drive_timeout_time)
        print ("Drive timed out in ",(time.time() - time_started_drive)," secs")

    else :
        move = False
    # demanded distance achieved, stop motors
    motor_stop()
    #print ("Calculated drive time-out = ", drive_timeout_time)
    #print ("Time for drive = ", (time.time() - time_started_drive), " secs")
    #print ("Finished drive_speed_distance")
    #print ("encoder 1  value = ",encoder_1())
    #print ("encoder 2  value = ",encoder_2())
    return

#_______________________________________________________________________________
def turn_speed_angle(speed, angle):
    global turn_timeout_time
    global wheelspace
    global wheel_diameter
    # nominal rotation speed of robot at speed 32 = 37.5 degrees/s (at speed 32)
    # adjust for accel/decel times etc and add a 2 second margin
    if speed != 0 :
        turn_timeout_time = (((angle * 32) / 60) /abs(speed)) +3
    else :
        turn_timeout_time = 15.0 #default drive timeout time
    
    turn_time_out = False
    # convert angle to an encoder value
    # Rotation disc circumference  = wheelspace * pi
    # wheel circumference = wheel_diameter * pi
    # number of wheel rotations per 360 degree turn = disc circ / wheel circ
    # This also equals encoder value per  1 degree of turn
    # optimised for speed 16
    print ("I am in turn_speed_angle")
    if angle == 0 :
        return
    angle_encoder = float(wheelspace / wheel_diameter)
    # reset encoders to 0
    reset_both_encoders()
    time.sleep(.1)
    # call function to turn on motors and start turn
    # central axis turn at rate set by speed
    time_started_turn = time.time() # gets time when turn was started
    drive_sync(0, speed)
    time.sleep(.3)
    # select encoder which has increasing value
    if speed < 0 :
    # correction factor for clockwise turn Default set to 1
        angle = angle / 1.0
        while encoder_2() < int(angle_encoder * angle) \
        and ((time.time() - time_started_turn) < turn_timeout_time) : # check drive timeout 10secs:
            move = True
        motor_stop()
        time.sleep(.1)
    elif speed > 0 :
    # correction factor for anti-clockwise turn. 1 = no correction
        angle = angle / 1.0
        while encoder_1() < int(angle_encoder * angle)\
        and ((time.time() - time_started_turn) < turn_timeout_time) : # check drive timeout 10secs:
            move = True
        motor_stop()
        time.sleep(.1)

    if (time.time() - time_started_turn) > turn_timeout_time : #set turn_time_out_flag
        motor_stop()
        move = False
        turn_time_out = True
        print ("Turn timed out in ",(time.time() - time_started_turn)," secs")
        return
    else :
        move = False
        motor_stop()
    #print ("Calculated drive time-out = ", turn_timeout_time)
    #print ("Time for drive = ", (time.time() - time_started_turn), " secs")
    return

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#                       SENSOR RELATED FUNCTIONS
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#_______________________________________________________________________________

#                           SONAR
#       front_sonar() SRF08(Also reads light sensor value in register 1)
# This function sets the SRF08 rangefinder to distances in cms
# x55 selects I2C, SRF08 base address xE0, command reg x00, 1byte, x51=ranging cms
# Register 01 Light sensor value / gain R/W
# Register 02/03H - 22/23H target range, nearest - furthest, hi byte/lo byte
# Currently set to read value for target 1. Other targets can be enabled
# returns actual distance recorded. Subtract front_sonar_offset to get distance to front of robot

def front_sonar(): # Assumes base address of E0 (write), E1 (read)
    # trigger ranging
    ser.write( b"\x55\xE0\x00\x01\x51" )
    n = ser.read(1)  #get acknowledge
    time.sleep(.1)
    # set I2C device to read and read two bytes from register 2/3 (target 1)
    ser.write( b"\x55\xE1\x02\x02" ) # 2 bytes, register auto-increments
    n = (ser.read(2))
    decoded_n = n.decode
    #print ("n undecoded = ",n)
    targ1_hi = (n[0])
    targ1_lo = (n[1])
    front_sonar_range = targ1_lo + (targ1_hi << 8)
    print ("front sonar range  = ", front_sonar_range)
    return front_sonar_range

#_______________________________________________________________________________
# KCH LED routines
def LED_B_red():
    robot.kch.leds[LED_B].colour = Colour.RED
def LED_B_blue():
    robot.kch.leds[LED_B].colour = Colour.BLUE
def LED_B_green():
    robot.kch.leds[LED_B].colour = Colour.GREEN 
def LED_B_off():
    robot.kch.leds[LED_B].colour = Colour.OFF    
def LED_A_blue():
    robot.kch.leds[LED_A].colour = Colour.BLUE
def LED_A_off():
    robot.kch.leds[LED_A].colour = Colour.OFF
def LED_C_blue():
    robot.kch.leds[LED_C].colour = Colour.BLUE    
def LED_C_green():
    robot.kch.leds[LED_C].colour = Colour.GREEN   
def LED_C_red():
    robot.kch.leds[LED_C].colour = Colour.RED
def LED_C_off():
    robot.kch.leds[LED_C].colour = Colour.OFF


def distance_ultrasound():
    return arduino.ultrasound_measure(2,3) # ultrasound pins might need to be changed

#_______________________________________________________________________________
#               camera_pan(angle)
# sets camera horizontal angle based on passed angle parameter in degrees
# returns pan angle in degrees, corrected by camera_servo_offset (in fraction of degrees)
# camera servo on channel "0"
# 0 is nominal straight ahead but may require offset 1 = 0.5 degrees
# Max Pan angle in degrees  +50 (right), -50 (left).
# Max values allowed by SR servo routine (-1 to +1)
# Set up servo parameters using  R.servo_board.servos[0].set_duty_limits(800,2200)
# Currently set to 500,2500

def camera_pan(angle):  # Camera servo = servo [0]. Servo Board= "0LX2M". Resolution ~ 0.5 degrees
    global cam_angle
    camera_servo_offset = (camera_servo_offset_value/100)*1.5  # value of 1 degree = 0.02
    servo_angle = (angle/100)*1.5  + camera_servo_offset # converts angle to range of +/- 1.0
    if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
        servo_angle = 1
    if servo_angle <= -1: # Set max range for servo used and SR max allowed value
        servo_angle = -1
    robot.servo_board.servos[0].position = servo_angle
    cam_angle = round(angle,1)
    time.sleep(.1) # allow time to move
    #print("Cam pan angle = ",cam_angle)
    return cam_angle

def servo_02_pan(angle):  #MS24, 270degree servo, 20kg
    global cam_angle
    camera_servo_offset = (camera_servo_offset_value/100)*.75  # value of 1 degree = 0.02
    servo_angle = (angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
    if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
        servo_angle = 1
    if servo_angle <= -1: # Set max range for servo used and SR max allowed value
        servo_angle = -1
    robot.servo_board.servos[2].position = servo_angle
    cam_angle = round(angle,1)
    time.sleep(.1) # allow time to move
    #print("Cam pan angle = ",cam_angle)
    return cam_angle

def play(melody, pause=0.06):
    for note, dur in melody:
        if note is None:
            time.sleep(dur)  # rest
        else:
            robot.power_board.piezo.buzz(note, dur)
        time.sleep(pause)

def startup_jingle():
    # The actual jingle sequence
    jingle = [
        # motif 1
        (Note.E6, 0.18),
        (Note.G6, 0.18),
        (Note.B6, 0.24),
        (Note.E7, 0.36),

        # motif 2
        (Note.D7, 0.22),
        (Note.C7, 0.22),
        (Note.B6, 0.30),

        # little rest
        (None, 0.12),

        # motif 3 (staccato)
        (Note.E6, 0.12),
        (Note.E6, 0.12),
        (Note.G6, 0.12),
        (Note.E6, 0.24),

        # final flourish
        (Note.B6, 0.18),
        (Note.D7, 0.18),
        (Note.E7, 0.4),
    ]
    play(jingle, pause=0.06)


#_______________________________________________________________________________

         #End of defined functions
# xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

#--------------------------Initialisation---------------------------------------
I2C_device_detected = False
# Look for the USB-ISS module. Initialises serial port using USB-ISS adaptor then communicates with the MD25
port, desc = find_ports("USB-ISS.") # Specifically searches for USB-ISS adaptor board
if port != "not detected": # i.e USB-ISS has been detected
    ser = serial.Serial(port,115200)    # Set up serial communications (ser) with USB-ISS module
    print ("USB-ISS adaptor detected")  # Sets up port name and sets baud rate to 115200
    USB_ISS_detected = True
    ser.write( b"\x5A\x03" ) # Specific command to obtain USB-ISS S/No. 8 bytes returned
    n = (ser.read(8))        # Reads back 8 bytes
    print("USB-ISS serial No. ",n)
    print ("USB-ISS adaptor OK, but if an error is now detected then there is a problem talking to the MD25")
    print ("Attempting communication with MD25")
    
    # Set up MD25
    motor_stop() # stop main drive motors
    set_acel_rate(acelrate) # ensure accel rate set to safe value
    dis_2sec_timeout() # disable motor time-outs
    print ("MD25 initialisation complete")
    
else:
    print ("USB-ISS not detected")
    print ("Will not attempt to access the adaptor or MD25 drive motors")
    print ("investigate USB-ISS connections")
    USB_ISS_detected = False

robot.servo_board.servos[0].set_duty_limits(500,2500) # Set up servo parameters
robot.servo_board.servos[2].set_duty_limits(500,2500) # Set up servo parameters
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
#robot.wait_start() #        Waiting for start button to be pressed
#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

#my_corner = 0
#my_corner = robot.zone #set corner in robot set-up
#robot_mode = robot.mode # returns DEV or COMP (no parentheses)

#&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
# *********************** Relocated R.wait_start() ****************************
time_elapsed = time.time() - time_started_robot    # Time from initial switch on for initialisatiom 
print ("Initialised in ",round(time_elapsed,0)," seconds")
time_started_game = time.time()   #  Get time when START switch pressed
print("Game timer started")
print("")
print("")

#_______________________________________________________________________________
#*******************************************************************************
# Set-up completed. Place your game code here
#*******************************************************************************
foundID = False

while True:
    markers = robot.camera.see()
    for marker in markers:
        if marker.id == 1:
            angle = (marker.position.horizontal_angle)*(180/pi)
            foundID = True
            break
        else:
            LED_C_red()
    if foundID == True:
        break

originalAngle = angle
defaultSpeed = 5

startup_jingle()
robot.wait_start()

while(angle<90):


    if(abs(angle)<15):
        LED_C_off()
        LED_A_off()
        LED_B_red()
    elif(angle<-15):
        LED_A_blue()
        LED_B_off()
        LED_C_off()
    elif(angle>15):
        LED_C_blue()
        LED_A_off()
        LED_B_off()

    turn_speed_angle(-defaultSpeed, 5)
    markers = robot.camera.see()

    for marker in markers:
        if marker.id == 1:
            angle = (marker.position.horizontal_angle)*(180/pi)
            break
    
while(angle>-90):
    if(abs(angle)<15):
        LED_C_off()
        LED_A_off()
        LED_B_red()
    elif(angle<-15):
        LED_A_blue()
        LED_B_off()
        LED_C_off()
    elif(angle>15):
        LED_C_blue()
        LED_A_off()
        LED_B_off()

    turn_speed_angle(defaultSpeed, 5)
    markers = robot.camera.see()

    for marker in markers:
        if marker.id == 1:
            angle = (marker.position.horizontal_angle)*(180/pi)
            break
    
while(angle<0):
    if(abs(angle)<15):
        LED_C_off()
        LED_A_off()
        LED_B_red()
    elif(angle<-15):
        LED_A_blue()
        LED_B_off()
        LED_C_off()
    elif(angle>15):
        LED_C_blue()
        LED_A_off()
        LED_B_off()

    turn_speed_angle(-defaultSpeed, 5)
    markers = robot.camera.see()

    for marker in markers:
        if marker.id == 1:
            angle = (marker.position.horizontal_angle)*(180/pi)
            break

#to fix - LEDs A and C are swapped; the robot won't stop going in circles.