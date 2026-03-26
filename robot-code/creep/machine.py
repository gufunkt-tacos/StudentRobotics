# 15 Feb 2026
# # Base I2c (8 bit) default address for the MD25 = Black xB2(xB3 read) [MD25 = Blue xB0 (xB1 read)]
# Add the ID codes e.g "SR0HG1B" for your specific SR modules
# Be careful with case, upper or lower and 0's O's etc. Copy from your log file if unsure
# Camera Board (046d:0825) - 0 
# Power Board   sr0HR20
# Motor board   SR0QBJ 
# Motor board   SR0VJ1K  M0: Vacuum Pump,  M1: ArmExtend 
# Ruggeduino    serial: 75230313833351314151
# Servo Board   sr0LX2M
# USB-ISS adaptor /dev/ttyACM0

#You will need a WiFi key to be able to connect
#You can find this inside robot-settings.toml on the USB drive containing your code.

#team_tla = "ZZZ23956"
#usercode_entrypoint = "robot.py"
#wifi_psk = "992e-8703-3240"
#wifi_region = "GB"
#wifi_enabled = true

# WIFI connection to a laptop/PC or tablet. Search for wifi sources. Use SR password then open the following in a browser
# http://robot.lan   

from enum import Flag, auto

from sr.robot3 import * # New SR code based on Python 3.11 (I'm using 3.12.2)
import time
import serial   # ignore import error if you get one
import math
import numpy as np
from math import cos, sin, pi, sqrt, atan2
import serial.tools.list_ports
import statistics

# These coordinates are correct for the 2025-2026 game. List starts at marker '0' to marker 19
MARKER_COORDS = \
    [[76,0],[153,0],[229,0],[305,0],[381,0] \
    ,[458,76],[458,153],[458,229],[458,305],[458,381] \
    ,[381,458],[305,458],[229,458],[153,458],[76,458] \
    ,[0,381],[0,305],[0,229],[0,153],[0,76]]

# Wall: list of x,y coordinates of the markers along each wall
# Each marker is spaced 76.25±20 mm apart and there are 5 on each wall
# Converted to cm
ARENA_MARKER_COORS = {
    0: {0: (76.25, 457.5), 1: (152.5, 457.5), 2: (228.75, 457.5), 3: (305.0, 457.5), 4: (381.25, 457.5)},
    1: {5: (457.5, 381.25), 6: (457.5, 305.0), 7: (457.5, 228.75), 8: (457.5, 152.5), 9: (457.5, 76.25)},
    2: {10: (381.25, 0.0), 11: (305.0, 0.0), 12: (228.75, 0.0), 13: (152.5, 0.0), 14: (76.25, 0.0)},
    3: {15: (0.0, 76.25), 16: (0.0, 152.5), 17: (0.0, 228.75), 18: (0.0, 305.0), 19: (0.0, 381.25)}
}

def get_marker_wall(id):
    return id // 5



get_marker_coords = lambda marker_id: ARENA_MARKER_COORS[marker_id // 5][marker_id]

class ObjectType(Flag):
    TOKEN = auto()
    ACID = auto()
    BASE = auto()

    ARENA_MARKER = auto()

    IGNORE = auto()

class Object:
    def __init__(self, id: int, position: float, h_angle: int, v_angle: int):
        self.id: int = id
        self.position: float = position
        self.h_angle: int = h_angle
        self.v_angle: int = v_angle
        self.type: ObjectType = get_type_from_id(id)

    def __repr__(self) -> str:
        return f"Object(id={self.id}, position={self.position}, h_angle={self.h_angle}, v_angle={self.v_angle}, type={self.type})"
    
    def __str__(self) -> str:
        return f"Object(id={self.id}, position={self.position}, h_angle={self.h_angle}, v_angle={self.v_angle}, type={self.type})"

def get_type_from_id(id: int) -> ObjectType:
    if id >= 0 and id <=19:
        return ObjectType.ARENA_MARKER
    elif id >= 100 and id <=139:
        return ObjectType.ACID | ObjectType.TOKEN
    elif id >= 140 and id <=179:
        return ObjectType.BASE | ObjectType.TOKEN
    else:
        return ObjectType.IGNORE


class CreepRobot():

    #robot.servo_board.servos[0].set_duty_limits(800,2200)
    #robot = Robot.ignored_arduinos=["7543535313835170F0B0"]
    #robot = Robot(wait_for_start=False, ignored_arduinos=["75830333338351803001"]) # New set-up "7543535313835170F0B0" 75830333338351803001
    #my_arduino = robot.arduinos["75230313833351314151"]
    #d2r = pi/180


    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #----------------------------------Defined functions----------------------------
    #__________________________________________________________________________

    #                         find_ports()
    # This routine examines the devices connected to the USB ports of the Pi4
    # The name of the device (e.g USB-ISS for the USB to I2C interface adaptor) is passed to the routine
    # If the device is found it prints out the tty.device_node, Vendor ID (VID) and Product ID (PID)

    corner_markers = [0,19,4,5,9,10,14,15]

    def find_ports(self, portname: str) -> tuple[str, str]:
        """
        This routine examines the devices connected to the USB ports of the Pi4.
        The name of the device (e.g USB-ISS for the USB to I2C interface adaptor) is passed to the routine.
        If the device is found it prints out the tty.device_node, Vendor ID (VID) and Product ID (PID).
        """

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
    def dis_2sec_timeout(self) -> None:
        """
        This function disables the MD25 motor controller 2 second timeout
        """

        # I2C interface uses 1 byte addressed device mode 0x55
        # x55 selects I2C, Blue MD25 base address xB0(write)xB1(read) , command reg x10, 1byte, disable x32
        # x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32


        resp = 0
        self.ser.write( b"\x55\xB2\x10\x01\x32" )  # b required to change unicode to bytes
        n = self.ser.read(1) #  get acknowledge, doesn't really matter about the format
        resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
        #print("response = ",resp)
        if resp != 0:
            print ("Motor timeout disabled")
        return


    def enables_2sec_timeout(self) -> None:
        """
        This function enables the MD25 motor controller 2 second timeout.
        """

        # x55 selects I2C, Blue MD25 base address xB0, command reg x10, 1byte, enable x33.
        # x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32.


        self.ser.write( b"\x55\xB2\x10\x01\x33" )  # b required to change unicode to bytes
        n = self.ser.read(1) # get acknowledge
        resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
        if resp != 0 :
            print ("Motor timeout enabled")
        return


    def reset_both_encoders(self) -> None:
        """
        This function resets both wheel encoder values to zero.
        """

        # x55 selects I2C, Blue MD25base address xB0, command reg x10, 1byte, reset encoders x20
        # x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , command reg x10, 1byte, disable x32

        resp = 0
        #print ("in reset_both encoders ")
        self.ser.write(b"\x55\xB2\x10\x01\x20")  # b required to change unicode to bytes
        #time.sleep(.1)
        n = self.ser.read(1) # get acknowledge
        resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
        if resp == 0 :
            print ("wheel encoders reset failed")
        if resp != 0 :
            print ("wheel encoders reset")
        time.sleep(.1)
        return



    # x55 selects I2C, MD25 base address xB0 (B1 to read), encoder data x02-x05, 1byte
    # x55 selects I2C, Orange MD25 base address xB2(write)xB3(read) , encoder data x02-x05, 1byte
    # x02 - 05  are the registers for the four encoder bytes
    # Read hibyte first to capture count for lower bytes
    # x01 is number of bytes (1) to be read from each register

    def encoder_1(self) -> int:
        """
        This function reads the 4 byte value from encoder 1 and converts to an integer.
        """

        #print "reading encoder 1"
        #MD25 is a single byte register device. Handles auto-increment of register address
        #select I2C device (x55), register to start read from (x02) and number of bytes (x04)
        self.ser.write( b"\x55\xB3\x02\x04" )  # b required to change unicode to bytes
        n = self.ser.read(4) # read four bytes corresponding to encoder1 (4 values)
        Enc1byte3 = (n[0]) # hi byte
        Enc1byte2 = (n[1])
        Enc1byte1 = (n[2])
        Enc1byte0 = (n[3]) # lo byte
        encoder1 = Enc1byte0 + (Enc1byte1 << 8) +(Enc1byte2 << 16) + (Enc1byte3 << 24)
        #print ("encoder1 =", encoder1)
        return encoder1

    def encoder_2(self) -> int:
        """
        This function reads the 4 byte value from encoder 2 and converts to an integer.
        """

        #print "reading encoder 2"
        #MD25 is a single byte register device. Handles auto-increment of address
        #select I2C device (x55), register to start read from (x06) and number of bytes (x04)
        self.ser.write( b"\x55\xB3\x06\x04" )  # b required to change unicode to bytes
        n = self.ser.read(4) # read four bytes corresponding to encoder2 (4 values)
        Enc2byte3 = (n[0])
        Enc2byte2 = (n[1])
        Enc2byte1 = (n[2])
        Enc2byte0 = (n[3])
        encoder2 = Enc2byte0 + (Enc2byte1 << 8) +(Enc2byte2 << 16) + (Enc2byte3 << 24)
        #print ("encoder2 =", encoder2)
        return encoder2


    def set_acel_rate(self, acelrate) -> None:
        """
        This function sets the motor acceleration rate, range 1 to 10 fastest
        Do not set to higher than 5 (default)
        """

        # x55 selectsI2C, Blue MD25 base address xB0, command accel rate reg x10, 1byte, accelrate
        # x55 selectsI2C, Orange MD25 base address xB2, command accel rate reg x10, 1byte, accelrate
        # x03 sets rate to 3 full revers to full forward in 2.1 seconds

        self.ser.write(bytes([0x55, 0xB2, 0x10, 0x01, acelrate]))
        n = self.ser.read(1) # get acknowledge
        resp = str(n[0]) # 0 is not OK (0) !0 is OK (1)
        time.sleep(0.1)
        if resp != 0 :
            print ("Accel rate set at ", acelrate)
        return



    def drive_both(self, speed1: int, speed2: int) -> None:
        """
        Independant control of both motors. Encoders reset at start of routine.

        Speed 1,2 values in the range -128 to +127
        """
        # Send a command to write to speed1 and speed 2
        # x55 selects I2C, Blue MD25 base address xB0, mode reg x0F, 1bytes, x00 = mode 0
        # x55 selects I2C, Orange MD25 base address xB2, mode reg x0F, 1bytes, x00 = mode 0
        # last two values are speed values e.g x70, x90. In default mode0:
        # 0(0H) = full reverse, 128(80H) stop, 255(FFH) full forward
        # but this function converts from -128 reverse 0, stop +127 forward


        self.reset_both_encoders()
        # Convert passed variables to +ve integers in the range 0 to 255
        # +ve speed = forward, +ve turn = clockwise
        speed1 = speed1 + 128
        speed2 = speed2 + 128
        #Set up mode register (15, x0F) for mode 0,Independant control of motors, no sync
        self.ser.write( b"\x55\xB2\x0F\x01\x00" )  # b required to change unicode to bytes
        self.ser.read(1) # get acknowledge
        #Write to registers speed 1 (speed1) and speed 2 (speed2)
        self.ser.write(bytes([0x55, 0xB2, 0x00, 0x02, speed1, speed2]))
        self.ser.read(1) # get acknowledge
        return



    def motor_stop(self) -> None:
        """
        Stop both motors.
        """
        self.ser.write( b"\x55\xB2\x00\x02\x80\x80" )  # b required to change unicode to bytes
        self.ser.read(1) # get acknowledge
        time.sleep(.1)
        # reset encoders elsewhere if required
        return



    def drive_sync(self, speed: int, turn: int) -> None:
        """
        Synchronised speed on both motors.

        Resets both encoders at beginning of routine.
        """

        # First set up mode register (15, x0F) for mode 2
        # Send a command to write to speed1 for speed and speed 2 for turn (0 if no turn required)
        # x55 selects I2C, MD25 base address xB2, mode reg x0F, 1byte, x02 = mode 2
        # Then write to speed 1 and 2 as per drive_both



        # This sets up MD25 register for speed turn. Turn will continue until motor stop commanded
        # Under MD25 control of speed only. Does not use encoders
        # If turn set to 0 then speed selects forward & reverse speeds
        # If speed set to 0 then robot will turn about central axis at rate set by turn value
        # If speed and turn = 0 then robot will stop
        # Set speed in the range 0 to 127 forward, 0 to -128 reverse
        # Set turn for rate of turn, 0 to 127 clockwise, 0 to -128 anticlock
        # Convert passed variables to +ve integers in the range 0 - 255
        # +ve speed = forward, +ve turn = clockwise
        self.reset_both_encoders() # may not be necessary
        #print ("in drive_sync(speed,turn):speed ",speed, " turn ", turn)
        #reset encoders elsewhere if required. Encoders not used in this mode
        if speed not in range(-128, 128):
            speed = 127
        if turn not in range(-128, 128):
            turn = 127
        speed = speed + 128
        turn = turn + 128
        #Set up mode register (15, x0F) for mode 2
        #Synchronises speed of both motors
        self.ser.write( b"\x55\xB2\x0F\x01\x02" )  # b required to change unicode to bytes
        self.ser.read(1) # get acknowledge
        time.sleep(.1)
        #Write to registers speed 1 (speed) and speed 2 (turn)
        #ser.write( "\x55\xB2\x00\x02"  + chr(speed) + chr(turn))
        self.ser.write(bytes([0x55, 0xB2, 0x00, 0x02, speed, turn]))
        self.ser.read(1) # get acknowledge
        return




    def drive_speed_distance(self, speed: int, distance: float) -> bool:
        """
        Drive in a straight line at a defined speed (-128 to +127)

        Should be used in CENTIMETRES NOT MILLIMETRES AAAAAAARGH.

        Resets both encoders at start of routine.
        Drive in a straight line at a defined speed (-128 to +127).
        Uses synchronised speed mode.
        Drive times out after a calculated time is exceeded.
        """

        # nominal speed of robot at speed 32 = 25cm/s (at speed 32)
        # adjust for accel/decel times etc and add a 1 second margin
        if speed != 0:
            drive_timeout_time = (((distance * 32) / 22) /abs(speed)) + 2
        else:
            drive_timeout_time = 15.0 #default drive timeout time
        if distance <= 0: # Only +ve values of distance allowed
            return False
        
        print("in drive_speed_distance. Will reset both encoders.")
        # convert distance to an encoder value
        required_distance_encoder_value = int((distance / (self.wheel_diameter * pi)) * 360)
        time.sleep(.1)
        # start motors
        # drives with 0 turn at speed set by "speed"
        time_started_drive = time.time() # gets time when drive was started
        self.drive_sync(speed, 0) # start motors
        # THIS DELAY IS ESSENTIAL TO CORRECT OPERATION, OR IS IT ??????
        time.sleep(.1)
        #If speed > 0 (OK but if < 0 then need to correct encoder for -ve values)
        if speed > 0 :
            while (self.encoder_1() < required_distance_encoder_value \
            or self.encoder_2() < required_distance_encoder_value)\
            and ((time.time() - time_started_drive) < drive_timeout_time) : # check drive timeout 15secs
                #print("in drive_speed_distance, encoder1 ",encoder_1())
                pass
        elif speed < 0 :
            while ((self.encoder_1()) > (self.max_encoder - required_distance_encoder_value) \
            or (self.encoder_2()) > (self.max_encoder - required_distance_encoder_value))\
            and ((time.time() - time_started_drive) < drive_timeout_time) : # check drive timeout 15secs:
                pass
        if (time.time() - time_started_drive) > drive_timeout_time :
            print ("Calculated drive time-out = ", drive_timeout_time)
            print ("Drive timed out in ",(time.time() - time_started_drive)," secs")
            return False
        # demanded distance achieved, stop motors
        self.motor_stop()
        #print ("Calculated drive time-out = ", drive_timeout_time)
        #print ("Time for drive = ", (time.time() - time_started_drive), " secs")
        #print ("Finished drive_speed_distance")
        #print ("encoder 1  value = ",encoder_1())
        #print ("encoder 2  value = ",encoder_2())
        return True
    

    
    def drive_actual_speed_distance(self, speed: float, distance: float) -> bool:
        """
        Speed is measured in mm/s

        Distance is measured in mm

        Equivalent in practice to drive_steprate_distance
        """

        maxSpeed = 2000
        if abs(speed) > maxSpeed:
            speed = (speed/abs(speed))*maxSpeed
        self.steprate = int((255 / maxSpeed) * speed) - 128
        return self.drive_speed_distance(self.steprate, distance)


        


    def turn_speed_angle(self, speed: int, angle: float) -> bool:
        """
        Speed is defined between -128 (CCW) to +127 (CW)

        Angle is in degrees
        """

        # nominal rotation speed of robot at speed 32 = 37.5 degrees/s (at speed 32)
        # adjust for accel/decel times etc and add a 2 second margin
        if speed != 0 :
            turn_timeout_time = (((angle * 32) / 60) /abs(speed)) + 3
        else :
            return False
        
        # convert angle to an encoder value
        # Rotation disc circumference  = wheelspace * pi
        # wheel circumference = wheel_diameter * pi
        # number of wheel rotations per 360 degree turn = disc circ / wheel circ
        # This also equals encoder value per  1 degree of turn
        # optimised for speed 16
        print ("I am in turn_speed_angle")
        if angle == 0 :
            return False
        angle_encoder = self.wheelspace / self.wheel_diameter
        # reset encoders to 0
        self.reset_both_encoders()
        time.sleep(.1)
        # call function to turn on motors and start turn
        # central axis turn at rate set by speed
        time_started_turn = time.time() # gets time when turn was started
        self.drive_sync(0, speed)
        time.sleep(.3)
        # select encoder which has increasing value
        if speed < 0 :
            while self.encoder_2() < angle_encoder * angle \
            and ((time.time() - time_started_turn) < turn_timeout_time) : # check drive timeout 10secs:
                pass
            self.motor_stop()
            time.sleep(.1)
        elif speed > 0 :
            while self.encoder_1() < angle_encoder * angle\
            and ((time.time() - time_started_turn) < turn_timeout_time) : # check drive timeout 10secs:
                pass
            self.motor_stop()
            time.sleep(.1)

        if (time.time() - time_started_turn) > turn_timeout_time : #set turn_time_out_flag
            self.motor_stop()
            print ("Turn timed out in ",(time.time() - time_started_turn)," secs")
            return False
        
        self.motor_stop()
        #print ("Calculated drive time-out = ", turn_timeout_time)
        #print ("Time for drive = ", (time.time() - time_started_turn), " secs")
        return True

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    #                         Grab Arm related funcyions

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    def VacPump(self, power):
        self.robot.motor_boards["SR0VJ1K"].motors[0].power = power


    def VacValve(self, on_off):
        if (on_off)== "VENT":
            self.robot.power_board.outputs[OUT_FIVE_VOLT].is_enabled = True
        else:
            self.robot.power_board.outputs[OUT_FIVE_VOLT].is_enabled = False

    def Arm_Extend(self, power): #(Pass variable power. Range (0 to 1))
        self.robot.motor_boards["SR0VJ1K"].motors[1].power = -power

    def Arm_Retract(self, power):
        self.robot.motor_boards["SR0VJ1K"].motors[1].power = power

    def Arm_Stop(self):
        self.robot.motor_boards["SR0VJ1K"].motors[1].power = 0

    def Arm_tilt(self, position):  # Servo port [1] MS24, 270 degree servo, 20kg
        # -120 degrees = max up, +60 degrees max down
        global Arm_tilt_angle
        if (position) == "UP":
            arm_angle = -120
        elif (position) == "DOWN":
            arm_angle = 80
        elif (position) == "LEVEL":
            arm_angle = 0
        else:
            arm_angle = 0     
        servo_angle = (arm_angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[1].position = servo_angle
        Arm_tilt_angle = round(arm_angle,1)
        time.sleep(.1) # allow time to move
        #print("Arm tilt angle = ",Arm_tilt_angle)
        return Arm_tilt_angle    

    def Arm_tilt_up(self):  # Servo port [1] MS24, 270 degree servo, 20kg
        # -120 degrees = max up, +60 degrees max down
        global Arm_tilt_angle
        arm_angle = -120
        servo_angle = (arm_angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[1].position = servo_angle
        Arm_tilt_angle = round(arm_angle,1)
        time.sleep(.1) # allow time to move
        #print("Arm tilt angle = ",Arm_tilt_angle)
        return Arm_tilt_angle 

    def Arm_tilt_down(self):  # Servo port [1] MS24, 270 degree servo, 20kg
        # -120 degrees = max up, +60 degrees max down
        global Arm_tilt_angle
        arm_angle = 80
        servo_angle = (arm_angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[1].position = servo_angle
        Arm_tilt_angle = round(arm_angle,1)
        time.sleep(.1) # allow time to move
        #print("Arm tilt angle = ",Arm_tilt_angle)
        return Arm_tilt_angle 

    def Arm_tilt_level(self):  # Servo port [1] MS24, 270 degree servo, 20kg
        # -120 degrees = max up, +60 degrees max down
        global Arm_tilt_angle
        arm_angle = 0
        servo_angle = (arm_angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[1].position = servo_angle
        Arm_tilt_angle = round(arm_angle,1)
        time.sleep(.1) # allow time to move
        #print("Arm tilt angle = ",Arm_tilt_angle)
        return Arm_tilt_angle    
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

    def speedofsound(self, temp:float = 18) -> float:
        """
        Returns the speed of sound by using external temperature
        """
        return 331.3 * sqrt( 1 + ( temp / 273.15 ))

    
    def left_front_sonar(self, samples=1) -> float:
        """
        Returns distance in cm from left sensor. 
        
        For more accuracy several samples can be taken which will be averaged together.
        """

        readings = []

        for _ in range(samples):
            # trigger in cm
            self.ser.write(b"\x55\xe0\x00\x01\x52")
            self.ser.read(1)  # acknowledge

            # wait until measurement complete
            while True:
                self.ser.write(b"\x55\xe1\x00\x01")  # read register 0
                status = self.ser.read(1)
                if status != b'\xff':          # 0xFF means busy
                    break
                time.sleep(0.01)

            # Read range registers 2 & 3
            self.ser.write(b"\x55\xe1\x02\x02")
            n = self.ser.read(2)

            targ1_hi = n[0]
            targ1_lo = n[1]
            distance = (targ1_lo + (targ1_hi << 8))* self.speedofsound() / 10000 / 2

            readings.append(distance)

            time.sleep(0.065)  # minimum safe delay between pings

        distance = statistics.mean(readings)
        # print("Right sonar: " + str(distance))
        # maybe use the median? might have better noise rejection
        return distance

    def right_front_sonar(self, samples=1) -> float:
        """
        Returns distance in cm from right sensor. 
        
        For more accuracy several samples can be taken which will be averaged together.
        """


        readings = []

        for _ in range(samples):
            # trigger in cm
            self.ser.write(b"\x55\xe2\x00\x01\x52")
            self.ser.read(1)  # acknowledge

            # wait until measurement complete
            while True:
                self.ser.write(b"\x55\xe3\x00\x01")  # read register 0
                status = self.ser.read(1)
                if status != b'\xff':          # 0xFF means busy
                    break
                time.sleep(0.01)

            # Read range registers 2 & 3
            self.ser.write(b"\x55\xe3\x02\x02")
            n = self.ser.read(2)

            targ1_hi = n[0]
            targ1_lo = n[1]
            distance = (targ1_lo + (targ1_hi << 8))* self.speedofsound() / 10000 / 2

            readings.append(distance)

            time.sleep(0.065)  # minimum safe delay between pings

        distance = statistics.mean(readings)
        # print("Right sonar: " + str(distance))
        # maybe use the median? might have better noise rejection
        return distance

    def rear_sonar(self): # base address of F0 (write), F1 (read)
        # trigger ranging
        self.ser.write( b"\x55\xF0\x00\x01\x51" )
        n = self.ser.read(1)  #get acknowledge
        time.sleep(.1)
        # set I2C device to read and read two bytes from register 2/3 (target 1)
        self.ser.write( b"\x55\xF1\x02\x02" ) # 2 bytes, register auto-increments
        n = self.ser.read(2)
        # decoded_n = n.decode() # this code wasnt be used by anything and caused it to crash
        #print ("n undecoded = ",n)
        targ1_hi = (n[0])
        targ1_lo = (n[1])
        rear_sonar_range = targ1_lo + (targ1_hi << 8)
        print ("rear sonar range  = ", rear_sonar_range)
        return rear_sonar_range

    #_______________________________________________________________________________
    # Sharp IR range sensor GP2Y0E03
    # Base address 0x60H(sucker height)
    # x55 selects I2C, GP2Y0E03(height) base address 0x60, register_shift_bit=0x35,register_distance= 0x5e,0x5f
    def sucker_height(self):
        """
        Returns height of sucker using infrared sensor.
        
        Should be in cm
        
        """
        #print "in ir_sensor height"
        ir_distance = 0
        self.ser.write(b"\x55\x60\x5e")
        #read from register x5e
        self.ser.write(b"\x55\x61\x5e\x01")
        n = self.ser.read(1)
        dist_hi = (n[0])
        time.sleep(.1)
        self.ser.write(b"\x55\x60\x5f")
        self.ser.write(b"\x55\x61\x5f\x01")
        n = self.ser.read(1)
        dist_lo = (n[0])
        time.sleep(.1)
        self.ser.write(b"\x55\x60\x35")
        self.ser.write(b"\x55\x61\x35\x01")
        n = self.ser.read(1)
        SBit =  (n[0])
        ir_distance = ((((dist_hi * 16) + dist_lo) / 16 )/ math.pow(2,SBit))
        #print "IR distance = ", ir_distance
        return ir_distance

    def sucker_pressure(self) -> int:
        """
        Returns vacuum pressure on the sucker
        """
        return int(self.robot.arduino.command("z"))
    
    def sucker_gripping(self) -> bool:
        """
        Returns True if the sucker is gripping an object, False otherwise.
        """
        return self.sucker_pressure() < 50


    #_______________________________________________________________________________
    # KCH LED routines
    def LED_B_red(self):
        self.robot.kch.leds[LED_B].colour = Colour.RED
    def LED_B_blue(self):
        self.robot.kch.leds[LED_B].colour = Colour.BLUE
    def LED_B_green(self):
        self.robot.kch.leds[LED_B].colour = Colour.GREEN 
    def LED_B_off(self):
        self.robot.kch.leds[LED_B].colour = Colour.OFF    
    def LED_A_blue(self):
        self.robot.kch.leds[LED_A].colour = Colour.BLUE
    def LED_A_off(self):
        self.robot.kch.leds[LED_A].colour = Colour.OFF
    def LED_C_blue(self):
        self.robot.kch.leds[LED_C].colour = Colour.BLUE    
    def LED_C_green(self):
        self.robot.kch.leds[LED_C].colour = Colour.GREEN   
    def LED_C_red(self):
        self.robot.kch.leds[LED_C].colour = Colour.RED
    def LED_C_off(self):
        self.robot.kch.leds[LED_C].colour = Colour.OFF

    def toggle_LED(self, led: int, colour: Colour):
        if self.robot.kch.leds[led].colour != colour:
            self.robot.kch.leds[led].colour = colour


    #def distance_ultrasound():
    #    return arduino.ultrasound_measure(2,3) # ultrasound pins might need to be changed

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

    def camera_pan(self, angle):  # Camera servo = servo [0]. Servo Board= "0LX2M". Resolution ~ 0.5 degrees
        global cam_angle
        camera_servo_offset = (self.camera_servo_offset_value/100)*1.5  # value of 1 degree = 0.02
        servo_angle = (angle/100)*1.5  + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[0].position = servo_angle
        cam_angle = round(angle,1)
        time.sleep(.1) # allow time to move
        #print("Cam pan angle = ",cam_angle)
        return cam_angle

    def RH_door(self, angle):  # Servo port [7] MS24, 270degree servo, 20kg
        global RH_door_angle
        servo_angle = (angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[7].position = servo_angle
        RH_door_angle = round(angle,1)
        time.sleep(.1) # allow time to move
        #print("Cam pan angle = ",cam_angle)
        return RH_door_angle

    def LH_door(self, angle):  #  Servo port [5]MS24, 270degree servo, 20kg
        global LH_door_angle
        servo_angle = (angle/100)*.75 # + camera_servo_offset # converts angle to range of +/- 1.0
        if servo_angle >= 1:  # Set max range for servo used and SR max allowed value
            servo_angle = 1
        if servo_angle <= -1: # Set max range for servo used and SR max allowed value
            servo_angle = -1
        self.robot.servo_board.servos[5].position = servo_angle
        LH_door_angle = round(angle,1)
        time.sleep(.1) # allow time to move
        #print("Cam pan angle = ",cam_angle)
        return LH_door_angle

    def Doors_wedge(self):
        self.LH_door(-70)
        self.RH_door(60)
        time.sleep(.5)

    def Doors_open(self):
        self.LH_door(5)
        self.RH_door(-15)
        
    def Doors_open_LR(self):
        self.LH_door(5)
        time.sleep(.5)
        self.RH_door(-15)
        time.sleep(.5)

    def Doors_open_RL(self):
        self.RH_door(-15)
        time.sleep(.5)
        self.LH_door(5)
        time.sleep(.5)

    def Doors_close(self):
        self.LH_door(-93)    
        self.RH_door(90)
        
    def Doors_close_LR(self):
        self.LH_door(-93)    # close LH door
        time.sleep(.5)  # then close RH door
        self.RH_door(90)
        time.sleep(.5)  

    def Doors_close_RL(self):
        self.RH_door(90)    # close RH door
        time.sleep(.5)  # then close LH door
        self.LH_door(-93) 
        time.sleep(.5) 

    #                       find_objects()
    # This function locates all objects/markers currently seen by camera
    # pass "all", "acid_tokens", "base_tokens", "arena_marker"#_______________________________________________________________________________
    # 

    def find_objects(self, object_type: ObjectType) -> list[Object] | None:
        '''
        Finds all objects of certain type in the camera's vision
        '''
        angle_correction = 0
        camera_vertical_height = 44.5
        camera_horizontal_offset = 5

        self.robot.kch.leds[LED_A].colour = Colour.OFF
        self.robot.kch.leds[LED_B].colour = Colour.OFF
        self.robot.kch.leds[LED_C].colour = Colour.OFF
        
        objects: list[Object] = []

        markers = self.robot.camera.see()
        time.sleep(self.rseewaittime)

        for marker in markers:
            robot_distance = round(marker.position.distance/10,2)
            if ((robot_distance ** 2 ) - ((camera_vertical_height -0) ** 2)) <= 0:
                robot_distance = 1000 
            robot_distance_corrected = round(math.sqrt((robot_distance ** 2) - ((camera_vertical_height - 0) ** 2)) - camera_horizontal_offset,0)
            
            h_uncorrected_robot_angle = round(math.degrees(marker.position.horizontal_angle))
            h_robot_angle = h_uncorrected_robot_angle + angle_correction

            v_uncorrected_robot_angle = round(math.degrees(marker.position.vertical_angle))
            v_robot_angle = v_uncorrected_robot_angle + angle_correction

            objects.append(Object(marker.id, robot_distance_corrected, h_robot_angle, v_robot_angle)) 

        if len(objects) == 0:
            return None
        
        return  [o for o in objects if object_type in o.type]
    

    #_______________________________________________________________________________

    def play(self, melody, pause=0.06):
        for note, dur in melody:
            if note is None:
                time.sleep(dur)  # rest
            else:
                self.robot.power_board.piezo.buzz(note, dur)
            time.sleep(pause)

    def startup_jingle(self):
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
        self.play(jingle, pause=0.06)
    

    #       drive_speed_distance_objchk(speed, distance, object_detect_range)
    # Drive in a straight line at a defined speed (-128 to +127)
    # Drive for a defined distance in centimetres or obstruction detected (object_detect_range)
    # by front sonar (define left or right) Object detection uses rear sonar when reversing
    # Returns object detected, distance requested, distance driven and distance remaining
    # Drive times out after a calculated time is exceeded (speed, distance based)

    def drive_speed_distance_objchk(self, speed, distance, object_detect_range):
        # nominal speed of robot at speed 32 = 25cm/s (at speed 32)
        # adjust for accel/decel times etc and add a 1 second margin
        if speed != 0 :
            drive_timeout_time = (((distance * 32) / 22) /abs(speed)) +2
        else :
            drive_timeout_time = 15.0 #default drive timeout time
        if distance <= 0:
            distance = 0
            move = False
            return
        global max_encoder
        global wheel_diameter
        global drive_time_out
        global object_detected
        drive_time_out = False
        object_detected = False
        print ("in drive_speed_distance_objchk, will reset both encoders")
        object_detected = False
        range_detected = 0
        # convert distance to an encoder value
        required_distance_encoder_value = int((distance / (self.wheel_diameter * 3.142)) * 360)
        # reset encoders to 0
        self.reset_both_encoders()
        time.sleep(.1)
        # start motors
        time_started_drive = time.time() # gets time when drive was started
        self.drive_sync(speed, 0)
        time.sleep(.1) #important ??
        #If speed > 0 (OK but if <0 then need to correct encoder for -ve values)
        move = False
        encoder1 = self.encoder_1()
        if speed > 0 :
            while ((self.encoder_1() < required_distance_encoder_value \
            or self.encoder_2() < required_distance_encoder_value))\
            and ((time.time() - time_started_drive) < drive_timeout_time ):
        #while (encoder_1() < required_distance_encoder_value) and (front_sonar() > object_detect_range):
        #distance_driven = (encoder_1() * wheel_diameter * 3.142)/360
                range_detected_front_left = self.left_front_sonar()
                move = True
                if (range_detected_front_left <= object_detect_range):
                    object_detected = True
                    print ("sonar front left detected at ", range_detected_front_left)
                    print ("sonar front right detected at ", range_detected_front_right)
                    break
                
                range_detected_front_right = self.right_front_sonar()
                if (range_detected_front_right <= object_detect_range):
                    object_detected = True
                    print ("sonar front left detected at ", range_detected_front_left)
                    print ("sonar front right detected at ", range_detected_front_right)
                    break    

        elif speed < 0 :
            #print ("sonar rear ",rear_sonar())
            while(((self.encoder_1()) > (self.max_encoder - required_distance_encoder_value) \
            or (self.encoder_2()) > (self.max_encoder - required_distance_encoder_value)))\
            and ((time.time() - time_started_drive) < drive_timeout_time):
            #while (encoder_1()-0) > (max_encoder - required_distance_encoder_value):
                range_detected_rear = 10000
                range_detected_rear = self.rear_sonar()
                move = True
                        
                if range_detected_rear <= object_detect_range :
                    object_detected = True
                    print ("object detect range ", object_detect_range)
                    print ("sonar rear detected at ", range_detected_rear)
                    print ("object detected during loop")
                    break
            print (" I have exited the reverse drive_objchk loop " )               
        if (time.time() - time_started_drive) > drive_timeout_time :
            drive_time_out = True
            
            print ("Drive timed out in ",(time.time() - time_started_drive)," secs")
        
        else :
            move = False
        # demanded distance achieved or object detected, stop motors
        self.motor_stop()
        time.sleep(.1)
            
        if speed >0 :
            distance_driven = round(((self.encoder_1() * self.wheel_diameter * 3.142)/360),2)
            if object_detected == True:
                print ("object at front left ", range_detected_front_left)
                print ("object at front right", range_detected_front_right)
            
        elif speed < 0:
            distance_driven = round((((self.max_encoder - self.encoder_1()) * self.wheel_diameter * 3.142)/360),2)
            if object_detected == True:
                print("object at rear ", range_detected_rear)
            
        distance_to_go = round ((distance - distance_driven),2)
        #print "Calculated drive time-out = ", drive_timeout_time
        #print "Time for drive = ", (time.time() - time_started_drive), " secs"
        #print "distance ", distance
        #print "distance_driven ", distance_driven
        #print "distance to go ", distance_to_go
        
        return object_detected, range_detected, distance, distance_driven, distance_to_go 

    #_______________________________________________________________________________

            #End of defined functions
    # xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

    #--------------------------Initialisation---------------------------------------
    
        
    def __init__(self):
        self.time_started_robot = time.time()   # get time when ON/OFF switch pressed
        self.robot = Robot(wait_for_start=False)
        self.rseewaittime =.5
        self.camera_vertical_height = 48


        #""""""""""""""""""""""""""""" variables""""""""""""""""""""""""""""""""""

        #These variables need to be defined as global within the functions that use them
        self.wheelspace = 37.9 # 37.9 new robot,31.5 tracks,34.60 for old robot,36.75 for 2020 test base
        self.wheel_diameter = 10.8 #10.8 for new robot, 10 without tyres,5.10 tracks,10.50 for 10cm wheel with tyre
        self.max_encoder = 4294967295 # required when encoder value <0
        self.camera_servo_offset_value = -2 # +ve offset anti-clock
        #...............................................................................
        self.my_corner = 0 # Will be set by competition dongle with R.zone
        # The following variables are used by the MD25 motor drive board
        self.acelrate = 5 # DO NOT SET acelerate to > 5 unless using low speeds. Reset to < 5 after
        self.encoder1 = 0 # Set encoder for motor 1 to zero
        self.encoder2 = 0 # Set encoder for motor 2 to zero
        self.encoder1value = 0
        self.encoder2value = 0
        #initialise speed & turn variables to zero
        self.turn = 0        
        self.speed = 0
        self.speed1 = 0
        self.speed2 = 0
        self.drive_timeout_time = 15
        self.drive_time_out = False
        self.turn_timeout_time = 10
        self.turn_timeout = False
        self.I2C_device_detected = False

        print("finished CreepRobot __init__")
    
    def initialise(self, center_components=True):
        print("initialising CreepRobot")
        # Look for the USB-ISS module. Initialises serial port using USB-ISS adaptor then communicates with the MD25
        port, desc = self.find_ports("USB-ISS.") # Specifically searches for USB-ISS adaptor board
        if port != "not detected": # i.e USB-ISS has been detected
            self.ser = serial.Serial(port,115200)    # Set up serial communications (ser) with USB-ISS module
            print ("USB-ISS adaptor detected")  # Sets up port name and sets baud rate to 115200
            USB_ISS_detected = True
            self.ser.write( b"\x5A\x03" ) # Specific command to obtain USB-ISS S/No. 8 bytes returned
            n = (self.ser.read(8))        # Reads back 8 bytes
            print("USB-ISS serial No. ",n)
            print ("USB-ISS adaptor OK, but if an error is now detected then there is a problem talking to the MD25")
            print ("Attempting communication with MD25")
            
            # Set up MD25
            self.motor_stop() # stop main drive motors
            self.set_acel_rate(self.acelrate) # ensure accel rate set to safe value
            self.dis_2sec_timeout() # disable motor time-outs
            print ("MD25 initialisation complete")
            
        else:
            print ("USB-ISS not detected")
            print ("Will not attempt to access the adaptor or MD25 drive motors")
            print ("investigate USB-ISS connections")
            USB_ISS_detected = False

        #NB Servo board [6] appears faulty, do not use
        self.robot.servo_board.servos[0].set_duty_limits(500,2500) # Set up servo parameters
        self.robot.servo_board.servos[7].set_duty_limits(500,2500) # Set up servo parameters
        self.robot.servo_board.servos[5].set_duty_limits(500,2500) # Set up servo parameters
        self.robot.servo_board.servos[3].set_duty_limits(500,2500) # Set up servo parameters

        if (center_components):
            self.camera_pan(90)   # rotate camera to fit in 50 cm
            self.LH_door(-93)    # close LH door
            self.RH_door(90)     # close RH door
            #Arm_tilt("UP")  # Arm at max tilt
            self.Arm_tilt_up()   # Arm at max tilt
            self.Arm_Retract(1)  # Retract arm, full speed
            self.VacValve("GRIP") # Allow suction (valve unpowered)
            self.VacPump(0)      # Vac pump stopped
            print("doors closed,camera turned,arm raised/retracted, Vac & Valve 'OFF'")

        robot_mode = DEV

        # THIS SHOULD BE CHANGED DURING COMPETITION
        # my_corner = self.robot.zone #set corner in robot set-up
        # robot_mode = self.robot.mode # returns DEV or COMP (no parentheses)


        #             MUST CHECK THESE LINES FOR COMPETTION MODE
        #  FOR TEST PURPOSES ONLY +++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # This code is for safety but may be removed

        #my_lab = []
        if robot_mode == DEV:
            my_mode = "DEV"
            self.my_corner = 0  # Change the corner number for test purposes
            if self.my_corner==0:
                self.my_lab = [18,19,0]
            else:
                for i in range(0,3,1):
                    j=3+i+((self.my_corner-1)*5)
                    self.my_lab.append(j)
            
        else :
            my_mode = "COMP"
            print ("I am in",my_mode,"mode")
            self.my_corner = self.robot.zone
            if self.my_corner==0:
                self.my_lab = [18,19,0]
            else:
                for i in range(0,3,1):
                    j=3+i+((self.my_corner-1)*5)
                    self.my_lab.append(j)
            print ("my_lab ",self.my_lab)
        print ("I am in",my_mode,"mode")    
        print ("Selected starting corner = ",self.my_corner)
        print ("my_lab  ", self.my_lab)


        #&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        #robot.wait_start() #        Waiting for start button to be pressed
        #&&&&&&&&&&&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'&&'

        #my_corner = 0
        #my_corner = robot.zone #set corner in robot set-up
        #robot_mode = robot.mode # returns DEV or COMP (no parentheses)

        #&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
        # *********************** Relocated R.wait_start() ****************************
        self.time_elapsed = time.time() - self.time_started_robot    # Time from initial switch on for initialisatiom 
        print ("Initialised in ",round(self.time_elapsed,0)," seconds")
        self.time_started_game = time.time()   #  Get time when START switch pressed
        print("Game timer started")
        print("")
        print("")
        self.startup_jingle()
        self.robot.wait_start()


