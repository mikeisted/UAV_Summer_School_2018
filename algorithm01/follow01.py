"""
Copyright Aisymmetrix Ltd

This software is licensed to be used and adapted solely for the purposes of the
UWE Bristol Summer School July, 2018.

No warranty is made or liability accepted for use of the software.

"""

# import the necessary packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from detectline1 import detectline1
import argparse
import imutils
from imutils.video import VideoStream
import cv2
import sys
from pid_controller.pid import PID


# framecount is incremented with each image captured and is used to periodically record data to the log file.
global framecount
framecount = 0

# Set the desired flight height above ground in metres.  Negative is upwards!
# SO FOR REAL FLIGHT THIS MUST BE A NEGATIVE NUMBER BETWEEN -1.0 AND -2.0.
flightHeight = -5.0  # Set the desired flight height above ground in metres (suggest -5.0 for the simulator)

#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE TRACKING  ---------------------
def tracking (vstate, vehicle, vs, flightHeight, connection_string):

    #In tracking state the vehicle processes images and maintains all data ready to fly autonomously.
    #However, it does not actually send velocity vectors unless GUIDED flight mode is selected.

    ########################### CONFIGURABLE BITS FOR FLIGHT #######################################

    # Initialise velocities
    # vMax should be between 0.5 and 2.0.
    vMax = 0.75 # The forward velocity (due to pitch) in m/s
   
    # Initialise P constant for yaw control
    # Yaw controller P constant.  Reduce this for more gentle response, increase for more aggressive.
    yawP = 0.2 # This is the proportion of the yaw error we attempt to change each time around.

    # These can be set to False to increase the frame rate
    recordLog = True

    ########################## END OF CONFIGURABLE BITS ##################################

    # framecount is incremented with each image captured and is used to periodically record data.
    global framecount
    
    # Display in the terminal console the state we are now in (i.e. tracking)
    print vstate

    # Initialise PID constants for yaw control
    yawPID = PID(p=yawP, i=0.003, d=0.03)
    
    # Altitude P constant
    altP = 0.2
    
    # Initialise PID constants for altitude control
    altPID = PID(p=altP, i=0.004, d=0.02)
    
    # Safety check on maximum velocity
    if not connection_string:
        if (vMax < 0.0) or (vMax > 2.0):
            print('Maximum velocity vMax out of range - setting to 0.75 m/s')
            vMax = 0.75

    # The maximum turning angle when the line is at the edge of the image (degrees)
    # This value corresponds to the field of view from the centre to the edge of the image.
    # It should not be changed!
    yawMax = 30.0

    # Initialise velocity vectors
    # Python does not need variables to be initialised, but (IMHO) it is good practice do do so.
    # Also, the first use of a variable sets it's type - so vital to state it's a float (decimal),
    # for example, in these cases.
    
    vx = 0.0
    vy = 0.0 # Sideways velocity (due to roll) in m/s
    vz = 0.0 # Vertical velocity is controlled for us to keep at requested height.
    yaw = 0.0

    # Open the file to record log data.
    f = open("locationdata.txt","a")

    # Now start the loop in which we adjust the height, then
    # turn and move towards the detected point on the line.

    target = None # Initialise the tuple (it's just an array) returned from video stream

    while vstate == "tracking":
        framecount = framecount + 1

        # First let's make sure the vehicle is at the right height.
        # Get height above ground using rangefinder.  This is reported in metres beween 0.2 and 7m.
        # It is made negative because in we are working in NED space, where negative means up.

        if connection_string:
            height = vehicle.location.global_relative_frame.alt  * -1 # This line for SITL
        else:
            height = vehicle.rangefinder.distance * -1.0 # Or this for real flight.
        
        # Check actual height and make adjustments as necessary.      
        # Calculate required z velocity to achieved requested height
        #Remember +z is DOWN!
        # First calculate the error, then get the correction using a PID controller.  
        zError = flightHeight - height
        vz = altPID(zError)

        # This grabs a frame from the video stream and returns position of line (left:-1 / right:+1)
        target = detectline1(vs,vehicle)
        xPos = target[0] # The value between -1 and 1
        lineFound = target[1] # This is set to True if the line was found (otherwise False, of course)

        # Now we work out which way to turn to follow the point on the line.
        # The position of the line is held in xPos, from -1 extreme left to +1 extreme right.
        # First check that we did find the line last time we got an image (lineFound  is True).
        
        if lineFound == True:

            # So the line was found.  Now set the yaw value as a fraction of the maximum allowed.
            # Rememember xPos is a signal from -1 (leftmost) to +1 (rightmost) of the image.
            yawError = xPos * yawMax
            yaw = yawPID(yawError)

            # So only actually send control commands if the vehicle is still in guided mode.
            if vehicle.mode.name == "GUIDED":

                # The yaw is is currently a number between about -30 to 30 degrees.
                # The controller needs a positive number only and to know which direction.
                if yaw < 0.0: # So need to turn left - anticlockwise
                    condition_yaw(abs(yaw),-1) # anticlockwise
                else:
                    condition_yaw(abs(yaw),1) # clockwise

                # Now send the commands tell the vehicle to keep moving forward and adjust for height.
                vy = 0.0
                vx = vMax
                send_ned_velocity(vx, vy, vz)

                # Log data and save image every nth frame
                n = 10
                if framecount%n == 0:
                    if recordLog == True:
                        # Append location data to logfile
                        f.write('\n' + time.strftime("%H_%M_%S_") + ' yawError, '+str(yawError)+', Yaw, '+str(yaw)+', Height, ' + str(height)+', vz, '+str(vz))
          
        else:
            # We didn't find the line and are therefore lost!
            # Set the vehicle state flag to "lost" to find the line again.
            # This stops the drone in one place and causes it to rotate until it finds the line.
            vstate = "lost"
        
    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST  ---------------------
def lost (vstate,vehicle, vs, flightHeight, connection_string):
       
    print vstate

    # The vehicle processes images and returns to tracking state if any lock is found.
    # Meanwhile it rotates clockwise and maintains height.

    target = None # Initialise tuple returned from video stream

    # Altitude P constant
    altlP = 0.2
    
    # Initialise PID constants for altitude control
    altlPID = PID(p=altlP, i=0.004, d=0.02)

    while vstate == "lost":

        # grab the frame from the threaded video stream and return position of line (left/right)
        # The last argument (height) is a dummy value.  We just want to know if the line was found.
        target = detectline1(vs, vehicle)
        lineFound = target[1]

        if lineFound == True:
            vstate = "tracking"
            break

        else:        

            # Check that operator has transferred to autopilot using TX switch.
            if vehicle.mode.name == "GUIDED":

                if connection_string:
                    height = vehicle.location.global_relative_frame.alt  * -1 # This used for SITL
                else:
                    height = vehicle.rangefinder.distance * -1.0 # This used for real flight.

                zError = flightHeight - height
                vz = altlPID(zError)

                send_ned_velocity (0,0,vz) # Stay stationary, but adjust altitude 
                condition_yaw(6, 1, True) # Rotate clockwise
                time.sleep(0.05)      

    return vstate


#-------------------------- FUNCTION SEND VELOCITY VECTORS IN LOCAL BODY FRAME--------------------
def send_ned_velocity(x,y,z):
    """
    Move vehicle in direction based on specified velocity vectors.
    Do not edit this.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,0,0,mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,0b0000111111000111,
        0, 0, 0,x,y,z,0,0,0,0,0)
    vehicle.send_mavlink(msg)

#--------------------------FUNCTION SEND YAW IN LOCAL NED FRAME--------------------
# Heading in range 0-360 degrees.  No negs!
def condition_yaw(heading, clockwise, relative=1): # In degrees
    """
    Rotate (yaw) vehicle.
    Do not edit this.
    """
    msg = vehicle.message_factory.command_long_encode(
        0,0,mavutil.mavlink.MAV_CMD_CONDITION_YAW,0,heading,20,clockwise,relative,0, 0, 0)
    vehicle.send_mavlink(msg)

#-------------- FUNCTION DEFINITION TO ARM AND TAKE OFF TO GIVEN ALTITUDE ---------------
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    print ('Taking off!')
    aTargetAltitude = -1.0 * aTargetAltitude
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    while True:
        # print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            break
        time.sleep(1)




###################################### MAIN PROGRAM ##############################################

#--------------------------SET UP CONNECTION TO VEHICLE----------------------------------

# Parse the arguments  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the physical UAV or to the simulator on the network
if not connection_string:
    print ('Connecting to pixhawk.')
    vehicle = connect('/dev/serial0', baud=57600, rate=20, wait_ready= True)
else:
    print ('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)

#--------------------------SET UP VIDEO THREAD ----------------------------------

# Create a *threaded *video stream, allow the camera sensor to warmup
print('[INFO] sampling THREADED frames from camera module...')
vs = VideoStream(src=0).start()
time.sleep(1.0)

#--------------------------SAFETY CHECK ON FLIGHT HEIGHT ----------------------------------
if not connection_string:
    if (flightHeight < -2.0) or (flightHeight > -1.0):
        print('Flight height (altitude) out of range - setting to -1.0 (i.e. 1m above ground)')
        flightHeight = -1.0

#---------------- If connected to the simulator, arm and take off ------------------

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.

# If on simulator, arm and take off.
if connection_string:

    # Get airborne and hover
    arm_and_takeoff(-5.0)
    print "Reached target altitude - currently in Guided mode on altitude hold"
    
    # Set the flight mode to guided so that we can control the vehicle motion.
    vehicle.mode = VehicleMode("GUIDED")

while True :
    # So we start an endless loop in which we follow the line in tracking state
    # or try and find it in lost state.
    # Typically this only ends when the user takes back control manually (by setting
    # flight mode out of GUIDED) and powers the Pi off.  So could be alot neater....

    if vstate == "tracking":
        # Enter tracking state
        vstate = tracking(vstate, vehicle, vs, flightHeight, connection_string)

    else:
        # Enter lost state
        vstate = lost(vstate, vehicle, vs, flightHeight,connection_string)
    


"""

#---------------------------- CLEAN UP ----------------------------


#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
"""
