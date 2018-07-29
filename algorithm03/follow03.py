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
from detectline3 import detectline3
import argparse
from imutils.video import VideoStream
import imutils
import cv2
import sys
from pid_controller.pid import PID
import string

# framecount increments each time we go through the main loop having successfully found the line.
# Every 10th time around the loop, we record some data.
global framecount
framecount = 0

# Set the desired flight height above ground in metres.  Negative is upwards!
# SO THIS MUST BE A NEGATIVE NUMBER BETWEEN -1.0 AND -2.5.# Set the desired flight height above ground in metres.
flightHeight = -2.0 # Set the desired flight height above ground in metres


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE TRACKING  ---------------------
def tracking (vstate, vehicle, vs, flightHeight, connection_string):
    
    #In tracking state the vehicle processes images and maintains all data ready to fly autonomously.
    #However, it does not actually send velocity vectors unless GUIDED flight mode is selected.

    ########################### CONFIGURABLE BITS #######################################

    # Initialise velocities
    # vMax should be between 0.5 and 2.0.
    vMax = 1.0 # The forward velocity (due to pitch) in m/s
    
    # Initialise P constant for yaw control
    # Yaw controller P constant.  Reduce this for more gentle response, increase for more aggressive.
    yawP = 0.25 # This is the proportion of the yaw error we attempt to change each time around.
    
    # We have some idea of how much to one side of the line we are.
    # This can be corrected by 'sideslipping' the vehicle, so introducing a sideways velocity.
    # This is the largest value permitted, in m/s.  The  bigger this is, the more the vehicle
    # sideslip to try to get back over the line. Maximum suggested is 1 m/s.
    vyMax = 0.3
    vyP = 0.4        # Sideslip (vy) controller P constant

    # These can be set to False to increase the frame rate
    recordLog = False
    saveImages = False
    annotateFrame = True
    

    ########################## END OF CONFIGURABLE BITS ##################################

    # framecount is incremented with each image captured and is used to periodically record data.
    global framecount

    # Display in the terminal console the state we are now in (i.e. tracking)
    print vstate
    
    # Initialise PID constants for yaw control
    yawPID = PID(p=yawP, i=0.003, d=0.03)
    
    # Altitude P constant
    altP = 0.3
    
    # Initialise PID constants for altitude control
    altPID = PID(p=altP, i=0.004, d=0.02)

    # Initialise PID constants for sideslip velocity (vy) control
    vyPID = PID(p=vyP, i=0.004, d=0.02)

    # Safety check on maximum velocity
    if not connection_string:
        if (vMax < 0.0) or (vMax > 2.0):
            print('Maximum velocity vMax out of range - setting to 0.75 m/s')
            vMax = 0.75

    # Initialise velocity vectors
    # Python does not need variables to be initialised, but (IMHO) it is good practice do do so.
    # Also, the first use of a variable sets it's type - so vital to state it's a float (decimal),
    # for example, in these cases.
    
    vy = 0.0 # Sideways velocity (due to roll) in m/s
    vz = 0.0 # Vertical velocity is controlled for us to keep at requested height.
    yaw = 0.0

    # Open the file to record key data.
    f = open("locationdata.txt","a")

    # The vehicle process images and maintains all data ready to fly autonomously.
    # However, it does not actually send velocity vector commands unless GUIDED
    # flight mode is selected.

    target = None # Initialise tuple returned from video stream
    frame = None

    while vstate == "tracking":
        framecount = framecount + 1

        # First let's make sure the vehicle is at the right height.
        # Get height above ground using rangefinder.  This is reported in metres beween 0.2 and 7m.

        if connection_string:
            height = vehicle.location.global_relative_frame.alt  * -1 # This used for SITL
        else:
            height = vehicle.rangefinder.distance * -1.0 # This used for real flight.
            
        #print height    

        # Check actual height and make adjustments as necessary.      
        # Calculate required z velocity to achieved requested height
        #Remember +z is DOWN!
        # First calculate the error, then get the correction using a simple P controller and deadband,
        # or a full PID controller.
        zError = flightHeight - height
        vz = altPID(zError)

        # grab the frame from the threaded video stream and return position of line (left/right)
        frame, target = detectline3(vs, vehicle, height)
        bearing = target[0]      # In degrees, left is -ve, right +ve
        offset = target[1]     # A value from -1 to 1, indicating how far to the side the line is.
        lineFound = target[2]  # True if the line was found ok.  False if the line was not found.
	#print bearing, offset

        # Now we work out which way to turn to follow the line.
        # First check that we did find the line last time we got an image (lineFound is True).
        if lineFound == True:

            # Here we set the yaw to match the bearing of the line - we are trying to fly along it!
            # Now set the yaw value as a fraction of the maximum allowed.
            # First calculate the error, then get the correction using a PID controller.
            yawError = bearing
            yaw = yawPID(yawError)

            # The position of the line is held in 'offset', from -1 extreme left to +1 extreme right.
            # Set sideslip to try to get above the line.
            # First calculate the error, then get the correction using a simple P controller and deadband,
            # or a full PID controller.
            vyError = offset * vyMax
            vy = vyPID(vyError)

            # So only actually send control commands if the vehicle is still in guided mode.
            if vehicle.mode.name == "GUIDED":
                
                if bearing < 0.0: # So need to turn left - anticlockwise
                    condition_yaw(abs(yaw), -1) # anticlockwise
                else:
                    condition_yaw(abs(yaw), 1) # clockwise
                    
                # Now send the velocities to the vehicle.
                vx = vMax
                #send_ned_velocity(0.0, 0.0, vz)
                send_ned_velocity(vx, vy, vz)

                if annotateFrame == True:
                    annotate_frame(frame,vstate,height,bearing,offset,vx,vy,vz)
                
                # Log data and save image every nth frame
                n = 10
                
                if framecount%n == 0:
                    if recordLog == True:
                        # Append location data to logfile
                        f.write('\n' + time.strftime("%H_%M_%S_") + ' yawError, '+str(yawError)+', Yaw, '+str(yaw)+', Height, ' + str(height)+', vz, '+str(vz))
                    if saveImages == True:
                        cv2.imwrite('images/'+ (time.strftime("%H_%M_%S_"))+str(framecount)+'.jpg', frame)
                
        else:
            # We didn't find the line and are therefore lost!
            # Set the vehicle state flag to "lost" to find the line again.
            # This stops the vehicle in one place and causes it to rotate until it finds the line.
            vstate = "lost"
            
        # Show the annotated video frame
        show_frame(frame)
    
    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST  ---------------------
def lost (vstate,vehicle, vs, flightHeight, connection_string):
       
    print vstate

    # The vehicle processes images and returns to tracking state if any lock is found.
    # Meanwhile it rotates clockwise and maintains height.

    target = None # Initialise tuple returned from video stream
    frame = None

    # Altitude P constant
    altlP = 0.3
    
    # Initialise PID constants for altitude control
    altlPID = PID(p=altlP, i=0.004, d=0.02)

    while vstate == "lost":

        # grab the frame from the threaded video stream and return position of line (left/right)
        # The last argument (height) is a dummy value.  We just want to know if the line was found.
        frame, target = detectline3(vs, vehicle, 1.0)
        angle = target[0]
        offset = target[1]
        lineFound = target[2]

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


                send_ned_velocity (0.0, 0.0, vz) # Stay stationary, but adjust altitude 
                condition_yaw(6, 1, True) # Rotate clockwise
                time.sleep(0.05)
                
        # Show the annotated video frame
        show_frame(frame)

    return vstate

#----------------------------------- ANNOTATE VIDEO FRAME ----------------------------------------
# Annotates the video frame with flight data
def annotate_frame(frame,vstate,height,bearing,offset,vx,vy,vz):

    # Convert and format all variables to strings to display
    height_d = str (round (height, 1))
    bearing_d = str (round (bearing, 1))
    offset_d = str (round (offset, 2))
    vx_d = str (round (vx, 2))
    vy_d = str (round (vy, 2))
    vz_d = str (round (vz, 2))

    # Display data on frame
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(frame,"State",(10,20), font, 0.4,(255,255,255),2)
    cv2.putText(frame,vstate,(60,20), font, 0.4,(255,255,255),2)
    cv2.putText(frame,"Height",(10,40), font, 0.4,(255,255,255),2)
    cv2.putText(frame,height_d,(60,40), font, 0.4,(255,255,255),2)
    cv2.putText(frame,"Bearing",(10,60), font, 0.4,(255,255,255),2)
    cv2.putText(frame,bearing_d,(60,60), font, 0.5,(255,255,255),2)    
    cv2.putText(frame,"Offset",(10,80), font, 0.4,(255,255,255),2)
    cv2.putText(frame,offset_d,(60,80), font, 0.5,(255,255,255),2)
    cv2.putText(frame,"vx",(10,100), font, 0.4,(255,255,255),2)
    cv2.putText(frame,vx_d,(60,100), font, 0.5,(255,255,255),2)
    cv2.putText(frame,"vy",(10,120), font, 0.4,(255,255,255),2)
    cv2.putText(frame,vy_d,(60,120), font, 0.5,(255,255,255),2)
    cv2.putText(frame,"vz",(10,140), font, 0.4,(255,255,255),2)
    cv2.putText(frame,vz_d,(60,140), font, 0.5,(255,255,255),2)
    

    return (frame)


#------------------------------- FUNCTION SHOW VIDEO FRAME ---------------------------------------
# Displays the annotated video frame
def show_frame(frame):
    cv2.imshow('Frame',frame)
    key = cv2.waitKey(1) & 0xFF
    
    return


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


#---------------- If connected to the simulator, arm and take off ------------------

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.

# If on simulator, arm and take off.
if connection_string:

    # Get airborne and hover to 5m altitude
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
        vstate = tracking(vstate, vehicle, vs, flightHeight,connection_string)

    else:
        # Enter lost state
        vstate = lost(vstate, vehicle, vs, flightHeight,connection_string)


"""

#---------------------------- RETURN TO HOME AND CLEAN UP ----------------------------


# Initiate return to home
print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")
print "Pause for 10s before closing vehicle"
time.sleep(10)

"""

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
