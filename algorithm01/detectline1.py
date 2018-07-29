"""
Copyright Aisymmetrix Ltd

This software is licensed to be used and adapted solely for the purposes of the
UWE Bristol Summer School July, 2018.

No warranty is made or liability accepted for use of the software.

"""

# import the necessary packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
from threading import Thread
import numpy as np
import imutils
import time
import cv2
import string

# imagecount is incremented with each image captured and is used to optionally save the annotated image to file.
global imagecount
imagecount = 0


def preparemask (hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper);
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask;

def meldmask (mask_0, mask_1):
    mask = cv2.bitwise_or(mask_0, mask_1)
    return mask;


def detectline1(vs,vehicle):
    # This is passed an instance of the videostream.
    # It gets the next video frame and finds the line (or not!).
    # It returns the position of the line in the frame from -1 to 1,
    # measured along a horizontal band in the middle of the image.
    # It also returns a boolean value to indicate if it was successful.
    # Every nth frame is saved for later inspection.

    global imagecount

    imagecount = imagecount + 1
    if imagecount > 99:
        imagecount  = 0

    # Set the image resolution.
    xres = 320
    yres = 240

    # Set the sensitivity of the hue
    # Make this bigger to capture a broader range of colour hues.
    sensitivity = 20
    
    # Initialise confidence to indicate the line has not been located with the current frame
    lineFound = False

    # Initialise variables for line calculations
    xRed = 0
    yRed = 0
    xPos = 0.0
         

    # Get the next video frame
    frame = vs.read()
    frame = imutils.resize(frame, width=xres)
    # Get roll of vehicle.  We will rotate the processed image to account for this.
    # roll = np.degrees(vehicle.attitude.roll)
    # Rotate the frame to account for the roll of the vehicle.
    # frame = imutils.rotate(frame, -roll)

    # Set y coords of regions of interest.
    # The upper and lower bounds
    roidepth = 20    # vertical depth of horizontal band (region of interest).
    roiymin = (yres/2) - (roidepth/2)  # lower y value for roi
    # roiymintop = roiymin - roidepth
    roiymax = roiymin + roidepth   # lower y value for roi
    
    # Convert to hsv and define region of interest before further processing.
    #fullhsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # green = 60
    # blue = 120;
    # yellow = 30;

    # Red is a special case as it sits either side of 0 on the HSV spectrum
    # So we create two masks, one greater than zero and one less than zero
    # Then combine the two.
    lower_red_0 = np.array([0, 100, 100]) 
    upper_red_0 = np.array([sensitivity, 255, 255])
    
    lower_red_1 = np.array([180 - sensitivity, 100, 100]) 
    upper_red_1 = np.array([180, 255, 255])


    # Process the horizontal band of the frame ready to find the line.
    roihsv = frame[roiymin:roiymax, 0:(xres-1)]
    blurred = cv2.GaussianBlur(roihsv, (11, 11), 0)
    roihsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)


    # Apply the masks to highlight just the red
    maskr_2 = preparemask (roihsv, lower_red_0 , upper_red_0)
    maskr_3 = preparemask (roihsv, lower_red_1 , upper_red_1 )
    maskr2 = meldmask ( maskr_2, maskr_3)


    # find contours in the lower roi and initialize the center
    cnts_red = cv2.findContours(maskr2.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None

    # Now to find the tracking line in the region of interest
    # only proceed if at least one contour was found
    # We are only interested in the horizontal position of the line (x).
    if len(cnts_red) > 0:
        
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c_red = max(cnts_red, key=cv2.contourArea)
        ((x_red, y_red), radius_red) = cv2.minEnclosingCircle(c_red)
        M_red = cv2.moments(c_red)

        # compute the center of the contour
        cx_red = int(M_red["m10"] / M_red["m00"])
        cy_red = int(M_red["m01"] / M_red["m00"])
        

        # cy_red is set in the region of interest, so need to adjust for origin in frame
        cy_red = cy_red + roiymin

        # only proceed if the radius meets a minimum size
        if radius_red > 5:
            lineFound = True
            # draw the circle and centroid on the original frame
            cv2.circle(frame, (cx_red, cy_red), int(radius_red),
            (0, 0, 255), 2)                            

            # calculate offset from centreline
            xRed = cx_red - (xres/2) # Contrived so pstve to right of centreline
            # coordA = [xRed, yRed]
            
            # xred is currently in pixels (-nve to left of centre, +ve to right)
            # We need to convert to range -1 to +1.
            # We also need to convert from integer values to floating point.
            xPos = (2.0 * xRed) / xres


    # And here we have either hit the buffers or found the target.
    # If we have not found the line, then lineFound will be False
    

    # Draw Region of interest
    cv2.line(frame, (0, roiymin), (xres, roiymin), (255,0,0))
    cv2.line(frame, (0, roiymax), (xres, roiymax), (255,0,0))

    cv2.imshow('Frame',frame)
    #cv2.imshow('Mask',maskr2) # un-comment this to see the black and white maskof the image
    key = cv2.waitKey(1) & 0xFF
    
    # Un-comment below to save every 10th image to the 'image' file.
    #if vehicle.mode.name == "GUIDED":
    #    if imagecount%10 == 0:
    #        cv2.imwrite('images/image'+ (time.strftime("%H_%M_%S_"))+str(imagecount)+'.jpg', frame)

    # So we return xPos, the signal from -1 to +1 (left to right) and lineFound,
    # a boolean to indicate if the line was found.
    return (xPos, lineFound)


def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

