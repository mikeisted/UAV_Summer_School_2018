"""
Copyright Aisymmetrix Ltd

This software is licensed to be used and adapted solely for the purposes of the
UWE Bristol Summer School July, 2018.

No warranty is made or liability accepted for use of the software.

"""

# import the necessary packages

from threading import Thread
import numpy as np
import imutils
import time
import cv2
import string


global imagecount
imagecount = 0

#--------------------------VERTICAL ANGLE FROM VERTICAL IMAGE OFFSET--------------------
# Returns the angle (radians) in the vertical plane between a target and the centreline in the
# camera image. 

def vert_image_angle (y):
    #print y
    vertical_angle_of_view = 48.8 # degrees
    vertical_resolution = 240 # pixels
    pix_per_degree = vertical_resolution / vertical_angle_of_view

    vertical_angle = (y / pix_per_degree)
    # print vertical_angle
    vertical_angle = np.radians(vertical_angle)
    

    return (vertical_angle)

#--------------------------HORIZONTAL ANGLE FROM HORIZONTAL IMAGE OFFSET----------------
# Returns the angle (radians) in the horizontal plane between a target and the centreline in the
# camera image. x is the x coordinate of the target, origin top left of frame.

def horiz_image_angle (x):
    horiz_angle_of_view = 62.2 # degrees
    horiz_resolution = 320 # pixels
    pix_per_degree = horiz_resolution / horiz_angle_of_view
    
    horiz_angle = np.radians(x / pix_per_degree)

    return (horiz_angle)

#--------------------------GET BEARING A to B-------------------------------------
# Returns the yaw (radians) as measured from the centreline.  Positive is clockwise.

def new_bearing(locA, locB):

    na = locA[0]
    ea = locA[1]
    nb = locB[0]
    eb = locB[1]

    #print na, nb, ea, eb
    deltan = nb - na
    deltae = eb - ea

    alphaAB = np.arctan2(deltae,deltan)
    #print 'AlphaAB', alphaAB

    return (alphaAB)

def preparemask (hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper);
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask;

def meldmask (mask_0, mask_1):
    mask = cv2.bitwise_or(mask_0, mask_1)
    return mask;


def detectline3(vs, vehicle, height):
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
    xColour1 = xRed = 0.0

    # Initialise confidence to indicate the line has not been located with the current frame
    coordA_Good = False
    red1Good = False
    coordB_Good = False
    red2Good = False

    # Initialise variables for line calculations
    coordA = [0,0]
    coordB = [0,0]
    xRed1 = xRed2 = 0
    yRed1 = yRed2 = 0

    #bearing = offset = 0
                

    # return the frame most recently read
    frame = vs.read()
    frame = imutils.resize(frame, width=320)
  
    # Set y coords of regions of interest.
    # The upper and lower bounds
    roidepth = 20    # vertical depth of regions of interest
    roiymin = 40    # minumum ranging y value for roi origin
    roiymintop = roiymin - roidepth
    roiymax = yres - roidepth -1   # maximum ranging y value for bottom roi origin
    
    # Convert to hsv and define region of interest before further processing.
    #fullhsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # green = 60
    # blue = 120;
    # yellow = 30;
    #Colour1 = 60

    # Set the sensitivity of the hue
    sensitivity = 20

    # Red is a special case as it sits either side of 0 on the HSV spectrum
    # So we create two masks, one greater than zero and one less than zero
    # Then combine the two.
    lower_red_0 = np.array([0, 100, 100]) 
    upper_red_0 = np.array([sensitivity, 255, 255])
    
    lower_red_1 = np.array([180 - sensitivity, 100, 100]) 
    upper_red_1 = np.array([180, 255, 255])


    # Initialise the bottom roi at the maximum limit
    y3 = roiymax
    y4 = y3 + roidepth

    while y3 > roiymin:

        # This defines the lower band, looking closer in
        roihsv2 = frame[y3:y4, 0:(xres-1)]
        blurred2 = cv2.GaussianBlur(roihsv2, (11, 11), 0)
        roihsv2 = cv2.cvtColor(blurred2, cv2.COLOR_BGR2HSV)
    

        # Prepare the masks for the lower roi 
        maskr_2 = preparemask (roihsv2, lower_red_0 , upper_red_0)
        maskr_3 = preparemask (roihsv2, lower_red_1 , upper_red_1 )
        maskr2 = meldmask ( maskr_2, maskr_3)

        # find contours in the lower roi and initialize the center
        cnts_red2 = cv2.findContours(maskr2.copy(), cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE)[-2]
        center2 = None

        # Now to find the tracking line in the lower roi
        # only proceed if at least one contour was found
        if len(cnts_red2) > 0:
            
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c_red2 = max(cnts_red2, key=cv2.contourArea)
            ((x_red2, y_red2), radius_red2) = cv2.minEnclosingCircle(c_red2)
            M_red2 = cv2.moments(c_red2)

            # compute the center of the contour
            cx_red2 = int(M_red2["m10"] / M_red2["m00"])
            cy_red2 = int(M_red2["m01"] / M_red2["m00"])
            

            # cy_red is set in the region of interest, so need to adjust for origin in frame
            cy_red2 = cy_red2 + y3
            # center = ( cx_red, cy_red )

            # only proceed if the radius meets a minimum size
            if radius_red2 > 5:
                coordA_Good = True
                # draw the circle and centroid on the frame
                cv2.circle(frame, (cx_red2, cy_red2), int(radius_red2),
                (0, 0, 255), 2)                            

                # calculate offset from centreline
                xRed2 = cx_red2 - (xres/2) # Contrived so pstve to right of centreline
                yRed2 = (yres/2) - cy_red2  # Negative values below centreline
                coordA = [xRed2, yRed2]

                # The target has been found, so we can break out of the loop here
                break

        # But here the target has not been found, we need to move the ROI up
        y3 = y3 - roidepth
        y4 = y3 + roidepth

    # And here we have either hit the buffers or found the target.

    
    # So now try for the top roi, working down.                      
    # Initialise the top roi at the very top
    y1 = 0
    y2 = y1 + roidepth

    while y2 < y3: # Go as far as the lower roi but no more.

        # This defines the upper roi, looking further away
        roihsv1 = frame[y1:y2, 0:(xres-1)]
        blurred1 = cv2.GaussianBlur(roihsv1, (11, 11), 0)
        roihsv1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

        # Prepare the masks for the top roi 
        maskr_0 = preparemask (roihsv1, lower_red_0 , upper_red_0)
        maskr_1 = preparemask (roihsv1, lower_red_1 , upper_red_1 )
        maskr1 = meldmask ( maskr_0, maskr_1)

        # find contours in the upper roi and initialize the center
        cnts_red1 = cv2.findContours(maskr1.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]
        center1 = None

        # Now to find the tracking line in the upper roi
        # only proceed if at least one contour was found
        if len(cnts_red1) > 0:
            
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c_red1 = max(cnts_red1, key=cv2.contourArea)
            ((x_red1, y_red1), radius_red1) = cv2.minEnclosingCircle(c_red1)
            M_red1 = cv2.moments(c_red1)

            # compute the center of the contour
            cx_red1 = int(M_red1["m10"] / M_red1["m00"])
            cy_red1 = int(M_red1["m01"] / M_red1["m00"])
            

            # cy_red is set in the region of interest, so need to adjust for origin in frame
            cy_red1 = cy_red1 + y1
            # center = ( cx_red, cy_red )

            # only proceed if the radius meets a minimum size
            if radius_red1 > 5:
                coordB_Good = True
                # draw the circle and centroid on the frame
                cv2.circle(frame, (cx_red1, cy_red1), int(radius_red1),
                (0, 0, 255), 2)

                # calculate offset from centreline
                xRed1 = cx_red1 - (xres/2) # Contrived so pstve to right of centreline
                yRed1 = (yres/2) - cy_red1  # Negative values below centreline
                coordB = [xRed1, yRed1]

                # The target has been found, so we can break out of the loop here
                break

        # But here the target has not been found, we need to move the ROI down
        y1 = y1 + roidepth
        y2 = y1 + roidepth

    # And here we have either hit the buffers or found the target.

    

    # Draw Region of interest
    cv2.line(frame, (0, y1), (xres, y1), (255,0,0))
    cv2.line(frame, (0, y2), (xres, y2), (255,0,0))
    cv2.line(frame, (0, y3), (xres, y3), (255,0,0))
    cv2.line(frame, (0, y4), (xres, y4), (255,0,0))

    
    if (coordA_Good == True) and (coordB_Good == True) :
        # Draw line to show bearing
        cv2.line(frame,(cx_red1,cy_red1),(cx_red2,cy_red2),(0,0,255),5)

    # Save every nth image to the 'image' file.
    #if vehicle.mode.name == "GUIDED":
    #   if imagecount%10 == 0:
    #       cv2.imwrite('/home/pi/images/image'+ (time.strftime("%H_%M_%S_"))+str(imagecount)+'.jpg', frame)
    
    # We now have the coordinates of two points on the line on the image from the camera.
    # But the angle measured on the camera will not be the same as that on the ground.
    # This is because the camera is at an angle to the ground.
    # We need to convert the angle seen by the camera to the actual angle on the ground.
    # ...using trigonometry!

    # coordA is closer in (at the bottom of the image)
    # coordB is farther away (top of the image)



    # Calculate overall pitch and pan (yaw) as inputs to calculation of locations.        
    camera_pitch = np.radians(-45) # The camera pitch is fixed at 45 degrees.        

    if coordA_Good == True:

        pitch_from_image = vert_image_angle(coordA[1])
        total_pitch = (camera_pitch + pitch_from_image) # Down is negative, in radians
        #print total_pitch
        #total_pitch = np.radians(-55)

        total_pan = horiz_image_angle(coordA[0])
        #print 'A total pan', total_pan
        #total_pan = np.radians(0)
        
        mult = height / np.tan(total_pitch)
        coordA_north = mult * np.cos(total_pan)
        coordA_east = mult * np.sin(total_pan)
        locA = (coordA_north, coordA_east) # Relative to current frame
        #print locA

    else:
        # So we could not get a lock on the line close in.
        locA = (0,0)
        

    if coordB_Good == True:

        pitch_from_image = vert_image_angle(coordB[1])
        total_pitch = (camera_pitch + pitch_from_image) # Down is negative, in radians
        #print 'total pitch', total_pitch
        #total_pitch = np.radians(-45)

        total_pan = horiz_image_angle(coordB[0])
        #print 'total pan', total_pan
        #total_pan = np.radians(-0.1)
        
        mult = height / np.tan(total_pitch)
        coordB_north = mult * np.cos(total_pan)
        coordB_east = mult * np.sin(total_pan)
        locB = (coordB_north, coordB_east) # Relative to current frame
        
    else:
        # So we could not get a lock on the line farther away.
        locB = (0,0)

    # Set coordinates to follow depending on whether we have a lock on one point or two.
    # If we only have one lock, follow that.
    if coordA_Good == True and coordB_Good == False:
        locB = locA
        locA = (0,0)
        coordB_Good == True

    # If B is good and A is bad, locA and locB will already be set up correctly.         
    
    if coordA_Good == True and coordB_Good == True:     

        # Calculate the bearing of the line.  Returned as a +ve in radians.
        bearingAB = (new_bearing(locA,locB))
        #print locA, locB, bearingAB
        angle = np.degrees(bearingAB)
        lineFound = True

    else:
        angle = 0.0
        lineFound = False


    # And finally we want to know the 'offset' of the line horizonatally,
    # so that we can adjust the sideways velocity to fly over the line.
    # This is related to the 'c' in y = mx + c.
    # Here we will just note how much to the side the lower (closer) point on the line is,
    # returning a value from -1 (extreme left) to +1 (extreme right).

    # The lower point was coordA.  We are just interested in the x value (coordA[0]).
    
    # xred is currently in pixels (-ve to left of centre, +ve to right)
    # We need to convert to range -1 to +1.
    # We also need to convert from integer values to floating point.
    offset = (2.0 * coordA[0]) / xres
    #offset = 0.0
    #angle = -5.0
    #print ('Angle: ',angle,'  Offset: ', offset)

    # We can now return the key values:
    # angle - the bearing in degrees of the line compared to the vehicle
    # offset - a signal from -1 to 1 to indicate if the line is to the left or right
    # lineFound - a boolean indicating if we found the line at all.
    return frame, (angle, offset, lineFound)


def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

