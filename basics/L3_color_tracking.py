# L3_color_tracking.py
# This program was designed to have SCUTTLE following a target using a USB camera input

import cv2              # For image capture and processing
import numpy as np      
import L2_speed_control as sc
import L2_inverse_kinematics as ik
import L2_kinematics as kin
import netifaces as ni
import L2_vector
from time import sleep
from math import radians, pi
import L1_lidar as lidar
import L1_motor as motor
np.set_printoptions(precision=3)                    # after math operations, don't print long values

def getValid(scan):                                 # remove the rows which have invalid distances
    dist = scan[:, 0]                               # store just first column
    angles = scan[:, 1]                             # store just 2nd column
    valid = np.where(dist > 0.08)                  # find values 16mm
    myNums = dist[valid]                            # get valid distances
    myAng = angles[valid]                           # get corresponding valid angles
    output = np.vstack((myNums, myAng))             # recombine columns
    n = output.T                                    # transpose the matrix
    return n

def nearest(scan):                                  # find the nearest point in the scan
    dist = scan[:, 0]                               # store just first column
    column_mins = np.argmin(dist, axis=0)           # get index of min values along 0th axis (columns)
    row_index = column_mins                         # index of the smallest distance
    vec = scan[row_index, :]                        # return the distance and angle of the nearest object in scan
    return vec                                      # contains [r, alpha]

def getNearest():                                   # combine multiple functions into one.  Call to get nearest obstacle.
    scan = lidar.polarScan()                        # get a reading in meters and degrees
    valids = getValid(scan)                         # remove the bad readings
    vec = nearest(valids)                           # find the nearest
    return vec                                      # pass the closest valid vector [m, deg]
# Gets IP to grab MJPG stream
def getIp():
    for interface in ni.interfaces()[1:]:   #For interfaces eth0 and wlan0
    
        try:
            ip = ni.ifaddresses(interface)[ni.AF_INET][0]['addr']
            return ip
            
        except KeyError:                    #We get a KeyError if the interface does not have the info
            continue                        #Try the next interface since this one has no IPv4
        
    return 0
    
#    Camera
stream_ip = getIp()
if not stream_ip: 
    print("Failed to get IP for camera stream")
    exit()

camera_input = 'http://' + stream_ip + ':8090/?action=stream'   # Address for stream

size_w  = 240   # Resized image width. This is the image width in pixels.
size_h = 160	# Resized image height. This is the image height in pixels.

fov = 1         # Camera field of view in rad (estimate)

#    Color Range, described in HSV
v1_min = 85      # Minimum H value
v2_min = 35     # Minimum S value
v3_min = 70      # Minimum V value

v1_max = 140     # Maximum H value
v2_max = 220    # Maximum S value
v3_max = 255    # Maximum V value

target_width = 80      # Target pixel width of tracked object
angle_margin = 0.2      # Radians object can be from image center to be considered "centered"
width_margin = 5       # Minimum width error to drive forward/back
xcount = 0
def main():
    global xcount
    global duty
    
    # Try opening camera with default method
    try:
        camera = cv2.VideoCapture(0)    
    except: pass

    # Try opening camera stream if default method failed
    if not camera.isOpened():
        camera = cv2.VideoCapture(camera_input)    

    camera.set(3, size_w)                       # Set width of images that will be retrived from camera
    camera.set(4, size_h)                       # Set height of images that will be retrived from camera

    try:
        while True:
            
            sleep(.05)                                          
            #distance = L2_vector.getNearest()  
            ret, image = camera.read()  # Get image from camera

            # Make sure image was grabbed
            if not ret:
                print("Failed to retrieve image!")
                break

            image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)              # Convert image to HSV

            height, width, channels = image.shape                       # Get shape of image

            thresh = cv2.inRange(image, (v1_min, v2_min, v3_min), (v1_max, v2_max, v3_max))   # Find all pixels in color range

            kernel = np.ones((5,5),np.uint8)                            # Set kernel size
            mask = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)     # Open morph: removes noise w/ erode followed by dilate
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)      # Close morph: fills openings w/ dilate followed by erode
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
            
            if len(cnts) and len(cnts) < 3:                             # If more than 0 and less than 3 closed shapes exist
                xcount = 0
                c = max(cnts, key=cv2.contourArea)                      # return the largest target area
                x,y,w,h = cv2.boundingRect(c)                           # Get bounding rectangle (x,y,w,h) of the largest contour
                center = (int(x+0.5*w), int(y+0.5*h))                   # defines center of rectangle around the largest target area
                angle = round(((center[0]/width)-0.5)*fov, 3)           # angle of vector towards target center from camera, where 0 deg is centered

                wheel_measured = kin.getPdCurrent()                     # Wheel speed measurements

                # If robot is facing target
                if abs(angle) < angle_margin:                                 
                    e_width = target_width - w                          # Find error in target width and measured width

                    # If error width is within acceptable margin
                    if abs(e_width) < width_margin:
                        sc.driveOpenLoop(np.array([0.,0.]))             # Stop when centered and aligned
                        print("Aligned! ",w)
                        quit()
                        continue
                        break

                    fwd_effort = e_width/target_width                   
                    distance = getNearest()                                                 # call the function which utilizes several functions in this program
                    #if distance[0] <= 0.125 and distance[1] >= 5 and distance[1] <= 90:       #conditions if object is too close to the left
                    #    print("Obstacle Left")
                    #    motor.sendLeft(-0.8)                                                #left wheel reverse
                    #    motor.sendRight(0.8)                                                #right wheel forward
                    #elif distance[0] <= 0.125 and distance[1] <= -5 and distance[1] >= -90:   #conditions if object is too close to the right
                    #    print("Obstacle Right")
                    #    motor.sendLeft(0.8)                                                 #left wheel forward
                    #    motor.sendRight(-0.8)                                               #right wheel reverse
                    if distance[0] <= 0.5  and distance[1] < 70 and distance[1] > -5:       #condition if object is too close in front left, turn 90ish degrees to the right
                        print("Obstacle Front Left")
                        motor.sendLeft(0.8)                                                #left wheel reverse
                        motor.sendRight(-0.8)                                                #right wheel forward
                        sleep(.75)                                                            #continue turn for 2 seconds
                        #motor.sendLeft(.8)
                        #motor.sendRight(1)
                        #sleep(1.5)
                        #motor.sendLeft(-.8)
                        #motor.sendRight(.8)
                        #sleep(.65)
                        #motor.sendLeft(.8)
                        #motor.sendRight(1)
                        #sleep(1)
                    elif distance[0] <= 0.5 and distance[1] > -65 and distance[1] < -6:      #condition if object is too close in front right, turn 90ish degrees to the left
                        print("Obstacle Front Right")
                        motor.sendLeft(-0.8)                                                 #left wheel forward
                        motor.sendRight(0.8)                                               #right wheel reverse
                        sleep(.75)                                                            #continue turn for 2 seconds
                        #motor.sendLeft(.8)
                        #motor.sendRight(1)
                        #sleep(1.5)                                                            #continue turn for 2 seconds
                        #motor.sendLeft(.8)
                        #motor.sendRight(-.8)
                        #sleep(.65)
                        #motor.sendLeft(.8)
                        #motor.sendRight(1)
                        #sleep(1.5)
                    else:
                        motor.sendLeft(0.8)                                                 #left wheel forward
                        motor.sendRight(.8)                                                #right wheel forward     
                    wheel_speed = ik.getPdTargets(np.array([0.8*fwd_effort, -0.5*angle]))   # Find wheel speeds for approach and heading correction
                    sc.driveClosedLoop(wheel_speed, wheel_measured, 0)  # Drive closed loop
                    print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)
                    continue

                wheel_speed = ik.getPdTargets(np.array([0, -1.1*angle]))    # Find wheel speeds for only turning

                sc.driveClosedLoop(wheel_speed, wheel_measured, 0)          # Drive robot
                print("Angle: ", angle, " | Target L/R: ", *wheel_speed, " | Measured L\R: ", *wheel_measured)

            else:
                print("No targets")
                distance = getNearest()                                                 # call the function which utilizes several functions in this program
                sc.driveOpenLoop(np.array([0.,0.]))
                #if distance[0] <= 0.125 and distance[1] >= 5 and distance[1] <= 90:       #conditions if object is too close to the left
                #    print("Obstacle Left")
                #    motor.sendLeft(-0.8)                                                #left wheel reverse
                #    motor.sendRight(0.8)                                                #right wheel forward
                #elif distance[0] <= 0.125 and distance[1] <= -5 and distance[1] >= -90:   #conditions if object is too close to the right
                #    print("Obstacle Right")
                #    motor.sendLeft(0.8)                                                 #left wheel forward
                #    motor.sendRight(-0.8)                                               #right wheel reverse
                if distance[0] <= 0.5  and distance[1] < 70 and distance[1] > -15:       #condition if object is too close in front left, turn 90ish degrees to the right
                    print("Obstacle Front Left")
                    motor.sendLeft(0.8)                                                #left wheel reverse
                    motor.sendRight(-0.8)                                                #right wheel forward
                    sleep(.75)                                                            #continue turn for 2 seconds
                    #motor.sendLeft(.8)
                    #motor.sendRight(1)
                    #sleep(1.5)
                    #motor.sendLeft(-.8)
                    #motor.sendRight(.8)
                    #sleep(.75)
                    #motor.sendLeft(.8)
                    #motor.sendRight(1)
                    #sleep(1.5)
                elif distance[0] <= 0.5 and distance[1] > -65 and distance[1] < -16:      #condition if object is too close in front right, turn 90ish degrees to the left
                    print("Obstacle Front Right")
                    motor.sendLeft(-0.8)                                                 #left wheel forward
                    motor.sendRight(0.8)                                               #right wheel reverse
                    sleep(.75)                                                            #continue turn for 2 seconds
                    ##otor.sendLeft(.8)
                    #motor.sendRight(1)
                    #sleep(1.5)                                                            #continue turn for 2 seconds
                    #motor.sendLeft(.8)
                    #motor.sendRight(-.8)
                    #sleep(.75)
                    #motor.sendLeft(.8)
                    #motor.sendRight(1)
                    #sleep(1.5)
                else:
                    motor.sendLeft(0.8)                                                 #left wheel forward
                    motor.sendRight(.8)                                                #right wheel forward                                                #right wheel forward
                #if distance[0] < .125:
                #    print("Obstacle")
                #    sc.driveOpenLoop(np.array([-5,5.75]))
                #    sleep(2.5)
                #    sc.driveOpenLoop(np.array([5,5.75]))
                #    sleep(3)
                #    sc.driveOpenLoop(np.array([5,-5.75]))
                ##    sleep(2.5)
                #    sc.driveOpenLoop(np.array([5,5.75]))
                #    sleep(3)
                #    sc.driveOpenLoop(np.array([5,5.-5.75]))
                #    sleep(2.5)
                #    sc.driveOpenLoop(np.array([5,5.75]))
                #    sleep(3)
                #    sc.driveOpenLoop(np.array([-5,5.75]))
                #    sleep(2.5)
                #else:
                #    sc.driveOpenLoop(np.array([5,5.75]))
                ##cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                #    cv2.CHAIN_APPROX_SIMPLE)[-2]                        # Find closed shapes in image
                #print("No targets")
                #sc.driveOpenLoop(np.array([0.,0.]))
                #if xcount==0:
                #    sc.driveOpenLoop(np.array([5.,5.75]))
                #    sleep(2) 
                #    xcount = xcount + 1
                #elif xcount == 1:
                #    sc.driveOpenLoop(np.array([5.,0.]))
                #    sleep(1)
                #    xcount = xcount+1   
                #elif xcount == 2:
                #    sc.driveOpenLoop(np.array([-5.,5.75]))
                #    sleep(1.5)
                #    xcount = xcount + 1
                #elif xcount == 3:
                #    sc.driveOpenLoop(np.array([5,0]))
                #    sleep(1)
                #    xcount = 0
                #else:
                #    xcount = 1
                #    break
                #sc.driveOpenLoop(np.array([0.,0.]))


                
    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        pass

    finally:
    	print("Exiting Color Tracking.")

if __name__ == '__main__':
    main()
