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
    valid = np.where(dist > 0.27)                  # find values 16mm
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

while True:
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
    if distance[0] <= 0.4  and distance[1] < 70 and distance[1] > 0:       #condition if object is too close in front left, turn 90ish degrees to the right
        print("Obstacle Front Left")
        motor.sendLeft(0.8)                                                #left wheel reverse
        motor.sendRight(-0.8)                                                #right wheel forward
        sleep(1)                                                            #continue turn for 2 seconds
        motor.sendLeft(.8)
        motor.sendRight(1)
        sleep(2)
        motor.sendLeft(-.8)
        motor.sendRight(.8)
        sleep(1)
    elif distance[0] <= 0.4 and distance[1] > -70 and distance[1] < 0:      #condition if object is too close in front right, turn 90ish degrees to the left
        print("Obstacle Front Right")
        motor.sendLeft(-0.8)                                                 #left wheel forward
        motor.sendRight(0.8)                                               #right wheel reverse
        sleep(1)                                                            #continue turn for 2 seconds
        motor.sendLeft(.8)
        motor.sendRight(1)
        sleep(2)                                                            #continue turn for 2 seconds
        motor.sendLeft(.8)
        motor.sendRight(-.8)
        sleep(1)
    else:
        motor.sendLeft(0.8)                                                 #left wheel forward
        motor.sendRight(1)                                                #right wheel forward