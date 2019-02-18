import numpy as np
from scipy.spatial import distance as dist
import argparse
import copy
import math
import os
from PIL import Image
import glob
import cv2
import imutils
from deepgaze.motion_tracking import ParticleFilter
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import time
from drawnow import *



## Particle Filter Parameters
height = 480
width = 640
#Filter parameters
tot_particles = 100
#Standard deviation which represent how to spread the particles
#in the prediction phase.
std = 5.2
#Init the filter
my_particle = ParticleFilter(height, width, tot_particles)
#Black image, window, bind the function
img = np.zeros((height,width,3), np.uint8)


import numpy as np
from scipy.spatial import distance as dist
import argparse
import copy
import math
import os
from PIL import Image
import glob
import cv2


nFangle= []
Fangle= []


class robotClass:
    def __init__(self, pos = [], team = '-no team!-', ID = '-no ID!-'):
        self.pos = pos # centre coordinates [x,y]    
        self.team = team # team 'B'  or 'Y'
        self.angle = 0 # angle of orientation 
        self.ID = ID # ID of robot on a team
        # ID markings
        self.circles = [] # [x,y,color]

    # this is a method to add a new circle for IDing the robot
    def newMarking(self, circle = [0,0,[0,0,0]]): 
        self.circles.append(circle)

class ballClass:
    def __init__(self, x = 0, y = 0):
        self.pos = [x, y]

roboList = []       # holds all robots currently seen- resets every loop
roboIDmarks = []    # holds all potential robot ID marks seen ('G' or 'P')
ball = None         # holds ball position in ballClass type object
IDdRobots = []      # potentially used for previous state- probably updated every loop

# colorID()
# E.H., Nov. 24th, 2018
# This function recognizes the colors of the recognized circle depending upon the BRG 
# color values. Averaging of colors may also be implemented in this function.
#              (input) -> (function) -> (output)
#             (B, G, R) -> colorID() -> 'color'
# v1:
# **note: Implement some form of color averaging across the circle
#         This will improve the reliability of the system
#         (less false positive IDs on a robot)
# v2:
#       Uses HSV instead of RGB- allows the hue to be better selected, assigning colors
#       much more accurately
def colorID(hue, sat, val):
    color = 'X' # Default case, 'X' will print an error for an unrecognized element
    if(val > 40):
        if (hue < 137 and hue >= 90):
            color = 'B' # Blue team circle
        elif (hue < 35 and hue >= 23):
            color = 'Y' # Yellow team circle
        elif ((hue >= 148 or hue < 7) and sat < 165): # Must address loop in hue
            color = 'P' # Purple ID circle
        elif (hue < 75 and hue >= 43):
            color = 'G' # Green ID circle
        elif ((hue < 23 and hue >= 8) and sat > 150):
            color = 'O' # Ball!

    return color 


# IDcircle()
# K.C. & E.H., Nov. 24th, 2018
# This function identifies any given circle based on its color and location. Although it
# does not return anything, it assigns the circle to its appropriate global variable
# (a robot object, ID mark list or ball object).
#              (input) -> (function) -> (output)
#               [x,y,r] -> ID_circle() -> none
# v1:
# Must implement all identifying functions. closestBot() is not fully developed, and must
# be added when completed.
def IDcircle(img, circle):
    global ball # so we can assign the ball its position globally

    x=int(circle[0])
    y=int(circle[1])

    hue = img[y,x,0]
    sat = img[y,x,1]
    val = img[y,x,2]
    color = colorID(hue, sat, val)
    #print('Circle is ', color)

    # if its blue or (if its yellow) --> Robot center/new robot
    if (color == 'B') or (color == 'Y'):
        roboList.append(robotClass([x,y],color))
        #print('Robot seen!')

    # if green or purple --> Robot ID marking
    elif (color == 'G') or (color == 'P'):        
        roboIDmarks.append([x,y,color])
            
    # if orange --> Ball location
    elif (color == 'O'):
        ball = ballClass(x,y)

# assignIDmarks()
# E.H., Dec, 2018
# This function cycles through the globally stored roboIDmarks list and assigns them to the
# closest available robot, provided they don't already have 4 assigned
def assignIDmarks(robot):
    #if isinstance(roboList, type(None)) == 0:
        # Assign each robot its four closest marks
        #for robot in roboList:
    closestMarks = [0,0,0,0] # indices of the closest four marks to the robot center
                                # [index in roboIDmarks, euclidean distance]
    furthestMark = [0,0] # [index in closestMarks, euclidean distance]

    # Assign this robot its four closest marks
    for i, mark in enumerate(list(roboIDmarks)):
        markDist = dist.euclidean([mark[0],mark[1]],robot.pos)

        # If there aren't already four marks given to the robot, 
        # just give it whatever is available in order to initialize robot.circles[]
        if len(robot.circles) < 4:
            robot.newMarking(mark)
            closestMarks[i] = [i, markDist]
            if markDist > furthestMark[1]:
                furthestMark = [i, markDist]

        # If there is a closer value than the furthest currently in robot.circles[]
        # replace the current furthest with this new one 
        elif markDist < furthestMark[1]:
            robot.circles[furthestMark[0]] = mark
            closestMarks[furthestMark[0]] = [i, markDist]

            furthestMark[1] = markDist

            # redetermine the furthest mark within the current closest marks
            for j, qark in enumerate(closestMarks):
                if qark[1] > furthestMark[1]:
                    furthestMark = [j, qark[1]]
                
    # * The below code was intended to shorten the list of circles in order to 
    # improve efficiency, however it had an error due to the index provided by
    # "wark[0]" being incorrect after the element in the previous iteration 
    # was deleted... *
    ## Remove the marks that were assigned to a robot- this will potentially make 
    ## assigning the rest of the marks much quicker with larger numbers of robots
    #for wark in closestMarks:
    #    del roboIDmarks[wark[0]]
    #else:
    #    print("No robots detected, but there are ID marks..?")

# RoboID()
# E.H., Jan, 2019
# This function reads the sorted robot.circles list and assigns an ID (robot.ID = x)
# to the robot. If the IDs are not properly sorted, this will not work
# v1:
# Doesn't have all IDs implemented yet
def RoboID(robot):
    #for robot in roboList:
    if len(robot.circles) == 4:
        if robot.circles[0][2] == 'P':
            if robot.circles[1][2] == 'P':
                if robot.circles[2][2] == 'P':
                    if robot.circles[3][2] == 'P':
                        robot.ID = 'ID9'
                    elif robot.circles[3][2] == 'G':
                        robot.ID = 'ID4'
                elif robot.circles[2][2] == 'G':
                    if robot.circles[3][2] == 'P':
                        robot.ID = 'ID3'
                    elif robot.circles[3][2] == 'G':
                        robot.ID = 'ID4'
            elif robot.circles[1][2] == 'G':
                if robot.circles[2][2] == 'P':
                    if robot.circles[3][2] == 'P':
                        robot.ID = 'ID5'
                    elif robot.circles[3][2] == 'G':
                        robot.ID = 'ID6'
                elif robot.circles[2][2] == 'G':
                    if robot.circles[3][2] == 'P':
                        robot.ID = 'ID7'
                    elif robot.circles[3][2] == 'G':
                        robot.ID = 'ID8'

# angle()
# E.H., Jan, 2019
# This function determines the angle of a passed robot using the IDs assigned to the robot
# by observing relative positions of said IDs. The angle determined is assigned to the robot
# at the end of the function, and reassignIDs() is called at the end to allow for identification
# of the robot.
#              (input) -> (function) -> (output)
#               ID marks -> angle() -> robot.angle
def angle(robot):	
    topIDs = [] # i.e. the two circles on the flat end of the robot
    bottomIDs = []
    theta1 = 999 # An impossible number for if statements later
    theta2 = 999

    # Uses the cosine law to figure out the angle every possible combo of ID circles makes
    # with the center of the robot (team ID), assigning to top or bottom IDs based on this angle
    for ii in range(len(robot.circles)-1):
        for jj in range(ii + 1, len(robot.circles)):
            temp1 = robot.circles[ii]
            temp2 = robot.circles[jj]

            # Determining distance between the different IDs
            a = dist.euclidean([temp1[0],temp1[1]],robot.pos) # Distance from ID 1 to centre
            b = dist.euclidean([temp2[0],temp2[1]],robot.pos) # Distance from ID 2 to centre
            c = dist.euclidean([temp1[0],temp1[1]],[temp2[0],temp2[1]]) # Distance from ID 1 to ID 2

            theta = math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

            if theta > 100 and theta < 130: # Ideally 114.84 degrees
                topIDs.append(temp1)
                topIDs.append(temp2)
                     
            if theta > 45 and theta < 75: # Ideally 65.16 degrees
                bottomIDs.append(temp1)
                bottomIDs.append(temp2)

            # the other ID pairs will be either ~180 or ~90 degrees

    # Takes the top two IDs and their average position, creating a vector to that point from the
    # center of the robot which the robot's angle can be derived from
    if len(topIDs) == 2:
        xMean = (topIDs[0][0] + topIDs[1][0])/2
        yMean = (topIDs[0][1] + topIDs[1][1])/2

        xDiff = xMean - robot.pos[0]
        yDiff = yMean - robot.pos[1]
        # Angle points in the direction the robot is facing
        theta1 = math.degrees(math.atan2(yDiff,xDiff))
    else:
        print("top went wrong...")
        
    # Takes the bottom two IDs and their average position, creating a vector from that point to
    # the center of the robot which the robot's angle can be derived from
    # (this is the opposite direction from the other one so the angle will be the same)
    if len(bottomIDs) == 2:
        xMean2 = (bottomIDs[0][0] + bottomIDs[1][0])/2
        yMean2 = (bottomIDs[0][1] + bottomIDs[1][1])/2

        xDiff2 = robot.pos[0] - xMean2
        yDiff2 = robot.pos[1] - yMean2
        # Negative for both of these to get an angle that is front facing
        theta2 = math.degrees(math.atan2(yDiff2,xDiff2))
    else:
        print("bottom is wrong")

    # Averages the vectors to get a better approx of the true angle
    if theta2 != 999 and theta1 != 999:
        xMean = (math.cos(math.radians(theta1)) + math.cos(math.radians(theta2)))/2
        yMean = (math.sin(math.radians(theta1)) + math.sin(math.radians(theta2)))/2
        theta = math.degrees(math.atan2(yMean,xMean))
        robot.angle = theta

    # If one of the vector calcs failed, just take the one that worked
    elif theta2 != 999 and theta1 == 999:
        theta = theta2
        robot.angle = theta
    elif theta2 == 999 and theta1 != 999:
        theta = theta1
        robot.angle = theta
    else:
        return "ERROR"

    reassignIDs(robot,topIDs,bottomIDs)

# reassignIDs()
# E.H., Jan, 2019
# This function reassigns the IDs found in a robot depending on the angle the robot
# is facing. It is helpful to draw out a visualization of this to understand why 
# certain angles are associated with certain indices in robot.circles
#              (input) -> (function) -> (output)
#              angle -> reassignIDs() -> robot.circles (sorted)
def reassignIDs(robot,topIDs,bottomIDs):
    # Reassignment of IDs only works if all four have been recognized
    if len(robot.circles) == 4 and len(topIDs) == 2 and len(bottomIDs) == 2:
        # I suggest drawing this out if you're having a hard time visualizing it-
        # see the design document for further detail on which ID is which
        if robot.angle <= 45 and robot.angle >= -45:
            if topIDs[0][1] > topIDs[1][1]:
                robot.circles[0] = topIDs[1]
                robot.circles[1] = topIDs[0]
            else:
                robot.circles[0] = topIDs[0]
                robot.circles[1] = topIDs[1]
            if bottomIDs[0][1] > bottomIDs[1][1]:
                robot.circles[2] = bottomIDs[1]
                robot.circles[3] = bottomIDs[0]
            else:
                robot.circles[2] = bottomIDs[0]
                robot.circles[3] = bottomIDs[1]
        if robot.angle < 135 and robot.angle > 45:
            if topIDs[0][0] > topIDs[1][0]:
                robot.circles[0] = topIDs[0]
                robot.circles[1] = topIDs[1]
            else:
                robot.circles[0] = topIDs[1]
                robot.circles[1] = topIDs[0]
            if bottomIDs[0][0] > bottomIDs[1][0]:
                robot.circles[2] = bottomIDs[0]
                robot.circles[3] = bottomIDs[1]
            else:
                robot.circles[2] = bottomIDs[1]
                robot.circles[3] = bottomIDs[0]
        if robot.angle < -45 and robot.angle > -135:
            if topIDs[0][0] > topIDs[1][0]:
                robot.circles[0] = topIDs[1]
                robot.circles[1] = topIDs[0]
            else:
                robot.circles[0] = topIDs[0]
                robot.circles[1] = topIDs[1]
            if bottomIDs[0][0] > bottomIDs[1][0]:
                robot.circles[2] = bottomIDs[1]
                robot.circles[3] = bottomIDs[0]
            else:
                robot.circles[2] = bottomIDs[0]
                robot.circles[3] = bottomIDs[1]
        if robot.angle <= -135 or robot.angle >= 135:  # must be "or", as sign swaps at 180
            if topIDs[0][1] > topIDs[1][1]:
                robot.circles[0] = topIDs[0]
                robot.circles[1] = topIDs[1]
            else:
                robot.circles[0] = topIDs[1]
                robot.circles[1] = topIDs[0]
            if bottomIDs[0][1] > bottomIDs[1][1]:
                robot.circles[2] = bottomIDs[0]
                robot.circles[3] = bottomIDs[1]
            else:
                robot.circles[2] = bottomIDs[1]
                robot.circles[3] = bottomIDs[0]



def PF_predict_center(img):
    for robot in roboList:
        x= robot.pos[0]
        y= robot.pos[1]


        my_particle.predict(x_velocity=0, y_velocity=0, std=std)

        #Estimate the next position using the internal model
        x_estimated, y_estimated, _, _ = my_particle.estimate() 
 
        #Update the position of the particles based on the measurement.
        #Adding some noise to the measurement.
        noise_coefficient = np.random.uniform(low=0.1, high=0.8)
        x_measured = x + np.random.randn() * noise_coefficient
        y_measured = y + np.random.randn() * noise_coefficient
        my_particle.update(x_measured, y_measured)

        #Drawing the circles for the mouse position the
        #estimation and the particles.
        for i in range(0, tot_particles):
            x_particle, y_particle = my_particle.returnParticlesCoordinates(i)
            cv2.circle(img,(x_particle, y_particle),2,(0,0,255),-1) #RED: Particles
        cv2.circle(img,(x, y),2,(0,255,0),-1) #GREEN: Mouse position
        cv2.circle(img,(x_estimated, y_estimated),2,(255,0,0),-1) #BLUE: Filter estimation

        robot.pos[0]= int(x_measured)
        robot.pos[1]= int(y_measured)


        #Print general information
        print("Total Particles: " + str(tot_particles))
        print("Effective N: " + str(my_particle.returnParticlesContribution()))
        print("Measurement Noise: " + str(noise_coefficient) + "/10")
        print("x=" + str(x) + "; y=" + str(y) + " | " + 
              "x_measured=" + str(int(x_measured)) + "; y_measured=" + str(int(y_measured))  + " | " +
              "x_estimated=" + str(int(x_estimated)) + "; y_estimated=" + str(int(y_estimated)))
        #print(my_particle.weights) #uncomment to print the weights
        #print(my_particle.particles) #uncomment to print the particle position
        print("")

        #if(my_particle.returnParticlesContribution() < 8):
        my_particle.resample()


def PF_predict_IDs():
    for robot in roboList:
        for circle in range(len(robot.circles)):
            x= robot.circles[circle][0]
            y= robot.circles[circle][1]


            my_particle.predict(x_velocity=0, y_velocity=0, std=std)

            #Estimate the next position using the internal model
            x_estimated, y_estimated, _, _ = my_particle.estimate() 
 
            #Update the position of the particles based on the measurement.
            #Adding some noise to the measurement.
            noise_coefficient = np.random.uniform(low=0.1, high=0.8)
            x_measured = x + np.random.randn() * noise_coefficient
            y_measured = y + np.random.randn() * noise_coefficient
            my_particle.update(x_measured, y_measured)

            #Drawing the circles for the mouse position the
           # estimation and the particles.
            for i in range(0, tot_particles):
                x_particle, y_particle = my_particle.returnParticlesCoordinates(i)
                cv2.circle(img,(x_particle, y_particle),2,(0,0,255),-1) #RED: Particles
            cv2.circle(img,(x, y),2,(0,255,0),-1) #GREEN: Mouse position
            cv2.circle(img,(x_estimated, y_estimated),2,(255,0,0),-1) #BLUE: Filter estimation

            robot.circles[circle][0]= int(x_measured)
            robot.circles[circle][1]= int(y_measured)


            #Print general information
            print("Total Particles: " + str(tot_particles))
            print("Effective N: " + str(my_particle.returnParticlesContribution()))
            print("Measurement Noise: " + str(noise_coefficient) + "/10")
            print("x=" + str(x) + "; y=" + str(y) + " | " + 
                  "x_measured=" + str(int(x_measured)) + "; y_measured=" + str(int(y_measured))  + " | " +
                  "x_estimated=" + str(int(x_estimated)) + "; y_estimated=" + str(int(y_estimated)))
            #print(my_particle.weights) #uncomment to print the weights
            #print(my_particle.particles) #uncomment to print the particle position
            print("")

            #if(my_particle.returnParticlesContribution() < 8):
            my_particle.resample()




plt.ion()

def makeFig():
   # plt.ylim(-25,-15)
    plt.title('Angle Measurement')
    plt.grid(True)
    plt.ylabel("Measured Angle using PF (deg)")
    plt.plot(nFangle, 'ro-')

    plt2=plt.twinx()
    #plt.ylim(-25,-15)
    plt2.plot(Fangle, 'b^-')



def main():
    # Declaring global variables so they can be cleared every loop
    global roboList
    global roboIDmarks
    global circles 
    global ball
    global IDdRobots

    flag =  0

    cap = cv2.VideoCapture(1) # 0 if your pc doesn't have a webcam, probably 1 if it does
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while(True):
        ret,frame = cap.read() # reading the video capture into a dummy var and frame
        
        # Reinitializing robot data (prevents buildup of data accross frames)
        roboList = []
        roboIDmarks= []
        circles = []
        ball = None

        # Histogram equalization for colors (haven't tested with this)
        #img_yuv = cv2.cvtColor(ii, cv2.COLOR_BGR2YUV)

        ### equalize the histogram of the Y channel
        #img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

        ### convert the YUV image back to RGB format
        #frame_yuv = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

        # HSV color masking
        hsv= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        lower_rangeG = np.array([0,0,30])
        upper_rangeG = np.array([180,255,255])

        mask = cv2.inRange(hsv, lower_rangeG, upper_rangeG) # mask for original frame with only good color
        result = cv2.bitwise_and(frame,frame,mask=mask)
        cv2.imshow("masked image",result)
        #cv2.imshow("masked dafe",frame)
  
        hsv_out_gray= cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        blurred_img = cv2.bilateralFilter(hsv_out_gray,9,75,75)

        # Some notes on the HoughCircles function:
        #  Utilizes edge detection to draw tangent lines, recognizing a circle where perpendicular lines to tangents
        #  meet, depending on the intensity of the intersecting tangent lines.
        #  param1: higher threshold for Canny edge detection (lower is half of this)
        #  param2: accumulator threshold for circle center detection- i.e. the lower it is, the less circular an object
        #          needs to be to be recognized as a circle
        #  minDist: Specifies minimum distance between circles (the 4th input to the function)
        #  
        # from documentation: cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) → circles
        circles = cv2.HoughCircles(blurred_img,cv2.HOUGH_GRADIENT,1,minDist = 20,param1=50,param2=25,minRadius=20,maxRadius=50)

        cv2.waitKey(1) # cv2.waitKey() is required to display images- waits 1 millisecond here

        img = copy.deepcopy(frame) # Sometimes if you copy stuff in Python, changes made to a copied variable end up in original
                                   # which necessitates a deepcopy

        if isinstance(circles, type(None)) == 0:
            for circle in circles[0,:]:
                IDcircle(hsv, circle) # ID all the circles recognized by color
                # draw the outer circle
                cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)

            if isinstance(ball, type(None)) == 0:
                print('Ball found at ',ball.pos)
                # Draw a blue circle on the ball
                cv2.circle(img,(ball.pos[0],ball.pos[1]),10,(200,0,0),5)  

            if (isinstance(roboIDmarks, type(None)) == 0) & (isinstance(roboList, type(None)) == 0):
                for robot in roboList:
                    assignIDmarks(robot) # Assign the ID marks observed to their appropriate robot
                    angle(robot)         # Determine angle of robots seen
                    nFangle.append(robot.angle)
                    PF_predict_center(img)
                    PF_predict_IDs()
                    angle(robot)         # Determine angle of robots seen
                    Fangle.append(robot.angle)
                    RoboID(robot)        # Give robots seen an ID

                    # Draw the robot circles seen robot by robot
                    # Draw a black circle on the centre of the robot
                    cv2.circle(img,(robot.pos[0],robot.pos[1]),10,(0,0,0),5)
                    if isinstance(robot.angle, type(None)) == 0:
                        # Display the robot's angle
                        cv2.putText(img, str(round(robot.angle,1)), (robot.pos[0]+ 100, robot.pos[1] + 130), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                        # Display the robot's position
                        cv2.putText(img, str(robot.pos), (robot.pos[0]+ 100, robot.pos[1] + 100), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                        # Display the robot's ID
                        cv2.putText(img, robot.ID, (robot.pos[0]+ 100, robot.pos[1] + 70), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                    for mark in robot.circles:
                        # Draw a black circle on every ID mark
                        cv2.circle(img,(mark[0],mark[1]),10,(0,0,0),5)  
            flag = 0 # go ahead and print "no circles detected" again

        elif(flag == 0):
            print("no circles detected")
            flag = 1 # don't print this again


        

        drawnow(makeFig)

        # Display drawn on frame and original frame
        cv2.imshow('circles on stream',img)
        #cv2.imshow('original stream',frame)
        #cv2.waitKey(250)
        if cv2.waitKey(1) & 0xFF == ord('\r'): # if enter is pressed, stop running
            break


    cv2.destroyAllWindows(0)
 
 




 
if __name__== "__main__":
    main()



