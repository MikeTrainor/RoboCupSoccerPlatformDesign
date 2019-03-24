import numpy as np
from scipy.spatial import distance as dist
import copy
import math
import cv2
import serial
import time
from fractions import gcd
import sys
from numpy.linalg import multi_dot
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QMainWindow
from PyQt5.QtWidgets import QSizePolicy, QSlider, QRadioButton
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QSize, Qt
from scipy import interpolate
import matplotlib.pyplot as plt
import pygame.math as vmath

#import GUI_attempt

#KL25=serial.Serial('COM6',256000,timeout=1)#open serial port
# two Lists used for real time plottting of the left motor and right motor
Rmotor= []
Lmotor= []

class robotClass:
    def __init__(self, pos = [], radius = 0, team = '-no team!-', ID = '-no ID!-'):
        self.pos = pos # centre coordinates [x,y]
        self.radius = radius # calculated radius of robot (based on centre dot)
        self.team = team # team 'B'  or 'Y'
        self.angle = 999 # angle of orientation 
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

param1val = 75
param2val = 15
valueMin = 125

def sampleCubicSplinesWithDerivative(points, tangents, resolution):
    '''
    Compute and sample the cubic splines for a set of input points with
    optional information about the tangent (direction AND magnitude). The 
    splines are parametrized along the traverse line (piecewise linear), with
    the resolution being the step size of the parametrization parameter.
    The resulting samples have NOT an equidistant spacing.

    Arguments:      points: a list of n-dimensional points
                    tangents: a list of tangents
                    resolution: parametrization step size
    Returns:        samples

    Notes: Lists points and tangents must have equal length. In case a tangent
           is not specified for a point, just pass None. For example:
                    points = [[0,0], [1,1], [2,0]]
                    tangents = [[1,1], None, [1,-1]]

    '''
    resolution = float(resolution)
    points = np.asarray(points)
    nPoints, dim = points.shape

    # Parametrization parameter s.
    dp = np.diff(points, axis=0)                 # difference between points
    dp = np.linalg.norm(dp, axis=1)              # distance between points
    d = np.cumsum(dp)                            # cumsum along the segments
    d = np.hstack([[0],d])                       # add distance from first point
    l = d[-1]                                    # length of point sequence
    nSamples = int(l/resolution)                 # number of samples
    s,r = np.linspace(0,l,nSamples,retstep=True) # sample parameter and step

    # Bring points and (optional) tangent information into correct format.
    assert(len(points) == len(tangents))
    data = np.empty([nPoints, dim], dtype=object)
    for i,p in enumerate(points):
        t = tangents[i]
        # Either tangent is None or has the same
        # number of dimensions as the point p.
        assert(t is None or len(t)==dim)
        fuse = list(zip(p,t) if t is not None else zip(p,))
        data[i,:] = fuse

    # Compute splines per dimension separately.
    samples = np.zeros([nSamples, dim])
    for i in range(dim):
        poly = interpolate.BPoly.from_derivatives(d, data[:,i])
        samples[:,i] = poly(s)
    return samples


def colorID(hue, sat, val):
    color = 'X' # Default case, 'X' will print an error for an unrecognized element
    if(val > valueMin):
        if (hue < 128 and hue >= 90):
        #if (hue < 250 and hue >= 150):
            color = 'B' # Blue team circle
        elif (hue < 35 and hue > 25 and sat > 80):
            color = 'Y' # Yellow team circle
        elif (hue >= 128 or (hue <= 9 and sat < 120)): # Must address loop in hue
            color = 'P' # Purple ID circle
        elif (hue < 90 and hue >= 43):
            color = 'G' # Green ID circle
        elif ((hue <= 12 and hue >= 3 and sat > 40) or (hue <=35 and hue >12 and sat < 80)):
            color = 'O' # Ball!
            #print(hue,sat,val)
        #else:
        #    print("unrecognized",hue,sat,val) # good for debugging unrecognized circles
        #    cv2.waitKey(1000)
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
    #r=int(circle[2])
    
    # Getting spaced pixels within the circle
    #diffx = [int(x+r/2), int(x-r/2)]
    #for idx,posx in list(enumerate(diffx)):
    #    if(posx >= len(img[0])):
    #        diffx[idx] = x
    #    elif(posx < 0):
    #        diffx[idx] = 0
    #diffy = [int(y+r/2), int(y-r/2)]
    #for idx2,posy in list(enumerate(diffy)):
    #    if(posy >= len(img)):
    #        diffy[idx2] = y
    #    elif(posy < 0):
    #        diffy[idx2] = 0
    #print("Colors",img[y,x,0],img[y,x,1],img[y,x,2])
    hue = img[y,x,0]
    sat = img[y,x,1]
    val = img[y,x,2]
    # taking the average hue, saturation and value of a circle
    #hue = sum([img[y,x,0],img[y,diffx[0],0],img[y,diffx[1],0],img[diffy[0],x,0],img[diffy[1],x,0]])/5
    #sat = sum([img[y,x,1],img[y,diffx[0],1],img[y,diffx[1],1],img[diffy[0],x,1],img[diffy[1],x,1]])/5
    #val = sum([img[y,x,2],img[y,diffx[0],2],img[y,diffx[1],2],img[diffy[0],x,2],img[diffy[1],x,2]])/5

    # getting the color of the circle with the averaged pixel
    color = colorID(hue, sat, val)
    #print(hue, sat, val,'   so the Circle is ', color)
    #cv2.waitKey(1000)

    # if its blue or (if its yellow) --> Robot center/new robot
    if (color == 'B') or (color == 'Y'):
        # passes in position [x,y], radius and color
        # where the radius is calced with: 25 radius centre in 85 radius robot so ~ 3.4 times radius
        roboList.append(robotClass([x,y],circle[2]*4,color))
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
    closestMarks = [] # indices of the closest four marks to the robot center
                                # [index in roboIDmarks, euclidean distance]
    #furthestMark = [0,0] # [index in closestMarks, euclidean distance]

    # Assign this robot its four closest marks
    #for i, mark in enumerate(list(roboIDmarks)):
    for mark in roboIDmarks:
        markDist = dist.euclidean([mark[0],mark[1]],robot.pos)

        if(markDist < robot.radius and len(robot.circles) < 4):
            robot.newMarking(mark)
            closestMarks.append(markDist)
        elif(markDist < robot.radius):
            for i, currentcircle in enumerate(list(robot.circles)):
                if markDist < closestMarks[i]:
                    robot.circles[i] = mark
                    closestMarks[i] = markDist


        # If there aren't already four marks given to the robot, 
        # just give it whatever is available in order to initialize robot.circles[]
        #if len(robot.circles) < 4:
        #    if markDist < robot.radius:
        #        robot.newMarking(mark)
        #    closestMarks[i] = [i, markDist]
        #    if markDist > furthestMark[1]:
        #        furthestMark = [i, markDist]

        ## If there is a closer value than the furthest currently in robot.circles[]
        ## replace the current furthest with this new one 
        #elif markDist < furthestMark[1]:
        #    if markDist < robot.radius:
        #        robot.circles[furthestMark[0]] = mark
        #    closestMarks[furthestMark[0]] = [i, markDist]

        #    furthestMark[1] = markDist

        #    # redetermine the furthest mark within the current closest marks
        #    for j, qark in enumerate(closestMarks):
        #        if qark[1] > furthestMark[1]:
        #            furthestMark = [j, qark[1]]

        #else:
            #print(robot.radius)
                
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
# v2:
# Has all IDs implemented
def RoboID(robot):
    #for robot in roboList:
    if len(robot.circles) == 4:
        if robot.circles[0][2] == 'P': # circle 1
            if robot.circles[1][2] == 'P': # circle 2
                if robot.circles[2][2] == 'P': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID9'
                    elif robot.circles[3][2] == 'G': # circle 4
                        robot.ID = 'ID4'
                elif robot.circles[2][2] == 'G': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID0'
                    elif robot.circles[3][2] == 'G': # circle 4
                        robot.ID = 'ID10'
            elif robot.circles[1][2] == 'G': # circle 2
                if robot.circles[2][2] == 'P': # circle 3
                    if robot.circles[3][2] == 'G': # circle 4
                        robot.ID = 'ID7'
                elif robot.circles[2][2] == 'G': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID3'
        elif robot.circles[0][2] == 'G': # circle 1
            if robot.circles[1][2] == 'P': # circle 2
                if robot.circles[2][2] == 'P': # circle 3
                    if robot.circles[3][2] == 'G': # circle 4
                        robot.ID = 'ID5'
                elif robot.circles[2][2] == 'G': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID1'
            elif robot.circles[1][2] == 'G': # circle 2
                if robot.circles[2][2] == 'P': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID11'
                    elif robot.circles[3][2] == 'G': # circle 4
                        robot.ID = 'ID6'
                elif robot.circles[2][2] == 'G': # circle 3
                    if robot.circles[3][2] == 'P': # circle 4
                        robot.ID = 'ID2'
                    elif robot.circles[3][2] == 'G': # circle 4
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
            try:
                theta = math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b))) #CRASHES ON RARE OCCASIONS
            except:
                print('Theta Error')
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
    #else:
        #print("top went wrong...")
        
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
    #else:
    #    print("bottom is wrong")

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
    return

# stoprobot()
# E.H., Mar, 2019
# This function will stop any robots passed into it by issuing stop commands to 
# the specified robot. Alternatively, if 'all' is passed in, it will stop all robots.
def stoprobot(ID):
    packet = bytearray()
    if(ID == 'all'): # Send stop commands to all robots
        for robot in range(0,12):
            packet.append(0xFF) # start bit
            packet.append(robot)  #Robot ID
            packet.append(0)
            packet.append(0)
            packet.append(0)
            packet.append(0)
            packet.append(0)
            packet.append(0xFF) # stop bit
            KL25.write(packet)
            packet = bytearray()
    elif(ID >= 0 and ID <= 11): # Only accept valid IDs
        packet.append(0xFF) # start bit
        packet.append(ID)  #Robot ID
        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet.append(0)
        packet.append(0xFF) # stop bit
        KL25.write(packet)


## PD controller Parametters
error1=0
error_prior1=0
error2=0
error_prior2=0

#Motor Speed Parameters
VrMax = 1000
VlMax = VrMax 
VrMin = -0x10
VlMin = VrMin

## The dt variable should be variable based on the number of frame rates obtained by the CV system. 
####### This frame rate should be calculated. #########
dt=1/10             
derivative1=0       
L=18                #Robot Diameter
R=3.5               #Wheel Radius
dirR=0
dirL=0
umax=575            #Max input for position control
u2max= 220
kp1= 2.65
kp2= 1.2
flag =  0
kd1=1.5
kd2=1.5
temp1=0
temp2=0
test = 0
counter = 0
points = []
tangents = []

   
cap = cv2.VideoCapture(cv2.CAP_DSHOW + 0) # 0 if your pc doesn't have a webcam, probably 1 if it does
def mainLoop():
    print("new loop\n\r")
    # Declaring global variables so they can be cleared every loop
    global roboList, roboIDmarks, circles, ball, IDdRobots

    global error1,error_prior1,error2,error_prior2,dt,derivative1,L,R,dirR,dirL,umax,u2max,kp1
    global kp2,flag,kd1,kd2,VrMax,VlMax,temp1,temp2,test,counter

    #Mikes Global Variables
    global points
    global tangents

   
    #cap = cv2.VideoCapture(cv2.CAP_DSHOW + 0) # 0 if your pc doesn't have a webcam, probably 1 if it does
    # https://stackoverflow.com/questions/52043671/opencv-capturing-imagem-with-black-side-bars
    # MSMF doesn't like being scaled up apparently, so switch from it (default) to DirectShow
    # so we can scale up the resolution read from the camera

    # Scaling up from 640x480 to HD 1280x720
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    

    #while(True):
    #  while(KL25.inWaiting()==0):

    ret,frame = cap.read() # reading the video capture into a dummy var and frame

    #cv2.waitKey(50)
        
    # Reinitializing robot data (prevents buildup of data accross frames)
    roboList = []
    roboIDmarks= []
    circles = []
    ball = None

    #Mike's Added Value Parameters
    points = []
    tangents = []
    resolution = 0.2 #was 0.2

    # Histogram equalization for colors (haven't tested with this)
    #img_yuv = cv2.cvtColor(ii, cv2.COLOR_BGR2YUV)

    ### equalize the histogram of the Y channel
    #img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

    ### convert the YUV image back to RGB format
    #frame_yuv = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

    # blurring image for less errant circles and better color recognition later
    # d = 5 as that is the recommended nearest neighbour for real time
    # sigmaColor = 150 to produce large blending effect
    # sigmaSpace is limited by d, so I suspect it doesn't matter
    blurred_img = cv2.bilateralFilter(frame,8,150,150) 

    # HSV color space conversion
    hsv= cv2.cvtColor(blurred_img,cv2.COLOR_BGR2HSV)

    # Color masking, not necessary due to blurring, but might be worth looking into further
    #lower_rangeG = np.array([0,0,0]) # Hue, Saturation, Value mask lower limit
    #upper_rangeG = np.array([180,255,255]) # " , " , " " upper limit

    #mask = cv2.inRange(hsv, lower_rangeG, upper_rangeG) # mask for original frame with only good color
    #result = cv2.bitwise_and(blurred_img,blurred_img,mask=mask)
    result = blurred_img

    #cv2.imshow("blurred image",result)
  
    hsv_out_gray= cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

    #cv2.imshow("houghin",hsv_out_gray)

    # Some notes on the HoughCircles function:
    #  Utilizes edge detection to draw tangent lines, recognizing a circle where perpendicular lines to tangents
    #  meet, depending on the intensity of the intersecting tangent lines.
    #  param1: higher threshold for Canny edge detection (lower is half of this)
    #  param2: accumulator threshold for circle center detection- i.e. the lower it is, the less circular an object
    #          needs to be to be recognized as a circle
    #  minDist: Specifies minimum distance between circles (the 4th input to the function)
    #  
    # from documentation: cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) → circles
    circles = cv2.HoughCircles(hsv_out_gray,cv2.HOUGH_GRADIENT,1,minDist=5,param1=param1val,param2=param2val,minRadius=1,maxRadius=15)

    cv2.waitKey(1) # cv2.waitKey() is required to display images- waits 1 millisecond here

    img = copy.deepcopy(frame) # Sometimes if you copy stuff in Python, changes made to a copied variable end up in original
                                # which necessitates a deepcopy

    #DELETE THIS BLOCK ############
    #test_circle = cv2.ellipse(img,(600,220),(100,100),180,90,-90,255,5)
    #test_line = cv2.line(img, (230,300), (600,220), 255, 5)
    #nx, ny = (500,1) #500 colomns by 1 rows vector]
    #x = np.linspace(230,600,nx) #x vector
    #y = np.linspace(300,220,nx) #y vector, start and end reverse
    #xpoint = x[round(len(x)*8/10)] #Going to rounded 8/10ths the way through the x vector
    #ypoint = y[round(len(y)*8/10)] #Going to rounded 8/10ths the way through the y vector
    #print(xpoint)
    #print(ypoint)
    ###########################

    if isinstance(circles, type(None)) == 0:
        for circle in circles[0,:]:
            IDcircle(hsv, circle) # ID all the circles recognized by color
            # draw the outer circle
            cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)

        if isinstance(ball, type(None)) == 0:
            # Draw a blue circle on the ball
            cv2.circle(img,(ball.pos[0],ball.pos[1]),10,(200,0,0),5)  
            cv2.putText(img, str(ball.pos), (ball.pos[0]+20,ball.pos[1]+20), cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)

        if (isinstance(roboIDmarks, type(None)) == 0) & (isinstance(roboList, type(None)) == 0):
            for robot in roboList:
                assignIDmarks(robot) # Assign the ID marks observed to their appropriate robot
                angle(robot)         # Determine angle of robots seen
                RoboID(robot)        # Give robots seen an ID

                # Draw the robot circles seen robot by robot
                # Draw a black circle on the centre of the robot
                cv2.circle(img,(robot.pos[0],robot.pos[1]),10,(0,0,0),3)
                

                #if isinstance(robot.angle, type(None)) == 0:
                #    # Display the robot's angle
                #    cv2.putText(img, str(round(robot.angle,1)), (robot.pos[0]+ 100, robot.pos[1] + 130), 
                #                cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                #    # Display the robot's position
                #    cv2.putText(img, str(robot.pos), (robot.pos[0]+ 100, robot.pos[1] + 100), 
                #                cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                #    # Display the robot's ID
                #    cv2.putText(img, robot.ID, (robot.pos[0]+ 100, robot.pos[1] + 70), 
                #                cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                #    # Display the robot's Team
                #    cv2.putText(img, robot.team, (robot.pos[0]+ 100, robot.pos[1] + 40), 
                #                cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                for mark in robot.circles:
                    # Draw a black circle on every ID mark
                    cv2.circle(img,(mark[0],mark[1]),10,(0,0,0),3)  
        flag = 0 # go ahead and print "no circles detected" again

    elif(flag == 0):
        #print("no circles detected")
        flag = 1 # don't print this again

    # Display drawn on frame and original frame
    #cv2.imshow('circles on stream',img)
    cv2.imshow('original stream',frame)

    #if cv2.waitKey(1) & 0xFF == ord('\r'): # if enter is pressed, stop running
    #    break

    # when the ball does not get detected
    if (isinstance(ball, type(None)) != 0):
        ball = ballClass(temp1,temp2)  
    
    #This if statement is simply for initialization, there has to be a better way of doing this
    if(counter == 0):
        for rob in roboList:
            #Mike's added value stuff Initialization
            points = []
            tangents = []
            resolution = 0.2
            points.append([rob.pos[0],rob.pos[1]]) #Robot Position
            points.append([ball.pos[0],ball.pos[1]]) #Ball Position
            points.append([600,220]) #Net Position

            #Finding the angle at which the robot approaches
            approach_x = (points[2][0] - points[1][0])
            approach_y = (points[2][1] - points[1][1])
            common_divisor = abs(gcd(approach_x,approach_y)) #Absolute value of the greatest common divisor
            approach_x = approach_x/common_divisor #Divide by common divisor
            approach_y = approach_y/common_divisor #Divide by common divisor

            #Tangents for alligning robot with ball and net
            tangents.append([math.tan(45*np.pi/180),1]) #Robot position, Slope converted from radians, this value is whatever angle the robot is currently facing
            tangents.append([approach_x, approach_y]) #Ball position
            tangents.append([approach_x, approach_y]) #Net position
        
    packet = bytearray()                    # ** Should this be within the for loop below?
    #packet.append(0xff)
    #packet.append(0x01)  #id
    #packet.append(0x30)  #mtr1
    #packet.append(0x01)  #dir1
    #packet.append(0x30)  #mtr2
    #packet.append(0x01)  #dir2
    #packet.append(0x01)  #kick
    #packet.append(0xff)

    #KL25.write(packet)
    #data = KL25.read(4) #Reading and Printing slows down the system incredibly, do not use for the demonstration
    #print(data.decode('ISO-8859-1')) #Reading and Printing slows down the system incredibly, do not use for the demonstration

    for rob in roboList:
        if(rob.ID != '-no ID!-') & (isinstance(roboList, type(None)) == 0):
            robotsID = int(''.join(filter(str.isdigit,rob.ID))) # extracting integer ID number from rob.ID
            # if the robot has a radius larger than the distance between it and the edge of the frame
            # skip over this robot
            if(rob.radius < (rob.pos[1] - len(img[1])) or rob.radius < (rob.pos[0] - len(img[0]))):
                stoprobot(robotsID)
                continue
            else:
                if(abs(abs(rob.angle)-180)>20 and counter>5):
                    if (abs(rob.angle-test) >=200 and counter > 5):
                        rob.angle=test#something is wrong with the angle measurement
                    
                else:
                    if (abs(abs( rob.angle)-abs(test)) >=50 and counter>5 ):
                        rob.angle=test#something is wrong with the angle measurement
                test=rob.angle
                #print("rob.angle",rob.angle)
                # Display the robot's angle
                cv2.putText(img, str(round(rob.angle,1)), (rob.pos[0]+ 100, rob.pos[1] + 130), 
                            cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                # Display the robot's position
                cv2.putText(img, str(rob.pos), (rob.pos[0]+ 100, rob.pos[1] + 100), 
                            cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                # Display the robot's ID
                cv2.putText(img, rob.ID, (rob.pos[0]+ 100, rob.pos[1] + 70), 
                            cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                # Display the robot's Team
                cv2.putText(img, rob.team, (rob.pos[0]+ 100, rob.pos[1] + 40), 
                            cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)

                #Mike's Added Value Stuff
                points.append([rob.pos[0],rob.pos[1]]) #Robot Position
                points.append([ball.pos[0],ball.pos[1]]) #Ball Position
                points.append([600,220]) #Net Position

                #Finding the angle at which the robot approaches
                approach_x = (points[2][0] - points[1][0])
                approach_y = (points[2][1] - points[1][1])
                common_divisor = abs(gcd(approach_x,approach_y)) #Absolute value of the greatest common divisor
                approach_x = approach_x/common_divisor #Divide by common divisor
                approach_y = approach_y/common_divisor #Divide by common divisor

                #Tangents for alligning robot with ball and net
                tangents.append([math.tan(45*np.pi/180),1]) #Robot position, Slope converted from radians, this value is whatever angle the robot is currently facing
                tangents.append([approach_x, approach_y]) #Ball position
                tangents.append([approach_x, approach_y]) #Net position

                points = np.asarray(points)
                tangents = np.asarray(tangents)

                #Finding the angle at which the robot approaches
                approach_x = (points[2][0] - points[1][0])
                approach_y = (points[2][1] - points[1][1])
                common_divisor = abs(gcd(approach_x,approach_y)) #Absolute value of the greatest common divisor
                approach_x = approach_x/common_divisor #Divide by common divisor
                approach_y = approach_y/common_divisor #Divide by common divisor

                # Interpolate with different tangent lengths, but equal direction.
                scale = 0.01 #Tunable Parameter, the closer to 0 the tighter the spline
                tangents_new = np.dot(tangents, scale*np.eye(2))
                samples_new = np.float32(sampleCubicSplinesWithDerivative(points, tangents_new, resolution))

                #Find the slope to the next point
                next_point_x =  np.float32(samples_new[round(len(samples_new)/10)][0] - points[0][0]) #Change in x between the robot and next point
                next_point_y =  np.float32(samples_new[round(len(samples_new)/10)][1] - points[0][1]) #Change in y between the robot and next point
                tangents[0][0] = math.atan2(next_point_y,next_point_x) #This value is the angle in radians that the robot must face for its path
                tangents[0][1] = 1
                tangents[1][0] = approach_x #x slope
                tangents[1][1] = approach_y #y slope
                tangents[2][0] = approach_x #x slope
                tangents[2][1] = approach_y #y slope

                points = np.asarray(points)
                tangents = np.asarray(tangents)
                trajectory = math.degrees(tangents[0][0]) #Angle in degrees

                #Printing Mike's Stuff
                print(samples_new[round(len(samples_new)/10) + 1][0]) #X position with resolution of 10, if this is not good enough divide into smaller pieces perhaps
                print(samples_new[round(len(samples_new)/10) + 1][1]) #X position with resolution of 10, if this is not good enough divide into smaller pieces perhaps
                print(trajectory) #Angle output in degrees

                #Display splines on the live feed & plot
                #path_plot = plt.scatter(samples3[:,0], samples3[:,1], marker='o', label='samples3')
                k = 0
                for k in range(0,len(samples_new)):
                    cv2.circle(img, (samples_new[k,0], samples_new[k,1]), 1, (0, 255, 255),5)

                #image1 = path_plot.imshow(grab_frame(cap))
                #plt.ion()
                #plt.show()

                ####### Angle Control 
                if rob.angle == 999:
                    rob.angle=test

                #Mike's Added Value Deciding which error to use Based on Robot Position and Angle
                pos_error = ((ball.pos[0]-rob.pos[0])**2+(ball.pos[1]-rob.pos[1])**2)**0.5 #The absolute error between robot and ball
                angle_error = math.degrees((math.atan2(ball.pos[1]-rob.pos[1],ball.pos[0]-rob.pos[0])))-rob.angle #Angle error between robot and ball

                if(abs(pos_error) <= 10 and abs(angle_error) <=5): #if the robot is close to the ball and is lined up
                    error2 = math.degrees((math.atan2(ball.pos[1]-rob.pos[1],ball.pos[0]-rob.pos[0])))-rob.angle
                    error1 = ((ball.pos[0]-rob.pos[0])**2+(ball.pos[1]-rob.pos[1])**2)**0.5
                    kick = 1 #kick the ball
                else:
                    error2=math.degrees((math.atan2(samples_new[round(len(samples_new)/10) + 1][1]-rob.pos[1], samples_new[round(len(samples_new)/10) + 1][0]-rob.pos[0])))-rob.angle #error2 for Mike's Added Value
                    error1 = ((next_point_x-rob.pos[0])**2+(next_point_y-rob.pos[1])**2)**0.5 #error1 for Mike's Added value

                #error2=math.degrees((math.atan2(ball.pos[1]-rob.pos[1],ball.pos[0]-rob.pos[0])))-rob.angle
                #error2=math.degrees((math.atan2(samples_new[round(len(samples_new)/10) + 1][1]-rob.pos[1], samples_new[round(len(samples_new)/10) + 1][0]-rob.pos[0])))-rob.angle #error2 for Mike's Added Value
                
                #Mike's Added Value Part 2 START
                color = 255
                cv2.line(img, (ball.pos[0],ball.pos[1]), (points[3][0],points[3][1]), color, 5)

                nx, ny = (500,1) #500 colomns by 1 rows vector]
                x = np.linspace(ball.pos[0],points[3][0],nx) #x vector
                y = np.linspace(ball.pos[1],points[3][1],nx) #y vector, start and end reversed

                xpoint = x[round(len(x)*8/10)] #Going to rounded 8/10ths the way through the x vector
                ypoint = y[round(len(y)*8/10)] #Going to rounded 8/10ths the way through the y vector
                error1 = ((xpoint-rob.pos[0])**2+(ypoint-rob.pos[1])**2)**0.5 #error1 for Mike's Added value part 2
                error2 = math.degrees((math.atan2(ypoint-rob.pos[1],xpoint-rob.pos[0])))-rob.angle
                #print(xpoint)
                #print(ypoint)

                #Mike's Added Value Part 2 END 

                #regulate the angle to reduce ambiguity
                if (abs(error2)<180):
                        error2=error2
                elif (np.sign(error2)==-1):
                        error2=error2+360
                elif (np.sign(error2)==1):
                        error2=error2-360
                else:
                        print("done")
                print("error2",error2)

                derivative2=(error2-error_prior2) #Shouldn't this be divided by a dt?
                error_prior2=error2
                u2=  (kp2*error2)   +   (kd2*derivative2) 
                print("u2",u2)
               
			    #Error In Position
                ####### Position Control #######
                temp1=ball.pos[0]
                temp2=ball.pos[1]
                #error1 = ((ball.pos[0]-rob.pos[0])**2+(ball.pos[1]-rob.pos[1])**2)**0.5
                #error1 = ((next_point_x-rob.pos[0])**2+(next_point_y-rob.pos[1])**2)**0.5 #error1 for Mike's Added value
                
                derivative1=(error1-error_prior1)
                error_prior1=error1
                print("error1:",error1)
                u1=  ( kp1*error1 )   +  ( kd1*derivative1 )


                # Assigning the direction of motors based on the wheel velocities sign
                if (error2>=-15 and error2<=15):
                    error2=error2
                    dirR=0
                    dirL=0
                else:	
                    if np.sign(error2)==-1:
                        dirR=0
                        dirL=1
                    elif np.sign(error2)==1:
                        dirR=1
                        dirL=0
                
             
                ## Setting limits to the inputs 
                if(u1 > umax):
                    u1=umax
                if(u1 < -umax):
                    u1 = -umax
                #u2=0
                if(u2 > u2max):
                    u2=u2max
                if(u2 < -u2max):
                    u2 = -u2max
                            
                # Assigning Individual Wheel velocities
                vr=u1+u2
                vl=u1-u2
                
                # Remove the sign in motor velocities
                Vr = abs(int(vr))
                Vl = abs(int(vl))

                # Assign the motor velocities to 0-256 range to send through 8bit UART
                #VrHex = int((Vr - VrMin)*255/ (VrMax - VrMin))
                #VlHex = int((Vl - VlMin)*255/ (VlMax - VlMin))
                VrHex = int(Vr*255/ VrMax)+0x20
                VlHex = int(Vl*255/ VlMax)+0x20

                if (abs(error1) < 10 and abs(error2) <5): 
                    kick= 0x01
                else:
                    kick = 0
                if (error1<50):
                    VrHex=0x0
                    VlHex=0x0
               
                print("VlHex:",VlHex)
                print("VrHex:",VrHex)
                counter = counter + 1
                #packet.append(0xFF)
                #packet.append(0x01)  #Robot ID
                #packet.append(VrHex) #VrHex
                #packet.append(dirR)  #dirR
                #packet.append(VlHex) #VlHex
                #packet.append(dirL)  #dirL
                #packet.append(kick)  #kick
                #packet.append(0xFF)
                #KL25.write(packet)
                #data = KL25.read(4)
                #print(data.decode('ISO-8859-1'))
                

    cv2.imshow('circles on stream',img)


    

def run():
    app = QApplication(sys.argv)
    window = roboGUI()
    window.show()
    sys.exit(app.exec_())
 
breakpointthing = None
class Worker(QObject):
    finished = pyqtSignal()

    def __init__(self):
        super(Worker, self).__init__()
        self.working = True

    def work(self):
        while(self.working):
            sys.settrace = breakpointthing
            mainLoop()
        stoprobot('all') # when stop button pressed, stop robots

class roboGUI(QMainWindow):
    def __init__(self, *args, **kwargs):
        super(roboGUI,self).__init__()
        self.window = QWidget(self)
        self.setCentralWidget(self.window)
        self.resize(300,100)
        self.setWindowTitle("roboGUI")
        self.layout = QVBoxLayout()

        self.startButton = QPushButton("START",self)
        self.startButton.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding)
        self.startButton.resize(self.startButton.minimumSizeHint())
        self.layout.addWidget(self.startButton)
        self.stopButton = QPushButton("STOP",self)
        self.stopButton.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding)
        self.stopButton.resize(self.stopButton.minimumSizeHint())
        self.layout.addWidget(self.stopButton)

        self.param1text = QHBoxLayout()
        self.param1title = QLabel("Parameter 1")
        self.param1value = QLabel(str(param1val))
        self.param1slider = QSlider(Qt.Horizontal)
        self.param1slider.setMinimum(1)
        self.param1slider.setMaximum(250)
        self.param1slider.setValue(param1val)
        self.param1slider.setTickInterval(1)
        self.param1text.addWidget(self.param1title)
        self.param1text.addWidget(self.param1value)
        self.layout.addLayout(self.param1text)
        self.layout.addWidget(self.param1slider)
        
        self.param2text = QHBoxLayout()
        self.param2title = QLabel("Parameter 2")
        self.param2value = QLabel(str(param2val))
        self.param2slider = QSlider(Qt.Horizontal)
        self.param2slider.setMinimum(1)
        self.param2slider.setMaximum(50)
        self.param2slider.setValue(param2val)
        self.param2slider.setTickInterval(1)
        self.param2text.addWidget(self.param2title)
        self.param2text.addWidget(self.param2value)
        self.layout.addLayout(self.param2text)
        self.layout.addWidget(self.param2slider)

        self.valuetext = QHBoxLayout()
        self.valuetitle = QLabel("Minimum Value for Color")
        self.valuevalue = QLabel(str(valueMin))
        self.valueslider = QSlider(Qt.Horizontal)
        self.valueslider.setMinimum(0)
        self.valueslider.setMaximum(255)
        self.valueslider.setValue(valueMin)
        self.valueslider.setTickInterval(1)
        self.valuetext.addWidget(self.valuetitle)
        self.valuetext.addWidget(self.valuevalue)
        self.layout.addLayout(self.valuetext)
        self.layout.addWidget(self.valueslider)

        self.window.setLayout(self.layout)
        self.window.setSizePolicy(QSizePolicy.Preferred,QSizePolicy.Expanding)
        
        self.thread = None
        self.worker = None

        self.param1slider.valueChanged[int].connect(self.changedValue_param1)
        self.param2slider.valueChanged[int].connect(self.changedValue_param2)
        self.valueslider.valueChanged[int].connect(self.changedValue_valueMin)
        self.startButton.clicked.connect(self.startLoop)

    def startLoop(self):
        breakpointthing = sys.gettrace()
        self.thread = QThread()
        self.worker = Worker()
        self.worker.moveToThread(self.thread)

        self.thread.started.connect(self.worker.work)
        self.stopButton.clicked.connect(self.stopLoop)
        self.worker.finished.connect(self.thread.quit)
        self.worker.finished.connect(self.worker.deleteLater)
        self.worker.finished.connect(self.thread.deleteLater)

        self.thread.start()

    def stopLoop(self):
        self.worker.working = False

    def changedValue_param1(self, value):
        global param1val
        param1val = value
        self.param1value.setText(str(param1val))
    def changedValue_param2(self, value):
        global param2val
        param2val = value
        self.param2value.setText(str(param2val))
    def changedValue_valueMin(self, value):
        global valueMin
        valueMin = value
        self.valuevalue.setText(str(valueMin))

    sys._excepthook = sys.excepthook 
    def exception_hook(exctype, value, traceback):
      #  print(exctype, value, traceback)
        sys._excepthook(exctype, value, traceback) 
        sys.exit(1) 
    sys.excepthook = exception_hook 

if __name__== "__main__":
    run()
    cv2.destroyAllWindows()
