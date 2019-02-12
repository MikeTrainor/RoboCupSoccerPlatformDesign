import numpy as np
from scipy.spatial import distance as dist
from scipy import interpolate
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import argparse
import copy
import math
import os
from PIL import Image
import glob
import cv2
from fractions import gcd

class robotClass:
    def __init__(self, pos = [], team = '-no team!-', ID = '-no ID!-'):
        # centre coordinates
        self.pos = pos        
        self.team = team
        self.angle = 0
        self.ID = ID
        # ID markings
        self.circles = [] # [x,y,color]

    def newMarking(self, circle = [0,0,[0,0,0]]):
        self.circles.append(circle)

class ballClass:
    def __init__(self, pos = []):
        self.pos = pos

roboList = []
roboIDmarks = []
ball = 0
IDdRobots = []
#################################################################################################################################################################################################
points = []
tangents = []
resolution = 0.2
#################################################################################################################################################################################################
#################################################################################################################################################################################################
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

#################################################################################################################################################################################################

# closestBot()
# x, Nov. yth, 2018
# This function identifies the nearest robot and returns the index of the robot within
# the list of robots (roboList) which the marking belongs to.
#              (input) -> (function) -> (output)
#               [x,y] -> closestBot() -> index
# v1:
# Unfinished. Needs distance calculations corrected, maybe a threshold for how close it
# needs to be...
def closestBot(circle):
    closestDist = 99999

    # Use Euclidean distance and cycle through roboList, comparing each of the
    # "pos" variables to the circle's [x,y] passed into the function
    for idx, pos in enumerate(roboList):
        newDist = dist.euclidean([circle[1],circle[2]],pos)
        if newDist < closestDist:
            closestDist = newDist
            closestIdx = idx

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
def colorID(blue, green, red):
    color = 'X' # Default case, 'X' will print an error for an unrecognized element
    if (blue > 125 and green < 150 and red < 150):
        color = 'B' # Blue team circle
    elif (blue < 150 and green > 200 and red > 200):
        color = 'Y' # Yellow team circle
    elif (blue > 75 and green < 150 and red > 150):
        color = 'P' # Purple ID circle
    elif (blue < 200 and green > 150 and red < 150):
        color = 'G' # Green ID circle
    elif (blue < 50 and green < 200 and red > 220):
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

    blue = img[y,x,0]
    green = img[y,x,1]
    red = img[y,x,2]
    color = colorID(blue, green, red)
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
        ball = ballClass([x,y])

def assignIDmarks():
    if isinstance(roboList, type(None)) == 0:
        # Assign each robot its four closest marks
        for robot in roboList:
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
    else:
        print("No robots detected, but there are ID marks..?")

def RoboID():
    for robot in roboList:
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


    #for robot in roboList:
    #    robot.ID = 'Unidentified!'
    #    pink = 0
    #    green = 0
    #    for mark in robot.circles:
    #        if mark[2] == 'P':
    #            pink += 1
    #        elif mark[2] == 'G':
    #            green += 1
    #    if pink == 4:
    #        robot.ID = 'Pink'
    #        IDdRobots[index] = 3
    #    elif green == 4:
    #        robot.ID = 'Green'
    #        IDdRobots[index] = 1
    #    elif (pink == 2) & (green == 2):
    #        robot.ID = 'Green and Pink'
    #        IDdRobots[index] = 2

    #    break ############################### This should be deleted in an actual implementation, but for ##################
    #                      ################### the purpose of testing, it has been included to prevent weird stuff ##########

    #return

#topIDs = []
#bottomIDs = []
def angle():	
    #angles = []	
    #global topIDs
    #global bottomIDs
    for robot in roboList:
        topIDs = []
        bottomIDs = []
        theta1 = 0
        theta2 = 0
        #thing = range(len(robot.circles)-1)
        for ii in range(len(robot.circles)-1):
            #thing2=range(ii + 1, len(robot.circles))
            for jj in range(ii + 1, len(robot.circles)):
                temp1 = robot.circles[ii]
                temp2 = robot.circles[jj]

                # Determining distance between the different IDs
                a = dist.euclidean([temp1[0],temp1[1]],robot.pos) # Distance from ID 1 to centre
                b = dist.euclidean([temp2[0],temp2[1]],robot.pos) # Distance from ID 2 to centre
                c = dist.euclidean([temp1[0],temp1[1]],[temp2[0],temp2[1]]) # Distance from ID 1 to ID 2

                theta = math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

                if theta > 100 and theta < 130: # Ideally 114.84 degrees
                    #print("topdots")
                    topIDs.append(temp1)
                    topIDs.append(temp2)
                    #print(topIDs)
                    #print("w/ centre of ",robot.pos)
                    #print("and angle of ",theta)
                     
                if theta > 45 and theta < 75: # Ideally 65.16 degrees
                    #print("bottomdots")
                    bottomIDs.append(temp1)
                    bottomIDs.append(temp2)
                    #print(bottomIDs)
                    #print(theta)

                # the other ID pairs will be either ~180 or ~90 degrees...
        #print("doneloop")
        if len(topIDs) == 2:
            xMean = (topIDs[0][0] + topIDs[1][0])/2
            yMean = (topIDs[0][1] + topIDs[1][1])/2
            

            xDiff = xMean - robot.pos[0]
            yDiff = yMean - robot.pos[1]
            # Angle points in the direction the robot is facing
            theta1 = math.degrees(math.atan2(yDiff,xDiff))
        else:
            print("top went wrong...")

        if len(bottomIDs) == 2:
            xMean2 = (bottomIDs[0][0] + bottomIDs[1][0])/2
            yMean2 = (bottomIDs[0][1] + bottomIDs[1][1])/2

            xDiff2 = robot.pos[0] - xMean2
            yDiff2 = robot.pos[1] - yMean2
            # Negative for both of these to get an angle that is front facing
            theta2 = math.degrees(math.atan2(yDiff2,xDiff2))
        else:
            print("bottom is wrong")

        if theta2 != 0 and theta1 != 0:
            xMean = (math.cos(math.radians(theta1)) + math.cos(math.radians(theta2)))/2
            yMean = (math.sin(math.radians(theta1)) + math.sin(math.radians(theta2)))/2
            theta = math.degrees(math.atan2(yMean,xMean))
            robot.angle = theta
        elif theta2 != 0 and theta1 == 0:
            theta = theta2
            robot.angle = theta
        elif theta2 == 0 and theta1 != 0:
            theta = theta1
            robot.angle = theta
        else:
            return "ERROR"

        #angles.append(theta)

    #def reassignIDs():
    #    # Assigning individual IDs
        if len(robot.circles) == 4 and len(topIDs) == 2 and len(bottomIDs) == 2:
            if theta <= 45 and theta >= -45:
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
            if theta < 135 and theta > 45:
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
            if theta < -45 and theta > -135:
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
            if theta <= -135 or theta >= 135:          # must be "or", as sign swaps at 180
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

    #print("robot circles: ",robot.circles)
    #print("with an angle of: ",robot.angle)
    #print("done this robots circles")



def main():
    ##First part: Recognizing circles in sample image

    #filename = 'T:\\Documents\\dataPatRec\\'
    #title = input('Name of files?')
    #filename = filename + title
    #images = []
    #for ii in range(1,31):
    #    im=cv2.imread(filename+str(ii)+'.jpg')
    #    images.append(im)

    #truthlist = [1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3]

    #            cv2.circle(img,(mark[0],mark[1]),10,(0,0,0),5)
    #        cv2.imshow("robot by robot circles", img)
    #        cv2.waitKey()
    
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    global roboList
    global roboIDmarks
    global circles 
    global ball
    global IDdRobots
    global points
    global tangents

    #Second part: recognizing circles in stream of images

    cap = cv2.VideoCapture(0)
    cap1 = cv2.VideoCapture(0)

    #################################################################################################################################################################################################
    # https://math.stackexchange.com/questions/2583779/which-spline-interpolation-method-can-incorporate-slope-information-at-the-suppo?rq=1
    # Input.
    points = []
    tangents = []
    resolution = 0.2

    #For tangents.append the slope is y/x
    #points.append([0.,0.]); tangents.append([1,1])
    #points.append([3.,4.]); tangents.append([1,0])
    #points.append([5.,2.]); tangents.append([0,-1])
    #points.append([3.,0.]); tangents.append([-1,-1])
    #points.append([200,300]); tangents.append([math.tan(45*np.pi/180),1]) #Robot position, Slope converted from radians, this value is whatever angle the robot is currently facing
    #points.append([500,400]); tangents.append([1,-1]) #Ball position
    #points.append([600,300]); tangents.append([1,1]) #Net position


    points.append([400,400]) #Robot Position
    points.append([300,200]) #Ball Position
    points.append([600,220]) #Net Position

    #Finding the angle at which the robot approaches
    approach_x = (points[2][0] - points[1][0])
    approach_y = (points[2][1] - points[1][1])
    common_divisor = abs(gcd(approach_x,approach_y)) #Absolute value of the greatest common divisor
    approach_x = approach_x/common_divisor #Divide by common divisor
    approach_y = approach_y/common_divisor #Divide by common divisor

    tangents.append([math.tan(45*np.pi/180),1]) #Robot position, Slope converted from radians, this value is whatever angle the robot is currently facing
    tangents.append([approach_x, approach_y]) #Ball position
    tangents.append([approach_x, approach_y]) #Net position

    points = np.asarray(points)
    tangents = np.asarray(tangents)

    # Interpolate with different tangent lengths, but equal direction.
    scale = 1.
    tangents1 = np.dot(tangents, scale*np.eye(2))
    samples1 = sampleCubicSplinesWithDerivative(points, tangents1, resolution)
    scale = 2.
    tangents2 = np.dot(tangents, scale*np.eye(2))
    samples2 = sampleCubicSplinesWithDerivative(points, tangents2, resolution)
    scale = 0.1
    tangents3 = np.dot(tangents, scale*np.eye(2))
    samples3 = sampleCubicSplinesWithDerivative(points, tangents3, resolution)
    #################################################################################################################################################################################################

    while(True):
        ret,frame = cap.read()
        
        roboList = []
        roboIDmarks= []
        circles = []
        ball = []

        #img_yuv = cv2.cvtColor(ii, cv2.COLOR_BGR2YUV)

        ### equalize the histogram of the Y channel
        #img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

        ### convert the YUV image back to RGB format
        #frame_yuv = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

        hsv= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        lower_rangeG = np.array([0,100,100])
        upper_rangeG = np.array([255,255,255])
        #lower_rangeBR = np.array([85,50,50])
        #upper_rangeBR = np.array([190,255,255])

        mask1 = cv2.inRange(hsv, lower_rangeG, upper_rangeG)
        #mask2= cv2.inRange(hsv, lower_rangeG, upper_rangeG)
        result1 = cv2.bitwise_and(frame,frame,mask=mask1)
        cv2.imshow("masked image",result1)
        # result2 = cv2.bitwise_and(frame,frame,mask=mask2)
        #FinalResult = cv2.bitwise_or(result1,result2)

  
        #hsv_out = np.bitwise_or(FinalResult, edged[:,:,np.newaxis])
        hsv_out_gray= cv2.cvtColor(result1, cv2.COLOR_BGR2GRAY)
        cv2.imshow("masked image2",hsv_out_gray)
       # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(hsv_out_gray,cv2.HOUGH_GRADIENT,1,20,param1=20,param2=13,minRadius=20,maxRadius=50)
        #cstream = copy.deepcopy(result1)
        #cv2.imshow("onetwo",cstream)
        cv2.waitKey(1)
        #cv2.imshow("filtered",result1)
        # Some notes on the HoughCircles function:
        #  - by adjusting param2, we can alter the threshold for the circles recognized
        #  - adjusting param1 does not seem to have an effect on the result as of now
        #  - the "1" scales the image for the function, "2" would scale down by 2
        # circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,minDist = 20,
        #                           param1=50,param2=40,minRadius=15,maxRadius=100)

        img = copy.deepcopy(frame)

        if isinstance(circles, type(None)) == 0:
            for circle in circles[0,:]:
                IDcircle(img, circle)

                # draw the outer circle
                cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)
     
                #cv2.imshow("circles on image", img)
                #cv2.waitKey()
            #cv2.imshow("masked", result1)
            # Assign the ID marks observed to their appropriate robot
            if isinstance(roboIDmarks, type(None)) == 0:
                assignIDmarks()
                angle()
                RoboID()

            
            #if isinstance(ball, type(None)) == 0:
            #    print('Ball found at ',ball)

            # Mark the robot circles seen robot by robot
            if isinstance(roboList, type(None)) == 0:
                for robot in roboList:
                    #reassignIDs()
                    

                    cv2.circle(img,(robot.pos[0],robot.pos[1]),10,(0,0,0),5)
                    if isinstance(robot.angle, type(None)) == 0:
                        cv2.putText(img, str(round(robot.angle,1)), (robot.pos[0]+ 100, robot.pos[1] + 130), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                        cv2.putText(img, str(robot.pos), (robot.pos[0]+ 100, robot.pos[1] + 100), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                        cv2.putText(img, robot.ID, (robot.pos[0]+ 100, robot.pos[1] + 70), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 5)
                    for mark in robot.circles:
                        cv2.circle(img,(mark[0],mark[1]),10,(0,0,0),5)

                    #print('There is a '+robot.team+' robot with these ID circles:\n')
                    for circle in robot.circles:
                        print(circle)
                    #robotIDstring = '... which indicates it is a '+robot.ID+' robot'
                    #print(robotIDstring)
                #cv2.imshow("original stream", frame)

                    points[0][0] = robot.pos[0]; #Assign the robot x position
                    points[0][1] = robot.pos[1]; #Assign the robot y position
        else: 
            print("no circles detected")
        
            
        ############################################################################################################################################################################################################
        #if isinstance(points, type(None)) == 0:
        
        #https://stackoverflow.com/questions/44598124/update-frame-in-matplotlib-with-live-camera-preview
        #Integrate this code with the robot's position which is x = robot.pos[0] and y = robot.pos[1]


        # Interpolate with different tangent lengths, but equal direction.
        scale = 0.1
        tangents3 = np.dot(tangents, scale*np.eye(2))
        samples3 = np.float32(sampleCubicSplinesWithDerivative(points, tangents3, resolution))
            

        #Finding the angle at which the robot approaches
        #points[0][0] = points[0][0] - 2
        #points[0][1] = points[0][1] - 2
        approach_x = (points[2][0] - points[1][0])
        approach_y = (points[2][1] - points[1][1])
        common_divisor = abs(gcd(approach_x,approach_y)) #Absolute value of the greatest common divisor
        approach_x = approach_x/common_divisor #Divide by common divisor
        approach_y = approach_y/common_divisor #Divide by common divisor

        #Find the slope to the next point
        next_point_x =  np.float32(samples3[round(len(samples3)/10)][0] - points[0][0]) #Change in x between the robot and next point
        next_point_y =  np.float32(samples3[round(len(samples3)/10)][1] - points[0][1]) #Change in y between the robot and next point
        tangents[0][0] = math.atan2(next_point_y,next_point_x) #This value is the angle in radians that the robot must face for its path
        tangents[0][1] = 1
        tangents[1][0] = approach_x #x slope
        tangents[1][1] = approach_y #y slope
        tangents[2][0] = approach_x #x slope
        tangents[2][1] = approach_y #y slope

        #if (samples3[:][0] >= 640) or (samples3[:][0] <= 0) or (samples3[:][1] >= 420) or (samples3[:][1] <= 0):
                


        points = np.asarray(points)
        tangents = np.asarray(tangents)

        trajectory = math.degrees(tangents[0][0]) #Angle in degrees
        print(trajectory)

        #Display splines on the live feed & plot
        path_plot = plt.scatter(samples3[:,0], samples3[:,1], marker='o', label='samples3')
        k = 0
        for k in range(0,len(samples3)):
            cv2.circle(img, (samples3[k,0], samples3[k,1]), 1, (0, 255, 255),5)

        #image1 = path_plot.imshow(grab_frame(cap))
        plt.ion()
        plt.show()
 
        ############################################################################################################################################################################################################
        cv2.imshow('circles on stream',img)
        cv2.imshow('original stream',frame)

        cv2.waitKey(250)
        if cv2.waitKey(1) & 0xFF == ord('\r'):
            break
    
    cv2.destroyAllWindows(0)

        #if cv2.waitKey(1) & 0xFF == ord('\r'):
        #    #cv2.destroyAllWindows()
        #    break
 
 
if __name__== "__main__":
    main()



            #if isinstance(roboList, type(None)) == 0:
        #    print('ROBOT FOUND!!!!!!!!!!!')
        #    for robot in roboList:
        #        print('There is a ',robot.team,' robot with these ID circles:\n')
        #        for circle in robot.circles:
        #            print(circle[0],circle[1],circle[3])

        #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ## smooth it, otherwise a lot of false circles may be detected
        ## ^This is likely why so many crazy circles happened- not enough
        ## blurring of the image being received by the camera
        #blurred = cv2.GaussianBlur(gray, (9,9), 0)
        #circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
        #                           param1=50,param2=30,minRadius=5,maxRadius=50)

        ## so we can have streams of both original and circled version
        #cstream = copy.deepcopy(frame)

        ## This if statement prevents crashing when there are no circles recognized
        #if isinstance(circles, type(None)) == 0:
        #    for circle in circles[0,:]:
        #        # circle(image,(x,y),radius,color,thickness?)
        #        # draw the outer circle
        #        cv2.circle(cstream,(circle[0],circle[1]),circle[2],(0,255,0),2)
        #        # draw the center of the circle
        #        cv2.circle(cstream,(circle[0],circle[1]),2,(0,0,255),3)

        #cv2.imshow("circles on stream", cstream)
        #cv2.imshow("original stream", frame)


#def main():
#    # Old top facing one
#    #roboList.append(robotClass([400,400],'B'))
#    #roboList[0].newMarking([121,720,[0,0,0]])
#    #roboList[0].newMarking([678,720,[0,0,0]])
#    #roboList[0].newMarking([181,35.7,[0,0,0]])
#    #roboList[0].newMarking([619,35.7,[0,0,0]])
#    # Old right facing one
#    #roboList.append(robotClass([759,585],'B'))
#    #roboList[0].newMarking([394,802,[0,0,0]])
#    #roboList[0].newMarking([392,367,[0,0,0]])
#    #roboList[0].newMarking([1079,862,[0,0,0]])
#    #roboList[0].newMarking([1078,304,[0,0,0]])

#    #working one with 180 degrees ** now -90 degrees
#    roboList.append(robotClass([168.5,107.7],'B'))
#    roboList[0].newMarking([60.8,276.2,[0,0,0]])
#    roboList[0].newMarking([275.8,276.2,[0,0,0]])
#    roboList[0].newMarking([0,0,[0,0,0]])
#    roboList[0].newMarking([337,0,[0,0,0]])

#    #should be working with -90 degrees ** now +/-180 degrees
#    roboList.append(robotClass([213,256],'B'))
#    roboList[1].newMarking([110,85,[0,0,0]])
#    roboList[1].newMarking([110,421,[0,0,0]])
#    roboList[1].newMarking([384,148,[0,0,0]])
#    roboList[1].newMarking([387,363,[0,0,0]])

#    #should be working with 0 degrees ** now 90 degrees
#    roboList.append(robotClass([257,261],'B'))
#    roboList[2].newMarking([148,88,[0,0,0]])
#    roboList[2].newMarking([365,85,[0,0,0]])
#    roboList[2].newMarking([87,363,[0,0,0]])
#    roboList[2].newMarking([423,364,[0,0,0]])

#    #should be working with 90 degrees ** now 0 degrees
#    roboList.append(robotClass([289,250],'B'))
#    roboList[3].newMarking([113,142,[0,0,0]])
#    roboList[3].newMarking([391,84,[0,0,0]])
#    roboList[3].newMarking([117,360,[0,0,0]])
#    roboList[3].newMarking([393,421,[0,0,0]])

#    angles = angle()
#    print("This is the angle of the robot->",angles)



#if __name__== "__main__":
#    main()