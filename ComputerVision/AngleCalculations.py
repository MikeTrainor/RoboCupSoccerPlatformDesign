import numpy as np
from scipy.spatial import distance as dist
import argparse
import copy
import math
import os
from PIL import Image
import glob
import cv2

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
    def __init__(self, pos = []):
        self.pos = pos

roboList = []       # holds all robots currently seen- resets every loop
roboIDmarks = []    # holds all potential robot ID marks seen ('G' or 'P')
ball = 0            # holds ball position in ballClass type object
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

# assignIDmarks()
# E.H., Dec, 2018
# This function cycles through the globally stored roboIDmarks list and assigns them to the
# closest available robot, provided they don't already have 4 assigned
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

# RoboID()
# E.H., Jan, 2019
# This function reads the sorted robot.circles list and assigns an ID (robot.ID = x)
# to the robot. If the IDs are not properly sorted, this will not work
# v1:
# Doesn't have all IDs implemented yet
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
    # Declaring global variables so they can be cleared every loop
    global roboList
    global roboIDmarks
    global circles 
    global ball
    global IDdRobots

    cap = cv2.VideoCapture(0) # 0 if your pc doesn't have a webcam, probably 1 if it does

    while(True):
        ret,frame = cap.read() # reading the video capture into a dummy var and frame
        
        # Reinitializing robot data (prevents buildup of data accross frames)
        roboList = []
        roboIDmarks= []
        circles = []
        ball = []

        # Histogram equalization for colors (haven't tested with this)
        #img_yuv = cv2.cvtColor(ii, cv2.COLOR_BGR2YUV)

        ### equalize the histogram of the Y channel
        #img_yuv[:,:,0] = cv2.equalizeHist(img_yuv[:,:,0])

        ### convert the YUV image back to RGB format
        #frame_yuv = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

        # HSV color masking
        hsv= cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

        lower_rangeG = np.array([0,100,100])
        upper_rangeG = np.array([255,255,255])

        mask = cv2.inRange(hsv, lower_rangeG, upper_rangeG) # mask for original frame with only good color
        result = cv2.bitwise_and(frame,frame,mask=mask)
        cv2.imshow("masked image",result)
  
        hsv_out_gray= cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        # consider implementing gaussian blur here

        # Some notes on the HoughCircles function:
        #  Utilizes edge detection to draw tangent lines, recognizing a circle where perpendicular lines to tangents
        #  meet, depending on the intensity of the intersecting tangent lines.
        #  param1: higher threshold for Canny edge detection (lower is half of this)
        #  param2: accumulator threshold for circle center detection- i.e. the lower it is, the less circular an object
        #          needs to be to be recognized as a circle
        #  minDist: Specifies minimum distance between circles (the 4th input to the function)
        #  
        # from documentation: cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) → circles
        circles = cv2.HoughCircles(hsv_out_gray,cv2.HOUGH_GRADIENT,1,minDist = 20,param1=20,param2=13,minRadius=20,maxRadius=50)

        cv2.waitKey(1) # cv2.waitKey() is required to display images- waits 1 millisecond here

        img = copy.deepcopy(frame) # Sometimes if you copy stuff in Python, changes made to a copied variable end up in original
                                   # which necessitates a deepcopy

        if isinstance(circles, type(None)) == 0:
            for circle in circles[0,:]:
                IDcircle(img, circle) # ID all the circles recognized by color
                # draw the outer circle
                cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
                # draw the center of the circle
                cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)

            
            if isinstance(roboIDmarks, type(None)) == 0:
                assignIDmarks() # Assign the ID marks observed to their appropriate robot
                angle()         # Determine angle of robots seen
                RoboID()        # Give robots seen an ID

            
            #if isinstance(ball, type(None)) == 0:
            #    print('Ball found at ',ball)

            # Draw the robot circles seen robot by robot
            if isinstance(roboList, type(None)) == 0:
                for robot in roboList:
                    #reassignIDs() <- move this somewhere else- this isn't a good place for it

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

        else:
            print("no circles detected")

        # Display drawn on frame and original frame
        cv2.imshow('circles on stream',img)
        cv2.imshow('original stream',frame)
        cv2.waitKey(250)
        if cv2.waitKey(1) & 0xFF == ord('\r'): # if enter is pressed, stop running
            break


    cv2.destroyAllWindows(0)
 
 
if __name__== "__main__":
    main()




# OLD CODE: has some dummy robot positions that might be useful
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
