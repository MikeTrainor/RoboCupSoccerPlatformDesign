import cv2 
import numpy as np
from scipy.spatial import distance as dist
import argparse
import imutils
import copy

class robotClass:
    def __init__(self, pos = [], team = '-no team!-', ID = '-no ID!-'):
        # centre coordinates
        self.pos = pos        
        self.team = team
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
    if (blue > 220 and green < 50 and red < 50):
        color = 'B' # Blue team circle
    elif (blue < 50 and green > 200 and red > 200):
        color = 'Y' # Yellow team circle
    elif (blue > 200 and green < 50 and red > 200):
        color = 'P' # Purple ID circle
    elif (blue < 50 and green > 220 and red < 50):
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
    x=int(circle[0])
    y=int(circle[1])

    blue = img[y,x,0]
    green = img[y,x,1]
    red = img[y,x,2]
    color = colorID(blue, green, red)
    print('Circle is ', color)

    roboList.append(robotClass())

    # if its blue or (if its yellow) --> Robot center/new robot
    if (color == 'B') or (color == 'Y'):
        roboList.append(robotClass([x,y],color))

    # if green or purple --> Robot ID marking
    elif (color == 'G') or (color == 'P'):        
        roboIDmarks.append([x,y,color])
            
    # if orange --> Ball location
    elif (color == 'O'):
        ball = ballClass([x,y])

def assignIDmarks():
    if isinstance(roboList, type(None)):
        # Assign each robot its four closest marks
        for robot in roboList:
            closestMarks = [0,0,0,0] # indices of the closest four marks to the robot center
            furthestMark = [0,0] # [index, euclidean distance]

            # Assign this robot its four closest marks
            for i, mark in enumerate(list(roboIDmarks)):
                markDist = dist.euclidean(mark,robot.pos)

                # If there aren't already four marks given to the robot, 
                # just give it whatever is available in order to initialize robot.circles[]
                if len(robot.circles) < 4:
                    robot.newMarking(mark)
                    closestMarks[i] = i

                # If there is a closer value than the furthest currently in robot.circles[]
                # replace the current furthest with this new one 
                elif markDist < furthestMark[1]:
                    robot.circles[furthestMark[0]] = mark   # might run into issues altering values in the "robot" object,
                                                            # as it is the subject of the for loop
                    closestMarks[furthestMark[0]] = i
                
                # Update the furthest mark- done after the above code so no weird stuff happens
                if markDist > furthestMark:
                    furthestMark = [i, markDist]

            # Remove the marks that were assigned to a robot- this will potentially make 
            # assigning the rest of the marks much quicker with larger numbers of robots
            for idx in closestMarks:
                del roboIDmarks[idx]
    else:
        print("No robots detected, but there are ID marks..?")

def main():
    #for i in range(10):
    #    new_robot = robot('B',i+2,1*i)
    #    roboList.append(new_robot)
    #    print(roboList[i].team,roboList[i].ID,roboList[i].x,roboList[i].y)

    #cball = ball([3,4])
    #print(cball.pos)

    #First part: Recognizing circles in sample image

    imgpath = "T:\\Documents\\University\\ECE 4553\\Project\\motherfucker.png"
    img = cv2.imread(imgpath)
    cv2.imshow("circles on image", img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # smooth it, otherwise a lot of false circles may be detected
    blurred = cv2.GaussianBlur(img, (9,9), 0)
    blurred2 = cv2.GaussianBlur(gray, (9,9), 0)
    cv2.imshow("I wonder what blurred looks like...", blurred)
    cv2.imshow("I wonder what blurred looks like...2", blurred2)
    # Some notes on the HoughCircles function:
    #  - by adjusting param2, we can alter the threshold for the circles recognized
    #  - adjusting param1 does not seem to have an effect on the result as of now
    #  - the "1" scales the image for the function, "2" would scale down by 2
    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,minDist = 20,
                               param1=50,param2=14,minRadius=15,maxRadius=50)
    cv2.waitKey()

    if isinstance(circles, type(None)) == 0:
        for circle in circles[0,:]:
            IDcircle(img, circle)

            # draw the outer circle
            cv2.circle(img,(circle[0],circle[1]),circle[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(img,(circle[0],circle[1]),2,(0,0,255),3)
            cv2.imshow("circles on image", img)
            cv2.waitKey()
        
        # Assign the ID marks observed to their appropriate robot
        if isinstance(roboIDmarks, type(None)) == 0:
            assignIDmarks()

    
    cv2.waitKey()
    cv2.destroyAllWindows()

    #Second part: recognizing circles in stream of images

    #cap = cv2.VideoCapture(0)
    #while True:
    #    ret, frame = cap.read()
    #    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #    # smooth it, otherwise a lot of false circles may be detected
    #    # ^This is likely why so many crazy circles happened- not enough
    #    # blurring of the image being received by the camera
    #    blurred = cv2.GaussianBlur(gray, (9,9), 0)
    #    circles = cv2.HoughCircles(gray,cv2.HOUGH_GRADIENT,1,20,
    #                               param1=50,param2=30,minRadius=5,maxRadius=50)

    #    # so we can have streams of both original and circled version
    #    cstream = copy.deepcopy(frame)

    #    # This if statement prevents crashing when there are no circles recognized
    #    if isinstance(circles, type(None)) == 0:
    #        for circle in circles[0,:]:
    #            # circle(image,(x,y),radius,color,thickness?)
    #            # draw the outer circle
    #            cv2.circle(cstream,(circle[0],circle[1]),circle[2],(0,255,0),2)
    #            # draw the center of the circle
    #            cv2.circle(cstream,(circle[0],circle[1]),2,(0,0,255),3)

    #    cv2.imshow("circles on stream", cstream)
    #    cv2.imshow("original stream", frame)

    #    if cv2.waitKey(1) & 0xFF == ord('\r'):
    #        #cv2.destroyAllWindows()
    #        break
 
 
if __name__== "__main__":
    main()
