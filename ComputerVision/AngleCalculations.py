import cv2 
import numpy as np
from scipy.spatial import distance as dist
import argparse
import copy
import math

class robotClass:
    def __init__(self, pos = [], team = '-no team!-', ID = '-no ID!-'):
        # centre coordinates
        self.pos = pos        
        self.angle = 0 #<- Should be added I suppose ********************* this might mess some stuff up
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
ball = 0

def angle():	
    angles = []	
    for robot in roboList:
        topIDs = []
        bottomIDs = []
        theta1 = 0
        theta2 = 0
        thing = range(len(robot.circles)-1)
        for ii in range(len(robot.circles)-1):
            thing2=range(ii + 1, len(robot.circles))
            for jj in range(ii + 1, len(robot.circles)):
                temp1 = robot.circles[ii]
                temp2 = robot.circles[jj]

                # Determining distance between the different IDs
                a = dist.euclidean([temp1[0],temp1[1]],robot.pos) # Distance from ID 1 to centre
                b = dist.euclidean([temp2[0],temp2[1]],robot.pos) # Distance from ID 2 to centre
                c = dist.euclidean([temp1[0],temp1[1]],[temp2[0],temp2[1]]) # Distance from ID 1 to ID 2

                theta = math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

                if theta > 100 and theta < 130: # Ideally 114.84 degrees
                    print("topdots")
                    topIDs.append(temp1)
                    topIDs.append(temp2)
                    print(topIDs)
                     
                if theta > 45 and theta < 80: # Ideally 65.16 degrees
                    print("bottomdots")
                    bottomIDs.append(temp1)
                    bottomIDs.append(temp2)
                    print(bottomIDs)

                # the other ID pairs will be either ~180 or ~90 degrees...
        print("doneloop")
        if len(topIDs) == 2:
            xDiff = topIDs[0][0] - topIDs[1][0]
            yDiff = topIDs[0][1] - topIDs[1][1]
            theta1 = math.degrees(math.atan2(yDiff,xDiff))
        else:
            print("top went wrong...")

        if len(bottomIDs) == 2:
            xDiff = bottomIDs[0][0] - bottomIDs[1][0]
            yDiff = bottomIDs[0][1] - bottomIDs[1][1]
            theta2 = math.degrees(math.atan2(yDiff,xDiff))
        else:
            print("bottom is wrong")

        if theta2 != 0 and theta1 != 0:
            theta = (theta1 + theta2)/2
            robot.angle = theta
            angles.append(theta)
        elif theta2 != 0 and theta1 == 0:
            theta = theta2
            robot.angle = theta
            angles.append(theta)
        elif theta2 == 0 and theta1 != 0:
            theta = theta1
            robot.angle = theta
            angles.append(theta)
        else:
            return "ERROR"

        # Assigning individual IDs
        if theta <= 45 and theta >= -45:
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
        if theta < 135 and theta > 45:
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
        if theta < -45 and theta > -135:
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
        if theta <= -135 or theta >= 135:          # must be "or", as a comparison doesn't swap sign at 180
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

        print("robot circles: ",robot.circles)
        print("with an angle of: ",robot.angle)
        print("done this robots circles")

        ## assigning the circles
        #if topIDs[0][1] > bottomIDs[0][1] and topIDs[1][1] > bottomIDs[1][1] and topIDs[1][1] > bottomIDs[0][1] and topIDs[0][1] > bottomIDs[1][1]:
        #    # both top circles are above the bottom circles
        #    if topIDs[0][0] > topIDs[1][0]:
        #        robot.circles[0] = topIDs[0]
        #        robot.circles[1] = topIDs[1]
        #    elif topIDs[0][0] < topIDs[1][0]:
        #        robot.circles[0] = topIDs[1]
        #        robot.circles[1] = topIDs[0]
        #    if bottomIDs[0][0] > bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[0]
        #        robot.circles[1] = bottomIDs[1]
        #    elif bottomIDs[0][0] < bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[1]
        #        robot.circles[1] = bottomIDs[0]
        #elif topIDs[0][1] < bottomIDs[0][1] and topIDs[1][1] < bottomIDs[1][1] and topIDs[1][1] < bottomIDs[0][1] and topIDs[0][1] < bottomIDs[1][1]:
        #    # both top circles are below the bottom circles
        #    if topIDs[0][0] < topIDs[1][0]:
        #        robot.circles[0] = topIDs[0]
        #        robot.circles[1] = topIDs[1]
        #    elif topIDs[0][0] > topIDs[1][0]:
        #        robot.circles[0] = topIDs[1]
        #        robot.circles[1] = topIDs[0]
        #    if bottomIDs[0][0] < bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[0]
        #        robot.circles[1] = bottomIDs[1]
        #    elif bottomIDs[0][0] > bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[1]
        #        robot.circles[1] = bottomIDs[0]
        #elif topIDs[0][0] > bottomIDs[0][0] and topIDs[1][0] > bottomIDs[1][0] and topIDs[1][0] > bottomIDs[0][0] and topIDs[0][0] > bottomIDs[1][0]:
        #    # both top circles are more "right" than the bottom circles
        #    if topIDs[0][0] > topIDs[1][0]:
        #        robot.circles[0] = topIDs[0]
        #        robot.circles[1] = topIDs[1]
        #    elif topIDs[0][0] < topIDs[1][0]:
        #        robot.circles[0] = topIDs[1]
        #        robot.circles[1] = topIDs[0]
        #    if bottomIDs[0][0] > bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[0]
        #        robot.circles[1] = bottomIDs[1]
        #    elif bottomIDs[0][0] < bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[1]
        #        robot.circles[1] = bottomIDs[0]
        #elif topIDs[0][0] < bottomIDs[0][0] and topIDs[1][0] < bottomIDs[1][0] and topIDs[1][0] < bottomIDs[0][0] and topIDs[0][0] < bottomIDs[1][0]:
        #    # both top circles are more "left" than the bottom circles
        #    if topIDs[0][0] > topIDs[1][0]:
        #        robot.circles[0] = topIDs[0]
        #        robot.circles[1] = topIDs[1]
        #    elif topIDs[0][0] < topIDs[1][0]:
        #        robot.circles[0] = topIDs[1]
        #        robot.circles[1] = topIDs[0]
        #    if bottomIDs[0][0] > bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[0]
        #        robot.circles[1] = bottomIDs[1]
        #    elif bottomIDs[0][0] < bottomIDs[1][0]:
        #        robot.circles[0] = bottomIDs[1]
        #        robot.circles[1] = bottomIDs[0]

    #        angleToCentre = math.atan2(yToCentre,xToCentre)
    #        if angleToCentre < 0:
    #            angleToCentre = angleToCentre + 

    #        if(angleToCentre < 56 and angleToCentre > 45): # 56 deg -> 0.97738 rad and 45 deg -> 0.78540 rad
    #            topIDs.append(circle)

    #        if(angleToCentre >56 and angleToCentre < 65):
    #            bottomIDs.append(circle)

    #    if(len(topIDs) == 2):
    #        xDiff = topIDs[0][0] - topIDs[1][0]
    #        yDiff = topIDs[0][1] - topIDs[1][1]
    #        theta1 = math.atan2(yDiff,xDiff)

    #    if(len(bottomIDs) == 2):
    #        xDiff = bottomIDs[0][0] - topIDs[1][0]
    #        yDiff = bottomIDs[0][1] - topIDs[1][1]
    #        theta2 = math.atan2(yDiff,xDiff)

    #    theta = (theta1+theta2)/2
    #    angles.append(theta)
    #return angles



def main():
    # Old top facing one
    #roboList.append(robotClass([400,400],'B'))
    #roboList[0].newMarking([121,720,[0,0,0]])
    #roboList[0].newMarking([678,720,[0,0,0]])
    #roboList[0].newMarking([181,35.7,[0,0,0]])
    #roboList[0].newMarking([619,35.7,[0,0,0]])
    # Old right facing one
    #roboList.append(robotClass([759,585],'B'))
    #roboList[0].newMarking([394,802,[0,0,0]])
    #roboList[0].newMarking([392,367,[0,0,0]])
    #roboList[0].newMarking([1079,862,[0,0,0]])
    #roboList[0].newMarking([1078,304,[0,0,0]])

    #working one with 180 degrees
    roboList.append(robotClass([168.5,107.7],'B'))
    roboList[0].newMarking([60.8,276.2,[0,0,0]])
    roboList[0].newMarking([275.8,276.2,[0,0,0]])
    roboList[0].newMarking([0,0,[0,0,0]])
    roboList[0].newMarking([337,0,[0,0,0]])

    #should be working with -90 degrees
    roboList.append(robotClass([213,256],'B'))
    roboList[1].newMarking([110,85,[0,0,0]])
    roboList[1].newMarking([110,421,[0,0,0]])
    roboList[1].newMarking([384,148,[0,0,0]])
    roboList[1].newMarking([387,363,[0,0,0]])

    #should be working with 0 degrees
    roboList.append(robotClass([257,261],'B'))
    roboList[2].newMarking([148,88,[0,0,0]])
    roboList[2].newMarking([365,85,[0,0,0]])
    roboList[2].newMarking([87,363,[0,0,0]])
    roboList[2].newMarking([423,364,[0,0,0]])

    angles = angle()
    print("This is the angle of the robot->",angles)



if __name__== "__main__":
    main()
