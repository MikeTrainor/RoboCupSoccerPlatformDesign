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
        for circle in robot.circles:
            # Finding distance between each id to the robot centre
            xToCentre = circle[0] - robot.pos[0]
            yToCentre = circle[1] - robot.pos[1]

            angleToCentre = math.atan2(yToCentre,xToCentre)
            if angleToCentre < 0:
                angleToCentre = angleToCentre + 

            if(angleToCentre < 56 and angleToCentre > 45): # 56 deg -> 0.97738 rad and 45 deg -> 0.78540 rad
                topIDs.append(circle)

            if(angleToCentre >56 and angleToCentre < 65):
                bottomIDs.append(circle)

        if(len(topIDs) == 2):
            xDiff = topIDs[0][0] - topIDs[1][0]
            yDiff = topIDs[0][1] - topIDs[1][1]
            theta1 = math.atan2(yDiff,xDiff)

        if(len(bottomIDs) == 2):
            xDiff = bottomIDs[0][0] - topIDs[1][0]
            yDiff = bottomIDs[0][1] - topIDs[1][1]
            theta2 = math.atan2(yDiff,xDiff)

        theta = (theta1+theta2)/2
        angles.append(theta)
    return angles



def main():
    roboList.append(robotClass([759,585],'B'))
    roboList[0].newMarking([394,802,[0,0,0]])
    roboList[0].newMarking([392,367,[0,0,0]])
    roboList[0].newMarking([1079,862,[0,0,0]])
    roboList[0].newMarking([1078,304,[0,0,0]])
    angles = angle()
    print("This is the angle of the robot->",angles)



if __name__== "__main__":
    main()
