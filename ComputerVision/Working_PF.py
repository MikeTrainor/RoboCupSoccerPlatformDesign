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

## Particle Filter Parameters
height = 480
width = 640
#Filter parameters
tot_particles = 100
#Standard deviation which represent how to spread the particles
#in the prediction phase.
std = 25 
#Init the filter
my_particle = ParticleFilter(height, width, tot_particles)
#Black image, window, bind the function
img = np.zeros((height,width,3), np.uint8)


##
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




def PF_predict_center(img):
    for robot in roboList:
        x= robot.pos[0]
        y= robot.pos[1]


        my_particle.predict(x_velocity=0, y_velocity=0, std=std)

        #Estimate the next position using the internal model
        x_estimated, y_estimated, _, _ = my_particle.estimate() 
 
        #Update the position of the particles based on the measurement.
        #Adding some noise to the measurement.
        noise_coefficient = np.random.uniform(low=0.0, high=10.0)
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
            noise_coefficient = np.random.uniform(low=0.0, high=10.0)
            x_measured = x + np.random.randn() * noise_coefficient
            y_measured = y + np.random.randn() * noise_coefficient
            my_particle.update(x_measured, y_measured)

            #Drawing the circles for the mouse position the
            #estimation and the particles.
            #for i in range(0, tot_particles):
            #    x_particle, y_particle = my_particle.returnParticlesCoordinates(i)
            #    cv2.circle(img,(x_particle, y_particle),2,(0,0,255),-1) #RED: Particles
            #cv2.circle(img,(x, y),2,(0,255,0),-1) #GREEN: Mouse position
            #cv2.circle(img,(x_estimated, y_estimated),2,(255,0,0),-1) #BLUE: Filter estimation

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

    #Second part: recognizing circles in stream of images

    cap = cv2.VideoCapture(0)

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
      #  cv2.imshow("masked image",result1)
        # result2 = cv2.bitwise_and(frame,frame,mask=mask2)
        #FinalResult = cv2.bitwise_or(result1,result2)

  
        #hsv_out = np.bitwise_or(FinalResult, edged[:,:,np.newaxis])
        hsv_out_gray= cv2.cvtColor(result1, cv2.COLOR_BGR2GRAY)
      #  cv2.imshow("masked image2",hsv_out_gray)
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
                PF_predict_center(img)
                PF_predict_IDs()
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
            
        
        else:
            print("no circles detected")

        
       
       
        
        cv2.imshow('circles on stream',img)
    #    cv2.imshow('original stream',frame)
    #     cv2.waitKey(250)
        if cv2.waitKey(1) & 0xFF == ord('\r'):
            break


    cv2.destroyAllWindows(0)

        #if cv2.waitKey(1) & 0xFF == ord('\r'):
        #    #cv2.destroyAllWindows()
        #    break
 
 
if __name__== "__main__":
    main()



