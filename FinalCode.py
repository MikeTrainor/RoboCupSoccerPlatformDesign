import numpy as np
from scipy.spatial import distance as dist
import copy
import math
import cv2
import serial
import time


KL25=serial.Serial('COM4',9600,timeout=1)#open serial port
# two Lists used for real time plottting of the left motor and right motor
Rmotor= []
Lmotor= []
transmit= [0,0,0,0,0,0,0,0]

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





    

    

def colorID(hue, sat, val):
    color = 'X' # Default case, 'X' will print an error for an unrecognized element
    if(val > 40):
        if (hue < 137 and hue >= 90):
            color = 'B' # Blue team circle
        elif (hue < 35 and hue > 20):
            color = 'Y' # Yellow team circle
        elif (hue >= 148 or (hue <= 8 and sat < 120)): # Must address loop in hue
            color = 'P' # Purple ID circle
        elif (hue < 90 and hue >= 43):
            color = 'G' # Green ID circle
        elif (hue <= 20 and hue >= 3):
            color = 'O' # Ball!
        else:
            print(hue,sat,val) # good for debugging unrecognized circles

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
    return

def main():
    # Declaring global variables so they can be cleared every loop
    global roboList
    global roboIDmarks
    global circles 
    global ball
    global IDdRobots
    
    
    ## PD controller Parametters
    error1=0
    error_prior1=0
    error2=0
    error_prior2=0
    ## The dt variable should be variable based on the number of frame rates obtained by the CV system. 
    ####### This frame rate should be calculated. #########
    dt=1/10             
    derivative1=0       
    L=18                #Robot Diameter
    R=3.5               #Wheel Radius
    umax=575            #Max input for position control
    u2max= 220
    kp1= 0.65
    kp2=kp1
    flag =  0
    kd1=0.5
    kd2=kd1
   
   
    cap = cv2.VideoCapture(cv2.CAP_DSHOW + 1) # 0 if your pc doesn't have a webcam, probably 1 if it does
    # https://stackoverflow.com/questions/52043671/opencv-capturing-imagem-with-black-side-bars
    # MSMF doesn't like being scaled up apparently, so switch from it (default) to DirectShow
    # so we can scale up the resolution read from the camera

    # Scaling up from 640x480 to HD 1280x720
    #cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    #cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)


    while(True):
      #  while(KL25.inWaiting()==0):

        ret,frame = cap.read() # reading the video capture into a dummy var and frame

        #cv2.waitKey(50)
        
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

        # blurring image for less errant circles and better color recognition later
        # d = 5 as that is the recommended nearest neighbour for real time
        # sigmaColor = 150 to produce large blending effect
        # sigmaSpace is limited by d, so I suspect it doesn't matter
        blurred_img = cv2.bilateralFilter(frame,5,150,150) 

        # HSV color space conversion
        hsv= cv2.cvtColor(blurred_img,cv2.COLOR_BGR2HSV)

        # Color masking, not necessary due to blurring, but might be worth looking into further
        #lower_rangeG = np.array([0,0,0]) # Hue, Saturation, Value mask lower limit
        #upper_rangeG = np.array([180,255,255]) # " , " , " " upper limit

        #mask = cv2.inRange(hsv, lower_rangeG, upper_rangeG) # mask for original frame with only good color
        #result = cv2.bitwise_and(blurred_img,blurred_img,mask=mask)
        result = blurred_img

        cv2.imshow("blurred image",result)
  
        hsv_out_gray= cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        # Some notes on the HoughCircles function:
        #  Utilizes edge detection to draw tangent lines, recognizing a circle where perpendicular lines to tangents
        #  meet, depending on the intensity of the intersecting tangent lines.
        #  param1: higher threshold for Canny edge detection (lower is half of this)
        #  param2: accumulator threshold for circle center detection- i.e. the lower it is, the less circular an object
        #          needs to be to be recognized as a circle
        #  minDist: Specifies minimum distance between circles (the 4th input to the function)
        #  
        # from documentation: cv2.HoughCircles(image, method, dp, minDist[, circles[, param1[, param2[, minRadius[, maxRadius]]]]]) → circles
        circles = cv2.HoughCircles(hsv_out_gray,cv2.HOUGH_GRADIENT,1,minDist = 20,param1=75,param2=27,minRadius=3,maxRadius=15)

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
                    if isinstance(robot.angle, type(None)) == 0:
                        # Display the robot's angle
                        cv2.putText(img, str(round(robot.angle,1)), (robot.pos[0]+ 100, robot.pos[1] + 130), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                        # Display the robot's position
                        cv2.putText(img, str(robot.pos), (robot.pos[0]+ 100, robot.pos[1] + 100), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                        # Display the robot's ID
                        cv2.putText(img, robot.ID, (robot.pos[0]+ 100, robot.pos[1] + 70), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                        # Display the robot's Team
                        cv2.putText(img, robot.team, (robot.pos[0]+ 100, robot.pos[1] + 40), 
                                    cv2.FONT_HERSHEY_DUPLEX, 1, (255,255,255), 3)
                    for mark in robot.circles:
                        # Draw a black circle on every ID mark
                        cv2.circle(img,(mark[0],mark[1]),10,(0,0,0),3)  
            flag = 0 # go ahead and print "no circles detected" again

        elif(flag == 0):
            print("no circles detected")
            flag = 1 # don't print this again

        # Display drawn on frame and original frame
        cv2.imshow('circles on stream',img)
        cv2.imshow('original stream',frame)

        if cv2.waitKey(1) & 0xFF == ord('\r'): # if enter is pressed, stop running
            break

        test = 100

        
        packet = bytearray()
        packet.append(0xff)
        packet.append(0x01)  #id
        packet.append(0x20)  #mtr1
        packet.append(0x01)  #dir1
        packet.append(0xdf)  #mtr2
        packet.append(0x01)  #dir2
        packet.append(0x01)  #kick
        packet.append(0xff)

        KL25.write(packet)
        data = KL25.read(4)
        print(data.decode('ISO-8859-1'))

        #for rob in roboList:
        #    if isinstance(ball, type(None)) == 0:
        #        #Error In Position
        #        ####### Position Control #######
        #        error1 = ((ball.pos[0]-rob.pos[0])**2+(ball.pos[1]-rob.pos[1])**2)**0.5
        #        derivative1=(error1-error_prior1)/dt
        #        error_prior1=error1
        #        u1=  ( kp1*error1 )   +  ( kd1*derivative1 )


        #        ####### Angle Control #######
        #        error2= (180+57.2958*(np.arctan((ball.pos[1]-rob.pos[1])/(ball.pos[0]-rob.pos[0]))))-rob.angle
        #        #print(180+57.2958*(np.arctan((ball.pos[1]-rob.pos[1])/(ball.pos[0]-rob.pos[0]))),rob.angle)
        #        derivative2=(error2-error_prior2)
        #        error_prior2=error2
        #        u2=  (kp2*error2)   +   (kd2*derivative2)
             
        #        ## Setting limits to the inputs 
        #        if(u1 > umax):
        #            u1=umax
        #        if(u1 < -umax):
        #            u1 = -umax

        #        if(u2 > u2max):
        #            u2=u2max
        #        if(u2 < -u2max):
        #            u2 = -u2max
        #        ## Normalizing the inputs to be of similar magnitude
        #        upos= u1*500/umax
        #        utheta = u2*500/u2max
                                
        #        # Assigning Individual Wheel velocities
        #        vr=(2*u1+u2*L)/(2*R)
        #        vl=(2*u1-u2*L)/(2*R)
                
        #        # Assigning the direction of motors based on the wheel velocities sign

        #        if(np.sign(vr) == 1):
        #            dirR= 0x00
        #        if(np.sign(vr) == -1):
        #            dirR= 0x01

        #        if(np.sign(vl) == 1):
        #            dirR= 0x00

        #        if(np.sign(vl) == -1):
        #            dirR= 0x01

        #        # Remove the sign in motor velocities
        #        Vr = chr(abs(vr))
        #        Vl = chr(abs(vl))
        #        # Assign the motor velocities to 0-256 range to send through 8bit UART
        #        VrHex = chr(Vr*0xFF/ umax)
        #        VlHex = chr(Vl*0xFF/ umax)

        #        if (abs(error1) < 10 and abs(error2) <5): 
        #            kick= 0x01
        #        else:
        #            kick = 0

        #        transmit[0]= 0xFF
        #        transmit[1]= 0x01
        #        transmit[2]= 0xa0 #VAHex
        #        transmit[3]= 0x01 #dirR
        #        transmit[4]= 0xdf #VlHex
        #        transmit[5]= 0x00 #dirL
        #        transmit[6]= 0x01 #kick
        #        transmit[7]= 0xFF

        #        for i in range(1,len(transmit)):
        #            KL25.write(transmit[i-1])




    cv2.destroyAllWindows()
 

if __name__== "__main__":
    main()
