import numpy
from numpy.linalg import multi_dot
#Mathematical model of a rotating mass

#This Kalman Filter filters out measurement noise and gives best estimate of where the robot is oriented. It is done in 1-d.
dt=0.1#time interval (should be frame rate)
u=1;#u is angular acceleration in this case. it should be calculated from outer loop controller output
F = numpy.matrix([[1,dt], [0,1]])#A Matrix (state transfer function) 
B=numpy.matrix([[dt**2/2], [dt]]) #B Matrix (control function)
X=numpy.matrix([[0],[0]])#initial state(belief values) (doesn't matter what it is.)
mu, sigma = 0, 5
noise=random.gauss(mu, sigma)#noise of angle from measurement (should be 1*1) assuming gaussian white noise with mean = 0 and variance of 10 degrees
P=numpy.matrix([[1,0],[0,1]])#state covariance matrix (doesn't matter what it is at the start as it will be iterated.)
Q=numpy.matrix([[0.01,0],[0 ,0.01]])#state predicted noise (when state transformation happens q(k) is process noise and covariance matrix is Q)
H=numpy.matrix([[1,0]])#observation matrix (only pos is observable, vel is not)
R=1#observation noise covariance 
for rob in roboList:
	Z=numpy.dot(H,X)+noise#i'm not sure if this is right. It should be directly from the measurement
	X_=numpy.dot(F,X)+numpy.dot(B,u)#predicted
	#update of the covariance matrix for predicted value
	P_=numpy.add(multi_dot([F,P,F.transpose()]),Q)
	K=numpy.dot(P_,H.transpose())/(multi_dot([H,P_,H.transpose()])+R);# calculating Kalman gain
	X=numpy.add(X_,K.dot(numpy.subtract(Z,H.dot(X_))))#belief which is a combination of estimation and measurement
	P=multi_dot([numpy.subtract(numpy.identity(2),K.dot(H)), P_])#update the covariance matrix











































dt=0.1#time interval (should be frame rate)
#Mathematical model of mass displacement

u=0#u is acceleration in this case. and it is supposed to get from the control in the controller
F = numpy.matrix([[1,dt], [0,1]])#A Matrix (state transfer function) 
B=numpy.matrix([[dt**2/2], [dt]]) #B Matrix (control function)
X=numpy.matrix([[0],[0]])#initial state(belief values) (doesn't matter what it is.)
mu, sigma = 0.5, 0.1
noise=random.gauss(mu, sigma)#noise of pos from measurement (should be 1*1)
#noise = randn(1)
P=numpy.matrix([[1,0],[0,1]])#state covariance matrix (doesn't matter what it is at the start as it will be iterated.)
Q=numpy.matrix([[0.01,0],[0 ,0.01]])#state predicted noise (when state transformation happens q(k) is process noise and covariance matrix is Q)
H=numpy.matrix([[1,0]])#observation matrix (only pos is observable, vel is not)
R=1#observation noise covariance 
for rob in roboList:
	Z=numpy.dot(H,X)+noise#i'm not sure if this is right should be directly from the measurement
	X_=numpy.dot(F,X)+numpy.dot(B,u)#predicted
	#update of the covariance matrix for predicted value
	P_=numpy.add(multi_dot([F,P,F.transpose()]),Q)
	K=numpy.dot(P_,H.transpose())/(multi_dot([H,P_,H.transpose()])+R);# calculating Kalman gain
	
	X=numpy.add(X_,K.dot(numpy.subtract(Z,H.dot(X_))))#belief which is a combination of estimation and measurement
	

	a = numpy.identity(4) 
	P=multi_dot([numpy.subtract(numpy.identity(2),K.dot(H)), P_])#update the covariance matrix
	
	#x1(i)=X(1);#put into array for plotting
	#x2=X_(1);#put into array for plotting

	#x4(i)=Z;

