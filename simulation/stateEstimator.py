import numpy as np
import math
import sys


class StateEstimator:
    #CONSTANTS for box length dimensions in cm
    XMAX = 50
    YMAX = 50
    #distance from center of car to wheel
    car_width = 3
    #time step
    dt = 0.25

    h_k = H_k = A = P_k_minus = u_k = Q_k = x_k_k_minus =obsv = F_k = R_k = Q_k = None
    def __init__(self, inputs, observations, state):
        h_k = geth()
        
        #P_kminus is the covariance matrix
        self.P_k_minus = np.array(
                           [[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0]])
        #Q_k is the covariance of process noise
        self.Q_k = np.array(
                    [[0.0921,0,     0,           0],
                     [0,     0.2509,0,            0],
                     [0,     0,     0.0054,       0],
                     [0,     0,     0,      0.0245]])
        #R_k is the covariance
        self.R_k = np.array(
            [[0.3909,0,0],
             [0,0.4664,0],
             [0,0,0.1749]])
        #state transition matrix
        rad = math.radians(x_k_k_minus[2][0])
        
    
    ########################    
    # Prediction functions #
    ########################
    def predict(self, inputs, state):
        #Get state information as inputs
        x0 = state[0][0]
        y0 = state[1][0]
        theta = state[2][0]
        thetad = state[3][0]

        #previous estimated state
        x_k_minus = np.array([x0],[y0],[theta],[thetad])

        #get inputs and form an input matrix
        #my_print('Enter pwm right')
        pwmR = inputs[1][0]
        #my_print('Enter pwm left')
        pwmL = inputs[0][0]

        self.u_k = np.array([[pwmR],[pwmL]])

        #A is the transition matrix for states
        self.A = np.array(
                    [[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,dt],
                     [0,0,0,0]])
        #B is the transition matrix for inputs
        self.B = np.array(
                    [[0.5*math.cos(theta), 0.5*math.cos(theta)],
                     [0.5*math.sin(theta), 0.5*math.sin(theta)],
                     [0,0],
                     [-1/(2*car_width), 1/(2*car_width)]])
        self.F_k = np.array(
                     [[1,0,-(self.u_k[0][0]+self.u_k[1][0])*math.sin(rad)*dt/2,0],
                      [0,1,(self.u_k[0][0]+self.u_k[1][0])*math.cos(rad)*dt/2,0],
                      [0,0,1,0],
                      [0,0,0,0]])
        #predicted state estimate
        self.x_k_k_minus = np.matmul(self.A,self.x_k_minus) + np.matmul(self.B,self.u_k)
        #predicted covariance estimate
        self.P_k_minus = np.matmul(np.transpose(self.F_k),np.matmul(self.F_k,self.P_k_minus))


    ##############################
    #UPDATE STATES               #
    ##############################
    #measurement
    def update(self, observations):

        #observations from laser and IMU
        dfront = observations[0][0]
        dright = observations[1][0]
        thetaIMU = observations[2][0]

        self.obsv = np.array([dfront],[dright],[thetaIMU])


        #get h and H for the predicted state
        self.h_k,self.H_k = gethandH()
        #calculating the measurement residual
        y_k = np.subtract(self.obsv, np.matmul(self.h,self.x_k_k_minus))
        #calculating innovation covariance
        S_k = np.add(np.matmul(np.transpose(self.H),np.matmul(self.H,self.P_k_minus)), self.R_k)
        #Kalman Gain
        K_k = np.matmul(np.linalg.inv(S_k), np.matmul(self.P_k_minus,np.transpose(H)))
        #Updated State Estimate
        x_k_k = np.add(self.x_k_k_minus,np.matmul(K_k,y_k))
        #Updated covariance estimate
        P_k_k = np.matmul(np.subtract(np.identity() - np.matmul(K_k,self.H)),P_k_minus)
        #set them to their new values
        self.P_k_minus = P_k_k

        return x_k_k

    #calculating H matrix
    def gethandH(self):
        s1 = 0.0
        s2 = 0.0
        rad = math.radians(x_k_k_minus[2][0])
        x = self.x_k_k_minus[0][0]
        y = self.x_k_k_minus[1][0]
        radd  = math.radians(x_k_k_minus[3][0])
        #calculating possible distances for front sensor
        d1 = (XMAX - x) / math.cos(rad)
        d2 = (YMAX - y) / math.sin(rad)
        d3 = x / math.cos(rad + math.pi)
        d4 = y / math.sin(rad + math.pi)
        #calculating possible distances for right sensor
        l1 = (XMAX - x) / math.sin(rad)
        l2 = (YMAX - y) / math.cos(rad + math.pi)
        l3 = x / math.sin(rad + math.pi)
        l4 = y / math.cos(rad)
        #get second least distance for both front and right distances
        s1 = sorted([d1,d2,d3,d4])[2]
        s2 = sorted([l1,l2,l3,l4])[2]

        h = None
        H = None
        if s1 == d1:
            h = np.array([[d1/x,0,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[-1/(math.cos(rad)),0,0,(XMAX-x_k_k_minus[0][0])*math.sin(rad)/(math.cos(rad)**2)],
                          [0,0,0,0],
                          [0,0,0,0]])
        elif s1 == d2:
            h = np.array([[0,d2/y,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[0,-1/math.sin(rad),0,(-1)*(YMAX-x_k_k_minus[1][0])*math.cos(rad)/(math.cos(rad)**2)],
                          [0,0,0,0],
                          [0,0,0,0]])
        elif s1 == d3:
            h = np.array([[d3/x,0,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[(1/(math.cos(rad+math.pi))),0,0,(x_k_k_minus[0][0])*math.sin(rad+math.pi)/(math.cos(rad+math.pi)**2)],
                          [0,0,0,0],
                          [0,0,0,0]])
        else:
            h = np.array([[0,d4/y,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.add(H,np.array([[0,1/(math.sin(rad+math.pi)),0,(-1)*(x_k_k_minus[1][0])*math.cos(rad+math.pi)/(math.sin(rad+math.pi)**2)],
                          [0,0,0,0],
                          [0,0,0,0]]))
        if s2 == l1:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [l1/x,0,0,0],
                          [0,0,0,0]]))
            H = np.add(H,np.array([[0,0,0,0],
                          [(-1/math.sin(rad)),0,0,(-1)*(XMAX-x_k_k_minus[0][0])*math.cos(rad)/(math.sin(rad+math.pi)**2)],
                          [0,0,0,0]]))
        elif s2 == l2:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [0,l2/y,0,0],
                          [0,0,0,0]]))
            H = np.add(H,np.array([[0,0,0,0],
                          [0,1/math.cos(rad + math.pi),0,(YMAX-x_k_k_minus[1][0])*math.sin(rad+math.pi)/(math.cos(rad+math.pi)**2)],
                          [0,0,0,0]]))
        elif s2 == l3:
            h = np.add(h,np.array([[0,0,0,0],
                          [l3/x,0,0,0],
                          [0,0,0,0]]))
            H = np.add(H,np.array([[0,0,0,0],
                          [1/math.sin(rad + math.pi),0,0,(-1)*(x_k_k_minus[0][0])*math.cos(rad+math.pi)/(math.sin(rad+math.pi)**2)],
                          [0,0,0,0]]))
        else:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [0,l4/y,0,0],
                          [0,0,0,0]]))
            H = np.add(H,np.array([[0,0,0,0],
                          [0,1/math.cos(rad),0,(x_k_k_minus[1][0])*math.sin(rad)/(math.cos(rad)**2)],
                          [0,0,0,0]]))
            
        return h,H