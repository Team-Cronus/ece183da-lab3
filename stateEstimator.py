import numpy as np
import math
import sys

#CONSTANTS for box length dimensions in cm
XMAX = 50
YMAX = 50
#distance from center of car to wheel
car_width = 3
#time step
dt = 0.25
class StateEstimator:
    h = A = P_k_minus = u_k = Q = x_k_k_minus = None
    def __init__(self, inputs, observations):
        h = geth()
        #Get state information as inputs
        #my_print('Enter initial x: ')
        #x0 = float(input())
        #my_print('Enter initial y:')
        #y0 = float(input())
        #my_print('Enter initial angle:')
        theta = observations[2][0]
        #my_print('Enter initial angle dot:')
        #thetad = float(input())

        #
        x_k_minus = np.array([x0],[y0],[theta],[thetad])

        #observations from laser and IMU
       # my_print('Enter front laser')
        #dfront = float(input())
        #my_print('Enter right laser')
        #dright = float(input())
        #my_print('Enter angle')
        #thetaIMU = float(input())

        obsv = np.array([dfront],[dright],[thetaIMU])

        #get inputs and form an input matrix
        #my_print('Enter pwm right')
        pwmR = inputs[1][0]
        #my_print('Enter pwm left')
        pwmL = inputs[0][0]

        u_k = np.array([[pwmL],[pwmR]])

        #A is the transmission matrix for states
        A = np.array([[1,0,0,0],
                     [0,1,0,0],
                     [0,0,1,dt],
                     [0,0,0,0]])
        #B is the transition matrix for inputs
        B = np.array([[0.5*math.cos(theta), 0.5*math.cos(theta)],
                     [0.5*math.sin(theta), 0.5*math.sin(theta)],
                     [0,0],
                     [-1/(2*car_width), 1/(2*car_width)]])
        #P_kminus is the covariance matrix
        P_k_minus = np.array([[0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0],
                            [0,0,0,0]])
        #Q is the covariance of process noise
        Q = np.array([[0.0921,0,     0,      0],
                     [0,     0.2509,0,      0],
                     [0,     0,     0.0054, 0],
                     [0,     0,     0,      0.0245]])



        #Prediction functions
        x_k_k_minus = np.matmul(A,x_k_minus) + np.matmul(B,u_k)
        P_k_minus = np.matmul(np.matmul(A,P_k_minus),np.transpose(A))

        #state transition matrix
        rad = math.radians(x_k_k_minus[2][0])
        F = np.array([[1,0,-(u_k[0][0]+u_k[1][0])*math.sin(rad)*dt/2,0],
                      [0,1,(u_k[0][0]+u_k[1][0])*math.cos(rad)*dt/2,0],
                      [0,0,1,0],
                      [0,0,0,0]])
        #gets observation matrix
        h,H = getH(s1,s2)




    def my_print(s):
        print(s)
        sys.stdout.flush()

    #calculating H matrix
    def gethandH():
        s1 = 0.0
        s2 = 0.0
        rad = math.radians(x_k_k_minus[2][0])
        x = x_k_k_minus[0][0]
        y = x_k_k_minus[1][0]
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
            H = np.array([[-1/(math.cos(rad)),0,0,0],
                          [0,0,0,0],
                          [0,0,0,0]])
        elif s1 == d2:
            h = np.array([[0,d2/y,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[0,-1/math.sin(rad),0,0],
                          [0,0,0,0],
                          [0,0,0,0]])
        elif s1 == d3:
            h = np.array([[d3/x,0,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[(1/(math.cos(rad+math.pi))),0,0,0],
                          [0,0,0,0],
                          [0,0,0,0]])
        else:
            h = np.array([[0,d4/y,0,0],
                          [0,0,0,0],
                          [0,0,1,0]])
            H = np.array([[0,1/(math.sin(rad+math.pi)),0,0],
                          [0,0,0,0],
                          [0,0,0,0]])
        if s2 == l1:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [l1/x,0,0,0],
                          [0,0,0,0]]))
            H = np.array([[0,0,0,0],
                          [(-1/math.sin(rad),0,0,0],
                          [0,0,0,0]])
        elif s2 == l2:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [0,l2/y,0,0],
                          [0,0,0,0]]))
            H = np.array([[0,0,0,0],
                          [0,1/math.cos(rad + math.pi),0,0],
                          [0,0,0,0]])
        elif s2 == l3:
            h = np.add(h,np.array([[0,0,0,0],
                          [l3/x,0,0,0],
                          [0,0,0,0]]))
            H = np.array([[0,0,0,0],
                          [1/math.sin(rad + math.pi),0,0,0],
                          [0,0,0,0]])
        else:
            h = np.add(h,np.array([
                          [0,0,0,0],
                          [0,l4/y,0,0],
                          [0,0,0,0]]))
            H = np.array([[0,0,0,0],
                          [0,1/math.cos(rad),0,0],
                          [0,0,0,0]])
            
        return h,H