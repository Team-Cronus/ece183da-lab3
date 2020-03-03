import numpy as np
import sys

#CONSTANTS for box length dimensions in cm
L = 25
W = 25
#distance from center of car to wheel
car_width = 3

def my_print(s):
    print(s)
    sys.stdout.flush()


#Get state information as inputs
my_print('Enter initial x: ')
x0 = float(input())
my_print('Enter initial y:')
y0 = float(input())
my_print('Enter initial angle:')
theta = float(input())
my_print('Enter initial angle dot:')
thetad = float(input())

xk_minus1 = np.array([x0],[y0],[theta],[thetad])

#observations from laser and IMU
my_print('Enter front laser')
dfront = float(input())
my_print('Enter right laser')
dright = float(input())
my_print('Enter angle')
thetaIMU = float(input())

obsv = np.array([dfront],[dright],[thetaIMU])