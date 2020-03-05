from stateEstimator import StateEstimator
import tkinter
import math
import time as t1
import numpy as np
import matplotlib.pyplot as plt
import sys
import random

STOP = False
XMAX = 50 #30cm to signify the largest x value and the dimension of box
YMAX = 50 #30cm to signify the largest x value and the dimension of box
#initializing some global variables
u = [[0.0],                                       #u vector holds our input values(left velocity, right velocity)
     [0.0]]
t = .25                                         #t is a variable that holds a simulated time that considers
                                                #   the time that a velocity is acting on a wheel
dist_between_wheels = 1                         #distance between center of car and wheels
X = [[0.0],[0.0],[0.0],[0.0]]       #X is a column vector [xb,yb,theta,x',y',theta']
X_SE = [[0.0],[0.0],[0.0],[0.0]]                                    #copy of X for the state estimator
init_log_size = 500                             #initial size of the logging arrays
xlog = [None]*init_log_size                     #create three arrays to store the values for state column vector 
                                                #for logging and graphing purposes
ylog = [None]*init_log_size
thetalog = [None]*init_log_size
xSElog = [None]*init_log_size
ySElog = [None]*init_log_size
thetaSElog = [None]*init_log_size
index = 0;                                      #index for logging

Z = [[0.0],                                     #Z is a column vector that holds the output values of the system
     [0.0],                                     #output values: distance from lazer1, lazer2, angle from magnometer
     [0.0]]
v = 1.25                                         #v is the speed of the wheels in cm/s

#initializes the starting point of the robot where (0,0) is the bottom left corner
def initialize():
    global X, X_SE
    #gather initial inputs to realize where the car is
    print('Enter initial x position: ')
    sys.stdout.flush()
    xb = float(input())
    print('Enter initial y position: ')
    sys.stdout.flush()
    yb = float(input())
    print('Enter initial theta position: ')
    sys.stdout.flush()
    theta = float(input())
    X = np.array([[xb],[yb],[theta],[0]])
    print(X)
    X_SE = np.array([[xb],[yb],[theta],[0]])

#gets noise for inputs
def get_noise_in():
    a = np.array([[random.gauss(0,0.01)],
                   [random.gauss(0,.01)],
                   [random.gauss(0,0.02)],
                   [random.gauss(0,0.02)]])
    return a
#gets noise for observations
def get_noise_ob():
    a = np.array([[random.gauss(0,0.01)],
                   [random.gauss(0,0.01)],
                   [random.gauss(0,0.02)]])
    return a

def state_update():
    global X, u
    #global xlog, ylog, index
    rad = X[2][0]
    rad = math.radians(rad)
    X[2][0] = rad
    A = [[1,0,0,0],                     #A is the state transition matrix that updates the x, y, and
         [0,1,0,0],                      #theta positioning by adding the previous velocoties multiplied my time
         [0,0,1,t],
         [0,0,0,0]]
    #B is a state transition matrix that takes our input and 
     #converts into x,y,and angular velocities
    B = [[0.5*math.cos(X[2][0]),0.5*math.cos(X[2][0])], 
         [0.5*math.sin(X[2][0]),0.5*math.sin(X[2][0])],
         [0,0],
         [1 /(2*dist_between_wheels),-1/(2*dist_between_wheels)]]
    Bu = np.reshape(np.matmul(B,u),(4,1))                                   #compute the value of B*u for state update
    Ax = np.matmul(A,X)                                                     #compute value of A*x for state update
    Noise = get_noise_in()

    #get noise from gauss distribution
    X = np.add(np.add(Ax,Bu),Noise) 
    #compute state update with noise
    deg = X[2][0]
    deg = math.degrees(deg)
    X[2][0] = deg
    if X[2][0] <= 0:                                                    #keeps the angle constrained between 0 and 360
        X[2][0] += 360.0
    elif X[2][0] >= 360.0:
        X[2][0] -= 360.0

#getD is a function that first checks for the correct wall that the laser sensors
#will be hitting. It does this based on the known position,angle, and the min and max
#values of the x and y coordinates. More about this wall checking is in the report
#once we can distinguish which wall we are pointing towards(for both lasers),
#we then calculate the distance from the car to the wall using simple geometry
def getD():
    global X
    s1 = 0.0
    s2 = 0.0
    rad = math.radians(X[2][0])
    
    #calculating possible distances for front sensor
    d1 = (XMAX - X[0][0]) / math.cos(rad)
    d2 = (YMAX - X[1][0]) / math.sin(rad)
    d3 = X[0][0] / math.cos(rad + math.pi)
    d4 = X[1][0] / math.sin(rad + math.pi)
    #calculating possible distances for right sensor
    l1 = (XMAX - X[0][0]) / math.sin(rad)
    l2 = (YMAX - X[1][0]) / math.cos(rad + math.pi)
    l3 = X[0][0] / math.sin(rad + math.pi)
    l4 = X[1][0] / math.cos(rad)

    #Get the correct distance
    s1 = sorted([d1,d2,d3,d4])[2]
    s2 = sorted([l1,l2,l3,l4])[2]
    
    return s1,s2

#output functions
def output():
    global Z
    Z[0][0],Z[1][0] = getD()
    Z[2][0] = X[2][0]
    Noise = get_noise_ob()

    Z = np.add(Z,Noise)
    #prints Output: forward laser distance, right laser distance, angle reading
    print('Output: ', Z[0][0], Z[1][0], Z[2][0])
    sys.stdout.flush()


#driving imitation implementation
#left and write inputs are velocities of each wheel
#drive function will update 
def drive(left, right, cmd):
    global u     #updates velocity of wheels
    global X,X_SE,xlog,ylog,SElog,index    #updates graph
    global Z
    global SE
    #global running                              #indicates that car is running
    #running = True
    print("Input: ", cmd, left, right)
    sys.stdout.flush()
    u[1][0] = left
    u[0][0] = right
    #Generate predicted states inside State Estimator
    SE.predict(u,X_SE)
    #generate simulated actual state and output
    state_update()
    output()
    #Generate corrected states from State Estimator
    X_SE = SE.update(Z)
    #log 
    xlog[index] = X[0][0]
    ylog[index] = X[1][0]
    xSElog[index] = X_SE[0][0]
    ySElog[index] = X_SE[1][0]
    line.set_marker(marker=(3,0,X[2][0]+10))
    line.set_xdata(xlog)
    line.set_ydata(ylog)
    line2.set_marker(marker=(3,0,X_SE[2][0]+10))
    line2.set_xdata(xSElog)
    line2.set_ydata(ySElog)
    fig.canvas.draw()
    index += 1
    #log into file into csv
    #outputs: laser1, laser 2, angle
    f1.write(str(Z[0][0])+","+str(X[1][0])+","+str(X[2][0])+"\n")
    #inputs: vl, vr
    f2.write(str(u[0])+","+str(u[1])+"\n")
    #actual state:
    f3.write(str(X[0][0])+","+str(X[1][0])+","+str(X[2][0])+"\n")
    #SE state logging
    fSE.write(str(X_SE[0][0])+","+str(X_SE[1][0])+","+str(X_SE[2][0])+"\n")
  

def forward(event):
    drive(v,v, 'forward')                              #.01 meters per second speed, will need to update
def left(event):
    drive(-1*v,v,'left') 
def right(event):
    drive(v,-1*v,'right') 
def backward(event):
    drive(-1*v,-1*v,'reverse')
def stop(event):
    drive(0,0,'stop')

#function to handle when a button is released
def button_release(event):
    #global t
    global running
    running = False

########################
##BEGINNING OF SCRIPT ##
########################
#prompt for starting positions
initialize()
#initializing State Estimator object
SE = StateEstimator()
#create file to log data
f1 = open('sim_out.txt','w')
f2 = open('sim_in.txt','w')
f3 = open('sim_st.txt','w')
fSE = open('SE_st.txt','w')
#creating a gui for controlling the car using tkinter
#handles the drive functions and sets up a loop to run in
window = tkinter.Tk()
window.title("Robot GUI")
window.minsize(300,200)
fd = tkinter.Button (window,text='Forward')
fd.bind('<ButtonPress-1>', forward)
fd.bind('<ButtonRelease-1>', button_release)
rt = tkinter.Button (window,text='right')
rt.bind('<ButtonPress-1>', right)
rt.bind('<ButtonRelease-1>', button_release)
lt = tkinter.Button (window,text='left')
lt.bind('<ButtonPress-1>', left)
lt.bind('<ButtonRelease-1>', button_release)
bw = tkinter.Button (window,text='backward')
bw.bind('<ButtonPress-1>', backward)
bw.bind('<ButtonRelease-1>', button_release)
sp = tkinter.Button (window,text='stop')
sp.bind('<ButtonPress-1>', stop)
sp.bind('<ButtonRelease-1>', button_release)
ex = tkinter.Button (window,text='Exit', command=window.destroy)

fd.grid(column=5,row=0)
rt.grid(column=6,row=1)
lt.grid(column=4,row=1)
bw.grid(column=5,row=2)
sp.grid(column=5,row=1)
ex.grid(column=8,row=1)



#graphing setup             adapted from https://pythonspot.com/matplotlib-update-plot/
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line, = ax.plot(xlog,ylog, marker=(3,0,X[2][0]), label='Actual Position')
line2, = ax.plot(xSElog,ySElog, marker=(3,0,X_SE[2][0]), label='Estimated Sate')
plt.xlabel('x(cm)')
plt.ylabel('y(cm)')
plt.title('State Estimation')
plt.ylim(0,YMAX)                          #set the dimensions to be 0 to 50 cm
plt.xlim(0,XMAX)
plt.legend( loc='upper left')
window.mainloop()

plt.close(fig)