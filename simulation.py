import tkinter
import math
import time as t1
import numpy as np

#initializing some global variables
running = False                                 #running is false when no button is pressed, 
                                                #   true when a button is pressed
u = [[0.0],                                       #u vector holds our input values(left velocity, right velocity)
     [0.0]]
t = 0.6                                         #t is a variable that holds a simulated time that considers
                                                #   the time that a velocity is acting on a wheel
theta = 0                                       #theta holds value of the angle between body's y axis and fixed axis
dist_between_wheels = .010 # 1cm, random value, will need to update later
X = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]     #X is a column vector [xb,yb,theta,x',y',theta']

def initialize():
    global theta
    global X
    #gather initial inputs to realize where the car is
    xb = float(input('Enter initial x position: '))
    yb = float(input('Enter initial y position: '))
    theta = float(input('Enter initial angle: '))
    X = [[xb],[yb],[theta],[0.0],[0.0],[0.0]]
   
initialize()

def state_update():
    global t
    global theta
    global X
    global u
    A = [[1,0,0,t,0,0],                     #A is the state transition matrix that updates the x, y, and
         [0,1,0,0,t,0],                      #theta positioning by adding the previous velocoties multiplied my time
         [0,0,1,0,0,t],
         [0,0,0,0,0,0],
         [0,0,0,0,0,0],
         [0,0,0,0,0,0]]
    B = [[0,0],
         [0,0],
         [0,0],
         [0.5*math.cos(theta),0.5*math.cos(theta)],
         [0.5*math.sin(theta),0.5*math.sin(theta)],
         [-1 * dist_between_wheels,dist_between_wheels]]
    Bu = np.reshape(np.matmul(B,u),(6,1))
    Ax = np.matmul(A,X)
    X = np.add(np.matmul(A,X),np.reshape(np.matmul(B,u),(6,1)))
    #TODO: fix the matrix multiplacation and add on the output model
    for ele in Bu:
        print('eleBu: ', ele)
    for ele in Ax:
        print('eleAx: ', ele)
    for ele in X:
        print('eleX: ', ele)
#driving imitation implementation
def drive(left, right):
    global u                                    #updates velocity of wheels
    global running                              #indicates that car is running
    running = True
    u[0] = left
    u[1] = right
    state_update()

def forward(event):
    drive(1.0,1.0)                              #.01 meters per second speed
    print('forward')
def left(event):
    drive(-1.0,1.0) 
    print('left')
def right(event):
    drive(1.0,-1.0) 
    print('right')
def backward(event):
    drive(-1.0,-1.0)
    print('backward')
def stop(event):
    drive(0,0)
    print('stop')

#function to handle when a button is released
def button_release(event):
    global t
    global running
    running = False
    print('hello')

#creating a gui for controlling the car using tkinter
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


fd.grid(column=5,row=0)
rt.grid(column=6,row=1)
lt.grid(column=4,row=1)
bw.grid(column=5,row=2)
sp.grid(column=5,row=1)


window.mainloop()