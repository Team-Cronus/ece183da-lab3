import tkinter
import math
import time as t1
import numpy as np
import matplotlib.pyplot as plt

l1 = 2 #2cm from the center of rotation to the laser-y (change with measurements!)
l2 = 2 #2cm from the center of rotation to the laser-x (change with measurements!)
XMAX = 50 #30cm to signify the largest x value and the dimension of box
YMAX = 50 #30cm to signify the largest x value and the dimension of box
#initializing some global variables
running = False                                 #running is false when no button is pressed, 
                                                #   true when a button is pressed
u = [[0.0],                                       #u vector holds our input values(left velocity, right velocity)
     [0.0]]
t = .25                                          #t is a variable that holds a simulated time that considers
                                                #   the time that a velocity is acting on a wheel
dist_between_wheels = .5                      # 1cm, random value, will need to update later
X = [[0.0],[0.0],[0.0],[0.0],[0.0],[0.0]]       #X is a column vector [xb,yb,theta,x',y',theta']
init_log_size = 500                             #initial size of the logging arrays
xlog = [None]*init_log_size                     #create two arrays to store the values for state column vector 
                                                #for logging and graphing purposes
ylog = [None]*init_log_size                                       
index = 0; #index for logging
Z = [[0.0],                                     #Z is a column vector that holds the output values of the system
     [0.0],                                     #output values: distance from lazer1, lazer2, angle from magnometer
     [0.0]]
v = 1.25                                         #v is the speed of the wheels in cm/s
#initializes the starting point of the robot where (0,0) is the bottom left corner
def initialize():
    global X
    #gather initial inputs to realize where the car is
    xb = float(input('Enter initial x position: '))
    yb = float(input('Enter initial y position: '))
    theta = float(input('Enter initial angle: '))
    X = [[xb],[yb],[theta],[0.0],[0.0],[0.0]]
    for ele in X:
        print('eleX: ', ele)

initialize()


#graphing setup             adapted from https://pythonspot.com/matplotlib-update-plot/
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
line, = ax.plot(xlog,ylog, marker=(3,0,X[2][0]))
plt.xlabel('x(cm)')
plt.ylabel('y(cm)')
plt.title('State Estimation')
plt.ylim(0,YMAX)                          #set the dimensions to be 0 to 50 cm
plt.xlim(0,XMAX)


def state_update():
    global X, u
    global xlog, ylog, index
    X[2][0] = math.radians(X[2][0])
    X[5][0] = math.radians(X[5][0])
    A = [[1,0,0,t,0,0],                     #A is the state transition matrix that updates the x, y, and
         [0,1,0,0,t,0],                      #theta positioning by adding the previous velocoties multiplied my time
         [0,0,1,0,0,t],
         [0,0,0,0,0,0],
         [0,0,0,0,0,0],
         [0,0,0,0,0,0]]
    B = [[0,0],                             #B is a state transition matrix that takes our input and converts into x,y,and angular velocities
         [0,0],
         [0,0],
         [0.5*math.cos(X[2][0]),0.5*math.cos(X[2][0])],
         [0.5*math.sin(X[2][0]),0.5*math.sin(X[2][0])],
         [-1 * dist_between_wheels,dist_between_wheels]]
    Bu = np.reshape(np.matmul(B,u),(6,1))                                   #compute the value of B*u for state update
    Ax = np.matmul(A,X)                                                     #compute value of A*x for state update
    X = np.add(Ax,Bu)
    X[5][0] = math.degrees(X[5][0])                                         #convert angular velocity for easier reading
    X[2][0] = math.degrees(X[2][0])
    if X[2][0] <= 0:                                                    #keeps the angle constrained between 0 and 360
        X[2][0] += 360.0
    elif X[2][0] >= 360.0:
        X[2][0] -= 360.0
    #for ele in X:
    #    print('eleX: ', ele)
    #logging and graphing 
    print('xp: ', X[0][0],' yp: ', X[1][0], 'theta: ', X[2][0], 'theta\': ', X[5][0])
    xlog[index] = X[0][0]
    ylog[index] = X[1][0]
    line.set_marker(marker=(3,0,X[2][0]+10))
    line.set_xdata(xlog)
    line.set_ydata(ylog)
    fig.canvas.draw()
    index +=1

#getD is a function that first checks for the correct wall that the laser sensors
#will be hitting. It does this based on the known position,angle, and the min and max
#values of the x and y coordinates. More about this wall checking is in the report
#once we can distinguish which wall we are pointing towards(for both lasers),
#we then calculate the distance from the car to the wall using simple geometry
def getD():
    global X
    D1 = 0.0
    D2 = 0.0
    rad = math.radians(X[2][0])
    #if the angle between the car and the magnetometer is between certain angles,
    #it has the ability to hit two different walls. We can check using the following calculation
    #more about the calculation will be on the report
    if X[2][0] < 90 and X[2][0] > 0:
        #Calculate D1, the distance from the laser in front of the car to the wall
        if math.atan2((YMAX-X[1][0]),(XMAX-X[0][0])) < rad:
            D1 = (YMAX - X[1][0])/math.sin(rad) - l1
        else:
            D1 = (XMAX - X[0][0])/math.cos(rad) - l1
        #Calculate D2, the distance from the laser on the right of the car to the wall
        if math.atan2((XMAX-X[0][0]),(X[1][0])) < (math.pi/2-rad):
            D2 = (XMAX - X[0][0])/math.sin(rad) - l2
        else:
            D2 = (-1 * X[1][0])/math.cos(rad) - l2
    elif X[2][0] < 180 and X[2][0] > 90:
        if math.atan2((YMAX-X[1][0]),(X[0][0])) < (math.pi - rad):
            D1 = (YMAX - X[1][0])/math.cos(rad - math.pi/2) - l1
        else:
            D1 = (X[0][0])/math.sin(rad - math.pi/2)-l1
        if math.atan2((YMAX-X[1][0]),(XMAX-X[0][0])) < (rad-math.pi/2):
            D2 = (YMAX-X[1][0])/math.sin(rad - math.pi/2) - l2
        else:
            D2 = (XMAX - X[0][0])/math.cos(rad - math.pi/2) - l2
    elif X[2][0] < 270 and X[2][0] > 180:
        if math.atan2((X[1][0]),(X[0][0])) < (rad-math.pi):
            D1 = (X[1][0])/math.sin(rad - math.pi)-l1
        else:
            D1 = (X[0][0])/math.cos(rad - math.pi)-l1
        if math.atan2((YMAX-X[1][0]),(X[0][0])) < (3*math.pi/2 - rad):
            D2 = (YMAX-X[1][0])/math.cos(rad - math.pi)-l2
        else:
            D2 = (X[0][0])/math.sin(rad - math.pi)-l2
    elif X[2][0] < 360 and X[2][0] > 270:
        if math.atan2((X[1][0]),(XMAX - X[0][0])) < (2*math.pi-rad):
            D1 = (X[1][0])/math.cos(2*math.pi - rad)-l1
        else:
            D1 = (XMAX-X[0][0])/math.sin(2*math.pi - rad)-l1
        if math.atan2((X[1][0]),(X[0][0])) < (rad - 3*math.pi/2):
            D2 = (X[1][0])/math.sin(2*math.pi - rad)-l2
        else:
            D2 = (X[0][0])/math.cos(2*math.pi - rad)-l2
    #if the angle is 90,180,270, or 0, we know which wall the lasers are sensing for sure
    elif X[2][0] == 90:
        D1 = YMAX - X[1][0]
        D2 = XMAX - X[0][0]
    elif X[2][0] == 180:
        D1 = X[0][0]
        D2 = YMAX - X[1][0]
    elif X[2][0] == 270:
        D1 = X[1][0]
        D2 = XMAX - X[0][0]
    elif X[2][0] == 0:
        D1 = XMAX - X[0][0]
        D2 = X[1][0]
    return D1,D2

##output functions
def output():
    global Z
    Z[0][0],Z[1][0] = getD()
    Z[2][0] = X[2][0]

    
    print('LzF: ', Z[0][0], ' LzR: ', Z[1][0], 'theta: ',Z[2][0])


#driving imitation implementation
#left and write inputs are velocities of each wheel
#drive function will update 
def drive(left, right):
    global u                                    #updates velocity of wheels
    #global running                              #indicates that car is running
    #running = True
    u[0] = left
    u[1] = right
    state_update()
    output()

def forward(event):
    drive(v,v)                              #.01 meters per second speed, will need to update
    print('forward')
def left(event):
    drive(-1*v,v) 
    print('left')
def right(event):
    drive(v,-1*v) 
    print('right')
def backward(event):
    drive(-1*v,-1*v)
    print('backward')
def stop(event):
    drive(0,0)
    print('stop')

#function to handle when a button is released
def button_release(event):
    #global t
    global running
    running = False
    #print('hello')

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