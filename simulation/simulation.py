import tkinter
import math
import time as t1
import numpy as np
import matplotlib.pyplot as plt
import sys

STOP = False
l1 = 2 #2cm from the center of rotation to the laser-y (change with measurements!)
l2 = 2 #2cm from the center of rotation to the laser-x (change with measurements!)
XMAX = 50 #30cm to signify the largest x value and the dimension of box
YMAX = 50 #30cm to signify the largest x value and the dimension of box
#initializing some global variables
running = False                                 #running is false when no button is pressed, 
                                                #   true when a button is pressed
u = [[0.0],                                       #u vector holds our input values(left velocity, right velocity)
     [0.0]]
t = .25                                         #t is a variable that holds a simulated time that considers
                                                #   the time that a velocity is acting on a wheel
dist_between_wheels = .5                        # 1cm, random value, will need to update later
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
    #TODO: change inputs to be adaptable as a child process
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

    #xb = float(sys.stdin.read())
    #yb = float(sys.stdin.read())
    #theta = float(sys.stdin.read())
    X = [[xb],[yb],[theta],[0.0],[0.0],[0.0]]
    #for ele in X:
     #   print('eleX: ', ele)


def state_update():
    global X, u
    #global xlog, ylog, index
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

    print('xp: ', X[0][0],' yp: ', X[1][0], 'theta: ', X[2][0], 'theta\': ', X[5][0])
    
    #index +=1

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

    #get minimum of the lengths, need to go back and view this
    s1 = sorted([d1,d2,d3,d4])[2]
    s2 = sorted([l1,l2,l3,l4])[2]
    

        
    return s1,s2

##output functions
def output():
    global Z
    Z[0][0],Z[1][0] = getD()
    Z[2][0] = X[2][0]

    #TODO: adjust for outputs to another program, maybe as a csv?
    #prints Output: forward laser distance, right laser distance, angle reading
    print('Output: ', Z[0][0], Z[1][0], Z[2][0])
    sys.stdout.flush()


#driving imitation implementation
#left and write inputs are velocities of each wheel
#drive function will update 
def drive(left, right, cmd):
    global u     #updates velocity of wheels
    global X,xlog,ylog,index    #updates graph
    global Z
    #global running                              #indicates that car is running
    #running = True
    print("Input: ", cmd, left, right)
    sys.stdout.flush()
    u[0] = left
    u[1] = right
    
    state_update()
    output()
    xlog[index] = X[0][0]
    ylog[index] = X[1][0]
    line.set_marker(marker=(3,0,X[2][0]+10))
    line.set_xdata(xlog)
    line.set_ydata(ylog)
    fig.canvas.draw()
    index += 1
    #log into file into csv
    #outputs: laser1, laser 2, angle
    f1.write(str(Z[0][0])+","+str(X[1][0])+","+str(X[2][0])+"\n")
    #inputs: vl, vr
    f2.write(str(u[0])+","+str(u[1])+"\n")
    #ideal state:
    f3.write(str(X[0][0])+","+str(X[1][0])+","+str(X[2][0])+"\n")



    

#TODO: Adjust for output to another program (csv maybe?)
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
    #print('hello')

#prompt for starting positions
initialize()

#create file to log data
f1 = open('sim_out.txt','w')
f2 = open('sim_in.txt','w')
f3 = open('sim_st.txt','w')
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
line, = ax.plot(xlog,ylog, marker=(3,0,X[2][0]))
plt.xlabel('x(cm)')
plt.ylabel('y(cm)')
plt.title('State Estimation')
plt.ylim(0,YMAX)                          #set the dimensions to be 0 to 50 cm
plt.xlim(0,XMAX)

window.mainloop()

plt.close(fig)