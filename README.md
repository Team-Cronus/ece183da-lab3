# LAB 3: RRT

# RRT Implementation 
In this lab we focus on implementing a rapidly-exploring random tree (RRT) algorithm. 
We have previous files from other labs in this repository as well as they are necessary
for the project to work. The main bulk of the RRT exists in the rrt folder. In the 
rrt folder you can find three other folders that seperate images of situations considered
in our program and how we handle them, along with the actual code implementation.
You can find the actual RRT code in the "new RRT code and data output" folder. The RRT.py
file is the script used to generate a path from the starting point and the goal. In this 
directory we also have images showing the results our paths. These images will be necessary
to showcase a proper path as it can take up to 30 minutes to generate a full and proper tree.

# Other Folders and Files
We also have a video of our robot car pathing through a created plan in the demo folder. In 
the video we use black tape to indicate the obstacles so we do not interrupt our observations.
The simulation folder contains the main simulation code and also an updated state estimator
implementation in python as opposed to our previous matlab version. You can run the 
simulation.py script to run a simulation with EKF. 

