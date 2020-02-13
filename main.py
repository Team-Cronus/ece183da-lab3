import subprocess
import time
#import simulation

print("BIG BOY RUNNING")
proc1 = subprocess.Popen(['python3','./simulation/simulation.py'], stdin=subprocess.PIPE, 
                         stdout=subprocess.PIPE, bufsize=1, 
                         encoding='ascii')#, shell=True)

#open communications for arduino



#open subprocess for matlab program
#proc3 = subprocess.Popen(matlab, stdin-subprocess.PIPE, stdout=subprocess.PIPE,bufsize=1,encoding='ascii')

#TODO: execute matlab program as well, then we can get lines from simulation
#       then send it into matlab program and also get matlab output
#TODO: make option to get data from arduino

while proc1.poll() is None: #and proc2.poll() is None
    if proc1.poll() is not None: #sim_output == '' 
        #print(proc1.poll())
        break
    sim_output = proc1.stdout.readline()        #gets stdout from simulation
    #ar_output = askdjfh from arduino
    #SE_output = proc3.stdout.readline()        #gets stdout from state estimator
    if sim_output:
        #print(sim_output, end = "")
        if 'Enter' in sim_output:
            print(sim_output, end = "")
            data = input()
            proc1.stdin.write(data+"\n")
            proc1.stdin.flush()
        if 'Output' in sim_output:  #take output values and send to SE
            obsv = (sim_output.split()[1],sim_output.split()[2],sim_output.split()[3]) #obsv = laser1 distance, laser2 distance, angle
            print(obsv)
            #write to matlab
        if 'Input' in sim_output:   #take input values and send to SE
            inps = (sim_output.split()[2], sim_output.split()[3])
            print(inps)
            #write to matlab
        #if 'State' in SE_output:               #prints state from state estimator
        #    print(SE_output)

    #print(proc1.poll())
print('end')
#if proc1.poll() is not None:


#for line in iter(proc1.stdout.readline, ''):
    #print(line.decode())

    #data = input()  
    #proc1.stdin.write(str.encode(data))
    #print('asdasd')
