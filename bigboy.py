import subprocess
#import simulation

print("BIG BOY RUNNING")
proc1 = subprocess.Popen('simulation.py', stdin=subprocess.PIPE, stdout=subprocess.PIPE, 
                         shell=True, bufsize=1)

inp = proc1.stdout.readline()

print(inp)
