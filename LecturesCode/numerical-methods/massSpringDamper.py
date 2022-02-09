# Mass-Spring-Damper system
#
# Input: 
# - the integration step-size
# - end of the simulation time

import numpy as np
import matplotlib as plt
import matplotlib.pyplot as plt
import math
import sys

# get input
h = float(sys.argv[1])
tend = float(sys.argv[2])
pngYesNo = int(sys.argv[3])

# model parameters; it'd be nice to read from json file
mass = 10
stiff = 1000
damp = 150

def ydot(time, u, x):
    f0 = -(damp/mass)*u - (stiff/mass)*x #+ 10*math.sin(3.*time)/mass
    return np.array(f0, u)

tm = np.arange(0,tend+h,h)
y = np.zeros((2, tm.size))
y[0,0] = 5
y[1,0] = 0
for i in np.arange(1,tm.size):
    y[:, i] = y[:, i - 1] +  h * ydot(tm[i - 1], y[0, i - 1], y[1, i - 1])


# plot as png or in window?
plt.plot(tm,y[0,:])
if pngYesNo == 1:
    plt.savefig("plot.png")
else:
    plt.show()