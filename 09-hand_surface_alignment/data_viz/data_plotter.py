#!/usr/bin/env python

# read text files and plot them

import matplotlib.pyplot as plt
import numpy as np
import sys

# data file to read given as argument
if len(sys.argv) < 2:
	print "Give the name of the file to read as an argument\n"
	exit()

file = np.loadtxt(sys.argv[1] ,skiprows=1)

time = file[:,0]
sensed_force = file[:,1:4]
sensed_moment = file[:,4:7]
control_force = file[:,7:10]
control_moment = file[:,10:13]

plt.figure(1)
plt.plot(sensed_force,label="sensed Force")
plt.plot(control_force, label="control Force")
plt.legend()

plt.figure(2)
plt.plot(sensed_moment,label="sensed moment")
plt.plot(control_moment, label="control moment")
plt.legend()

plt.show()