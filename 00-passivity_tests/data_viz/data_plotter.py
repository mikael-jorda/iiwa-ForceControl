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
force = file[:,1:4]
desired_force = file[:,4:7]
Rc = file[:,7]

print np.mean(Rc)

plt.figure(1)
# plt.plot(time, force[:,2],'r',label="Force")
plt.plot(force[:,2],'r',label="Force")
# plt.plot(time, desired_force[:,2],'k', label="Desired Force")
plt.plot(desired_force[:,2],'k', label="Desired Force")
plt.legend()

plt.figure(2)
plt.plot(Rc)
plt.axis([0, np.size(Rc), -0.05, 1.05])

plt.show()